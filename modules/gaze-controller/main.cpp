/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <mutex>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <cmath>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/iKin/iKinFwd.h>
#include <cer_kinematics/head.h>

#define MIN_TS  0.01    // [s]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace cer::kinematics;

// forward declaration
class Controller;

/****************************************************************/
class TargetPort : public BufferedPort<Property>
{
    Controller *ctrl;

    /****************************************************************/
    void onRead(Property &request);

public:
    /****************************************************************/
    TargetPort() : ctrl(NULL) { }

    /****************************************************************/
    void setController(Controller *ctrl) { this->ctrl=ctrl; }
};


/****************************************************************/
class Controller : public RFModule
{
    PolyDriver        drivers[3];
    IEncodersTimed*   ienc[3];
    IControlMode*     imod;
    IPositionControl* ipos;
    IPositionDirect*  iposd;

    vector<int> posDirectMode;
    vector<int> curMode;

    map<string,HeadSolver> solver;
    map<string,Matrix> intrinsics;
    minJerkTrajGen* gen;
    
    BufferedPort<Property> statePort;
    TargetPort targetPort;
    RpcServer rpcPort;
    Stamp txInfo;

    mutex mtx;
    int verbosity;
    bool closing;
    bool controlling;
    set<string> avFrames;
    string control_frame;
    double stop_threshold;
    double Ts;

    Vector qd;
    Vector pitchPhy;
    Vector yawPhy;

    /****************************************************************/
    void getIntrinsics(ResourceFinder &rf)
    {
        for (set<string>::iterator it=avFrames.begin(); it!=avFrames.end(); it++)
        {
            const string &camera=*it;
            yInfo("#### Retrieving intrinsics for \"%s\" camera",camera.c_str());

            string groupName=camera;
            transform(groupName.begin(),groupName.end(),groupName.begin(),::toupper);
            groupName="CAMERA_CALIBRATION_"+groupName;

            bool ok=false;
            Bottle &group=rf.findGroup(groupName);
            if (!group.isNull())
            {
                if (group.check("fx") && group.check("fy") &&
                    group.check("cx") && group.check("cy"))
                {
                    Matrix K=eye(3,4);
                    K(0,0)=group.find("fx").asFloat64();
                    K(1,1)=group.find("fy").asFloat64();
                    K(0,2)=group.find("cx").asFloat64();
                    K(1,2)=group.find("cy").asFloat64();
                    
                    yInfo("%s",K.toString(3,3).c_str());
                    intrinsics[camera]=pinv(K.transposed()).transposed(); 
                    ok=true;
                }
            }

            if (!ok)
                yWarning("Intrinsics for \"%s\" camera not configured!",camera.c_str());
        }
    }

    /****************************************************************/
    Vector getEncoders(double *timeStamp=NULL)
    {
        Vector encs(6,0.0);
        Vector stamps(encs.length());

        Vector encs_=encs;
        Vector stamps_=stamps;

        ienc[0]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(0,encs_.subVector(0,2));
        stamps.setSubvector(0,stamps_.subVector(0,2));

        ienc[1]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs[3]=encs_[3];
        stamps[3]=stamps_[3];

        ienc[2]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(4,encs_.subVector(0,1));
        stamps.setSubvector(4,stamps_.subVector(0,1));

        if (timeStamp!=NULL)
            *timeStamp=findMax(stamps);

        return encs;
    }

    /****************************************************************/
    void getCurrentMode()
    {
        imod->getControlModes(curMode.data());
    }

    /****************************************************************/
    bool areJointsHealthy()
    {
        for (size_t i=0; i<curMode.size(); i++)
            if ((curMode[i]==VOCAB_CM_HW_FAULT) ||
                (curMode[i]==VOCAB_CM_IDLE))
                return false;
        return true;
    }

    /****************************************************************/
    void setPositionDirectMode()
    {
        for (size_t i=0; i<curMode.size(); i++)
        {
            if (curMode[i]!=posDirectMode[i])
            {
                imod->setControlModes(posDirectMode.data());
                break;
            }
        }        
    }

    /****************************************************************/
    void stopControl()
    {        
        ipos->stop();
        controlling=false;
    }

    /****************************************************************/
    void getBounds(PolyDriver &driver, Matrix &lim) const
    {
        IEncoders      *ienc;
        IControlLimits *ilim;

        driver.view(ienc);
        driver.view(ilim);

        int nAxes;
        ienc->getAxes(&nAxes);

        lim.resize(nAxes,2);
        for (int i=0; i<nAxes; i++)
            ilim->getLimits(i,&lim(i,0),&lim(i,1));
    }

    /****************************************************************/
    void alignJointsBounds()
    {
        for (map<string,HeadSolver>::iterator it=solver.begin();
             it!=solver.end(); it++)
        {            
            yInfo("##### Aligning joints bounds for control frame \"%s\"",
                  it->first.c_str());
            HeadSolver &s=it->second;

            HeadParameters p=s.getHeadParameters();
            iKinChain &chain=*p.head.asChain();
            Matrix lim;
            Value part;

            getBounds(drivers[1],lim);
            p.torso.l_min=lim(0,0);
            p.torso.l_max=lim(0,1);
            p.torso.alpha_max=fabs(lim(1,1));

            chain[0].setMin(CTRL_DEG2RAD*lim(3,0));
            chain[0].setMax(CTRL_DEG2RAD*lim(3,1));

            part=drivers[1].getValue("remote");
            yInfo("limits of %s part: heave=[%g,%g] [m], [pitch,roll]=[%g,%g] [deg], yaw=[%g,%g] [deg]",
                  part.asString().c_str(),p.torso.l_min,p.torso.l_max,-p.torso.alpha_max,p.torso.alpha_max,lim(3,0),lim(3,1));

            getBounds(drivers[2],lim);
            part=drivers[2].getValue("remote");

            for (int i=0; i<2; i++)
            {
                chain[1+i].setMin(CTRL_DEG2RAD*lim(i,0));
                chain[1+i].setMax(CTRL_DEG2RAD*lim(i,1));

                yInfo("limits of %s part: joint %d=[%g,%g] [deg]",
                      part.asString().c_str(),i,CTRL_RAD2DEG*chain[1+i].getMin(),
                      CTRL_RAD2DEG*chain[1+i].getMax());
            }

            s.setHeadParameters(p);
        }
    }

    /****************************************************************/
    void getJointsBounds(Vector &pitchLim, Vector &yawLim)
    {
        HeadSolver &s=solver.begin()->second;
        HeadParameters p=s.getHeadParameters();
        iKinChain &chain=*p.head.asChain();

        pitchLim.resize(2);
        pitchLim[0]=CTRL_RAD2DEG*chain[1].getMin();
        pitchLim[1]=CTRL_RAD2DEG*chain[1].getMax();

        yawLim.resize(2);
        yawLim[0]=CTRL_RAD2DEG*chain[2].getMin();
        yawLim[1]=CTRL_RAD2DEG*chain[2].getMax();
    }

    /****************************************************************/
    void applyCustomJointsBounds(const Bottle *pitchLim,
                                 const Bottle *yawLim)
    {
        Value part=drivers[2].getValue("remote");
        for (map<string,HeadSolver>::iterator it=solver.begin();
             it!=solver.end(); it++)
        {
            yInfo("##### Applying custom joints bounds for control frame \"%s\"",
                  it->first.c_str());
            HeadSolver &s=it->second;

            HeadParameters p=s.getHeadParameters();
            iKinChain &chain=*p.head.asChain();            

            int i=0;
            if (pitchLim!=NULL)
            {
                bool doPrint=false;
                if (pitchLim->size()>0)
                {
                    double val=pitchLim->get(0).asFloat64();
                    val=std::min(std::max(val,pitchPhy[0]),pitchPhy[1]);
                    chain[1+i].setMin(CTRL_DEG2RAD*val);
                    doPrint=true;
                }

                if (pitchLim->size()>1)
                {
                    double val=pitchLim->get(1).asFloat64();
                    val=std::min(std::max(val,pitchPhy[0]),pitchPhy[1]);
                    chain[1+i].setMax(CTRL_DEG2RAD*val);
                    doPrint=true;
                }

                if (doPrint)
                    yInfo("limits of %s part: joint %d=[%g,%g] [deg]",
                          part.asString().c_str(),i,CTRL_RAD2DEG*chain[1+i].getMin(),
                          CTRL_RAD2DEG*chain[1+i].getMax());
            }

            i++;
            if (yawLim!=NULL)
            {
                bool doPrint=false;
                if (yawLim->size()>0)
                {
                    double val=yawLim->get(0).asFloat64();
                    val=std::min(std::max(val,yawPhy[0]),yawPhy[1]);
                    chain[1+i].setMin(CTRL_DEG2RAD*val);
                    doPrint=true;
                }

                if (yawLim->size()>1)
                {
                    double val=yawLim->get(1).asFloat64();
                    val=std::min(std::max(val,yawPhy[0]),yawPhy[1]);
                    chain[1+i].setMax(CTRL_DEG2RAD*val);
                    doPrint=true;
                }

                if (doPrint)
                    yInfo("limits of %s part: joint %d=[%g,%g] [deg]",
                          part.asString().c_str(),i,CTRL_RAD2DEG*chain[1+i].getMin(),
                          CTRL_RAD2DEG*chain[1+i].getMax());
            }

            s.setHeadParameters(p);
        }
    }

    /****************************************************************/
    void fillState(const Vector &q, Property &state)
    {
        state.clear();
        for (set<string>::iterator it=avFrames.begin(); it!=avFrames.end(); it++)
        {
            const string &frame=*it;

            Matrix Hee;
            solver[frame].fkin(q,Hee);

            Vector pose=cat(Hee.getCol(3).subVector(0,2),
                            dcm2axis(Hee));

            Bottle val; val.addList().read(pose);
            state.put(frame,val.get(0));

            if (frame==control_frame)
            {
                Vector ang(2);
                Vector z=Hee.getCol(2);
                ang[0]=CTRL_RAD2DEG*atan2(z[1],z[0]);
                ang[1]=CTRL_RAD2DEG*atan2(z[2],z[0]);

                Bottle val; val.addList().read(ang);
                state.put("angular",val.get(0));
            }
        }
    }

public:
    /****************************************************************/
    Controller() : gen(NULL), closing(false), controlling(false)
    {
        avFrames=HeadParameters::getTypes();
        for (set<string>::iterator it=avFrames.begin(); it!=avFrames.end(); it++)
        {
            const string &frame=*it;
            HeadParameters p(frame);
            solver[frame].setHeadParameters(p);
        }
        control_frame="gaze"; 
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("cer")).asString();
        bool get_bounds=(rf.check("get-bounds",Value("on")).asString()=="on");
        verbosity=rf.check("verbosity",Value(0)).asInt32();
        stop_threshold=rf.check("stop-threshold",Value(2.0)).asFloat64();
        double T=rf.check("T",Value(1.0)).asFloat64();
        Ts=rf.check("Ts",Value(MIN_TS)).asFloat64();
        Ts=std::max(Ts,MIN_TS);

        Bottle *pitchLim=NULL; Bottle *yawLim=NULL;
        Bottle &jointsLimits=rf.findGroup("joints-limits");
        if (!jointsLimits.isNull())
        {
            pitchLim=jointsLimits.find("pitch").asList();
            yawLim=jointsLimits.find("yaw").asList();
        }

        Property option;

        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/torso_tripod");
        option.put("local","/cer_gaze-controller/torso_tripod"); 
        if (!drivers[0].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/torso_tripod").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/torso");
        option.put("local","/cer_gaze-controller/torso");
        if (!drivers[1].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/torso").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/head");
        option.put("local","/cer_gaze-controller/head");
        option.put("writeStrict","on");
        if (!drivers[2].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/head").c_str());
            close();
            return false;
        }

        drivers[0].view(ienc[0]);
        drivers[1].view(ienc[1]);
        drivers[2].view(ienc[2]);
        drivers[2].view(imod);
        drivers[2].view(ipos);
        drivers[2].view(iposd);

        statePort.open("/cer_gaze-controller/state:o");
        targetPort.open("/cer_gaze-controller/target:i");
        targetPort.setController(this);
        targetPort.useCallback();

        rpcPort.open("/cer_gaze-controller/rpc");
        attach(rpcPort);

        int nAxes;
        ienc[2]->getAxes(&nAxes);
        for (int i=0; i<nAxes; i++)
            posDirectMode.push_back(VOCAB_CM_POSITION_DIRECT);
        curMode=posDirectMode;

        Vector q=getEncoders();
        qd=q.subVector(4,5);

        getCurrentMode();

        if (get_bounds)
            alignJointsBounds();
        getJointsBounds(pitchPhy,yawPhy);
        applyCustomJointsBounds(pitchLim,yawLim);

        string cameras_context="cameraCalibration";
        string cameras_file="cer.ini";
        Bottle &camerasGroup=rf.findGroup("cameras");
        if (!camerasGroup.isNull())
        {
            cameras_context=camerasGroup.check("context",Value(cameras_context)).asString();
            cameras_file=camerasGroup.check("file",Value(cameras_file)).asString();
        }

        ResourceFinder rf_cameras;
        rf_cameras.setDefaultContext(cameras_context);
        rf_cameras.setDefaultConfigFile(cameras_file.c_str());
        rf_cameras.configure(0,NULL);
        getIntrinsics(rf_cameras);

        for (auto &s:solver)
            s.second.setVerbosity(verbosity);
        gen=new minJerkTrajGen(qd,Ts,T);

        return true;
    }

    /****************************************************************/
    bool close()
    {
        closing=true;

        if (controlling)
            stopControl();

        if (!targetPort.isClosed())
            targetPort.close(); 

        if (!statePort.isClosed())
            statePort.close();

        if (!rpcPort.asPort().isOpen())
            rpcPort.close(); 
        
        for (int i=0; i<3; i++)
            if (drivers[i].isValid())
                drivers[i].close(); 

        delete gen;
        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return Ts;
    }

    /****************************************************************/
    bool look(Property &request)
    {
        if (closing)
            return false;

        if (verbosity>0)
            yInfo("Received request: %s",request.toString().c_str());

        if (request.check("stop"))
        {
            lock_guard<mutex> lg(mtx);
            stopControl();
            return true;
        }

        if (request.check("control-frame"))
        {
            string control_frame_=request.find("control-frame").asString();
            const set<string>::iterator it=avFrames.find(control_frame_);
            if (it==avFrames.end())
                yError("Unrecognized control frame type \"%s\"!",control_frame_.c_str());                
            else
                control_frame=control_frame_;
        }

        bool doControl=false;        
        Vector xd(3);
        Vector q;

        string target_type=request.check("target-type",Value("cartesian")).asString();
        Bottle *target_location=request.find("target-location").asList();
        if ((target_type!="cartesian") && (target_type!="angular") && (target_type!="image"))
            yError("Unrecognized target type \"%s\"!",target_type.c_str());
        else if (target_location==NULL)
            yError("Missing \"target-location\" option!");
        else if (target_type=="cartesian")
        {
            if (target_location->size()>=3)
            {
                xd[0]=target_location->get(0).asFloat64();
                xd[1]=target_location->get(1).asFloat64();
                xd[2]=target_location->get(2).asFloat64();

                q=getEncoders();
                doControl=true;
            }
            else
                yError("Missing Cartesian coordinates!");
        }
        else if (target_type=="angular")
        {
            if (target_location->size()>=2)
            {
                Vector azi(4,0.0); azi[2]=1.0;
                azi[3]=CTRL_DEG2RAD*target_location->get(0).asFloat64();

                Vector ele(4,0.0); ele[1]=-1.0;
                ele[3]=CTRL_DEG2RAD*target_location->get(1).asFloat64();

                Matrix Hee;
                Vector q0(6,0.0);
                mtx.lock();
                solver[control_frame].fkin(q0,Hee);
                mtx.unlock();

                Vector xc(4,0.0);
                xc[2]=xc[3]=1.0;

                xd=axis2dcm(azi)*axis2dcm(ele)*Hee*xc;
                xd.pop_back();

                q=getEncoders();
                doControl=true;
            }
            else
                yError("Missing angular coordinates!");
        }
        else if (target_type=="image")
        {
            string image=request.check("image",Value("left")).asString();
            if (avFrames.find(image)==avFrames.end())
                yError("Unrecognized image type \"%s\"!",image.c_str());
            else if (intrinsics.find(image)==intrinsics.end())
                yError("Intrinsics not configured for image type \"%s\"!",image.c_str());
            else 
            {                    
                if (target_location->size()>=2)
                {
                    Vector p(3,1.0);
                    if (target_location->size()>=3)
                        p[2]=target_location->get(2).asFloat64();

                    p[0]=p[2]*target_location->get(0).asFloat64(); 
                    p[1]=p[2]*target_location->get(1).asFloat64();

                    Matrix Hee;
                    q=getEncoders();
                    mtx.lock();
                    solver[image].fkin(q,Hee);
                    mtx.unlock();

                    Vector xc=intrinsics[image]*p;
                    xc[3]=1.0;

                    xd=Hee*xc;
                    xd.pop_back();

                    doControl=true;
                }
                else
                    yError("Missing image coordinates!");
            }
        }
        else
            yError("Reached a point in the code we shouldn't have reached :(");

        if (doControl)
        {
            lock_guard<mutex> lg(mtx);
            solver[control_frame].setInitialGuess(q);
            solver[control_frame].ikin(xd,qd);

            if (!controlling)
                gen->init(q.subVector(4,5));

            controlling=true;
            if (verbosity>0)
                yInfo("Going to: %s",qd.toString(3,3).c_str());
        }

        return doControl;
    }

    /****************************************************************/
    bool updateModule()
    {
        lock_guard<mutex> lg(mtx);
        getCurrentMode();        

        double timeStamp;
        Vector q=getEncoders(&timeStamp);
     
        if (timeStamp>=0.0)
            txInfo.update(timeStamp);
        else
            txInfo.update();

        fillState(q,statePort.prepare());
        statePort.setEnvelope(txInfo);
        statePort.writeStrict();

        if (controlling)
        {
            gen->computeNextValues(qd);
            Vector ref=gen->getPos();

            if (verbosity>1)
                yInfo("Commanding new set-points: %s",ref.toString(3,3).c_str());

            if (areJointsHealthy())
            {
                setPositionDirectMode(); 
                iposd->setPositions(ref.data());

                Vector q_=q.subVector(4,5);
                if (norm(qd-q_)<stop_threshold)
                {
                    controlling=false;
                    if (verbosity>0)
                        yInfo("Just stopped at: %s",q_.toString(3,3).c_str());
                }
            }
            else
            {
                yWarning("Detected joints in HW_FAULT and/or IDLE => stopping control");
                stopControl();                
            }
        }

        return true;
    }

    /****************************************************************/
    bool respond(const Bottle &cmd, Bottle &reply)
    {
        int cmd_0=cmd.get(0).asVocab32();
        if (cmd.size()==3)
        {
            if (cmd_0==Vocab32::encode("set"))
            {
                string cmd_1=cmd.get(1).asString();
                if (cmd_1=="T")
                {
                    lock_guard<mutex> lg(mtx);
                    gen->setT(cmd.get(2).asFloat64());
                    reply.addVocab32(Vocab32::encode("ack"));
                }
                else if (cmd_1=="Ts")
                {
                    lock_guard<mutex> lg(mtx);
                    Ts=cmd.get(2).asFloat64();
                    Ts=std::max(Ts,MIN_TS);
                    gen->setTs(Ts);
                    reply.addVocab32(Vocab32::encode("ack"));
                }
                else if (cmd_1=="verbosity")
                {
                    lock_guard<mutex> lg(mtx);
                    verbosity=cmd.get(2).asInt32();
                    for (auto &s:solver)
                        s.second.setVerbosity(verbosity);
                    reply.addVocab32(Vocab32::encode("ack"));
                }
                else if (cmd_1=="joints-limits::pitch")
                {
                    if (Bottle *pitchLim=cmd.get(2).asList())
                    {
                        lock_guard<mutex> lg(mtx);
                        applyCustomJointsBounds(pitchLim,NULL);
                        reply.addVocab32(Vocab32::encode("ack"));
                    }
                }
                else if (cmd_1=="joints-limits::yaw")
                {
                    if (Bottle *yawLim=cmd.get(2).asList())
                    {
                        lock_guard<mutex> lg(mtx);
                        applyCustomJointsBounds(NULL,yawLim);
                        reply.addVocab32(Vocab32::encode("ack"));
                    }
                }
            }
        }
        else if (cmd.size()==2)
        {
            if (cmd_0==Vocab32::encode("get"))
            {
                string cmd_1=cmd.get(1).asString();
                if (cmd_1=="done")
                {
                    lock_guard<mutex> lg(mtx);
                    reply.addVocab32(Vocab32::encode("ack"));
                    reply.addInt32(controlling?0:1);
                }
                else if (cmd_1=="T")
                {
                    lock_guard<mutex> lg(mtx);
                    reply.addVocab32(Vocab32::encode("ack"));
                    reply.addFloat64(gen->getT());
                }
                else if (cmd_1=="Ts")
                {
                    lock_guard<mutex> lg(mtx);
                    reply.addVocab32(Vocab32::encode("ack"));
                    reply.addFloat64(Ts);
                }
                else if (cmd_1=="verbosity")
                {
                    lock_guard<mutex> lg(mtx);
                    reply.addVocab32(Vocab32::encode("ack"));
                    reply.addInt32(verbosity);
                }
                else if (cmd_1=="joints-limits::pitch")
                {
                    lock_guard<mutex> lg(mtx);
                    Vector pitchLim,yawLim;
                    getJointsBounds(pitchLim,yawLim);
                    reply.addVocab32(Vocab32::encode("ack"));                    
                    reply.addList().read(pitchLim);
                }
                else if (cmd_1=="joints-limits::yaw")
                {
                    lock_guard<mutex> lg(mtx);
                    Vector pitchLim,yawLim;
                    getJointsBounds(pitchLim,yawLim);
                    reply.addVocab32(Vocab32::encode("ack"));                    
                    reply.addList().read(yawLim);
                }
            }
            else if (cmd_0==Vocab32::encode("look"))
            {
                if (Bottle *b=cmd.get(1).asList())
                {
                    Property p(b->toString().c_str());
                    if (look(p))
                        reply.addVocab32(Vocab32::encode("ack"));
                }
            }
        }
        else if (cmd_0==Vocab32::encode("stop"))
        {
            lock_guard<mutex> lg(mtx);
            stopControl();
            reply.addVocab32(Vocab32::encode("ack"));
        }

        if (reply.size()==0)
            reply.addVocab32(Vocab32::encode("nack")); 
        
        return true;
    }
};


/****************************************************************/
void TargetPort::onRead(Property &request)
{
    if (ctrl!=NULL)
        ctrl->look(request);
}


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }
    
    ResourceFinder rf;
    rf.configure(argc,argv);

    Controller controller;
    return controller.runModule(rf);
}

