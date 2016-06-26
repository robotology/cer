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

#include <string>
#include <set>
#include <map>
#include <cmath>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

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


/****************************************************************/
class Controller : public RFModule, public PortReader
{
    PolyDriver         drivers[3];
    IEncodersTimed*    ienc[3];
    IControlMode2*     imod;
    IPositionControl2* ipos;
    IPositionDirect*   iposd;

    VectorOf<int> posDirectMode;
    VectorOf<int> curMode;

    map<string,HeadSolver> solver;
    map<string,Matrix> intrinsincs;
    minJerkTrajGen* gen;
    
    BufferedPort<Property> statePort;
    Port targetPort;
    RpcServer rpcPort;
    Stamp txInfo;

    Mutex mutex;
    int verbosity;
    bool closing;
    bool controlling;
    set<string> controlFrames;
    string control_frame;
    double stop_threshold;
    double Ts;
    Vector qd;    

    /****************************************************************/
    bool read(ConnectionReader &connection)
    {
        Property request;
        request.read(connection);

        if (closing)
            return true;

        if (verbosity>0)
            yInfo("Received target request: %s",request.toString().c_str());

        if (request.check("control"))
        {
            string control_frame_=request.find("control").asString();
            const set<string>::iterator it=controlFrames.find(control_frame_);
            if (it==controlFrames.end())
                yError("Unrecognized control frame type \"%s\"!",control_frame_.c_str());                
            else
                control_frame=control_frame_;
        }

        bool doControl=false;        
        Vector xd(3);
        Vector q;

        string type=request.check("type",Value("cartesian")).asString();
        Bottle *target=request.find("target").asList();
        if ((type!="cartesian") || (type!="image"))
            yError("Unrecognized target type \"%s\"!",type.c_str());
        else if (target==NULL)
            yError("Missing \"target\" option!");
        else if (type=="cartesian")
        {
            if (target->size()>=3)
            {
                xd[0]=target->get(0).asDouble();
                xd[1]=target->get(1).asDouble();
                xd[2]=target->get(2).asDouble();

                q=getEncoders();
                doControl=true;
            }
            else
                yError("Provided too few Cartesian coordinates!");
        }
        else if (type=="image")
        {
            string image=request.check("image",Value("left")).asString();
            const set<string>::iterator it=controlFrames.find(image);
            if ((it==controlFrames.end()) || (image=="center"))
                yError("Unrecognized image type \"%s\"!",image.c_str());
            else if (intrinsincs.find(image)==intrinsincs.end())
                yError("Intrinsics not configured for image type \"%s\"!",image.c_str());
            else 
            {                    
                if (target->size()>=3)
                {
                    Vector p(3);
                    p[0]=target->get(0).asDouble();
                    p[1]=target->get(1).asDouble();
                    p[2]=target->get(2).asDouble();
                    p[0]*=p[2]; p[1]*=p[2];

                    Matrix Hee;
                    q=getEncoders();
                    solver[image].fkin(q,Hee);

                    Vector xc=intrinsincs[image]*p;
                    xc[3]=1.0;

                    xd=Hee*xc;
                    xd.pop_back();

                    doControl=true;
                }
                else
                    yError("Provided too few image coordinates!");
            }
        }

        if (doControl)
        {
            LockGuard lg(mutex);
            solver[control_frame].setInitialGuess(q);
            solver[control_frame].ikin(xd,qd);

            if (!controlling)
                gen->init(q.subVector(4,5));
            
            controlling=true;
            if (verbosity>0)
                yInfo("Going to: %s",qd.toString(3,3).c_str());
        }

        return true;
    }

    /****************************************************************/
    void getIntrinsics(ResourceFinder &rf)
    {
        for (set<string>::iterator it=controlFrames.begin();
             it!=controlFrames.end(); it++)
        {
            const string &frame=*it;
            if (frame!="center")
            {
                yInfo("#### Retrieving intrinsics for \"%s\" control frame",frame.c_str());

                string groupName=frame;
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
                        K(0,0)=group.find("fx").asDouble();
                        K(1,1)=group.find("fy").asDouble();
                        K(0,2)=group.find("cx").asDouble();
                        K(1,2)=group.find("cy").asDouble();
                        
                        yInfo("%s",K.toString(3,3).c_str());
                        intrinsincs[frame]=pinv(K.transposed()).transposed(); 
                        ok=true;
                    }
                }

                if (!ok)
                    yWarning("Intrinsics for \"%s\" control frame not configured!",
                             frame.c_str());
            }
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
        encs.setSubvector(4,encs_.subVector(0,4));
        stamps.setSubvector(4,stamps_.subVector(0,4));

        if (timeStamp!=NULL)
            *timeStamp=findMax(stamps);

        return encs;
    }

    /****************************************************************/
    void getCurrentMode()
    {
        imod->getControlModes(&curMode[0]);
    }

    /****************************************************************/
    void setPositionDirectMode()
    {
        for (size_t i=0; i<curMode.size(); i++)
        {
            if (curMode[i]!=posDirectMode[i])
            {
                imod->setControlModes(&posDirectMode[0]);
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
            HeadSolver &s=it->second;
            yInfo("##### control frame \"%s\"",it->first);

            HeadParameters p=s.getHeadParameters();
            iKinChain &chain=*p.head.asChain();
            Matrix lim;
            Value part;

            getBounds(drivers[1],lim);
            p.torso.l_min=lim(0,0);
            p.torso.l_max=lim(0,1);
            p.torso.alpha_max=fabs(lim(1,1));

            chain[0].setMin((M_PI/180.0)*lim(3,0));
            chain[0].setMax((M_PI/180.0)*lim(3,1));

            part=drivers[1].getValue("remote");
            yInfo("limits of %s part: heave=[%g,%g] [m], [pitch,roll]=[%g,%g] [deg], yaw=[%g,%g] [deg]",
                  part.asString().c_str(),p.torso.l_min,p.torso.l_max,-p.torso.alpha_max,p.torso.alpha_max,lim(3,0),lim(3,1));

            getBounds(drivers[2],lim);
            for (int i=0; i<2; i++)
            {
                chain[1+i].setMin((M_PI/180.0)*lim(i,0));
                chain[1+i].setMax((M_PI/180.0)*lim(i,1));

                part=drivers[2].getValue("remote");
                yInfo("limits of %s part: joint %d=[%g,%g] [deg]",
                      part.asString().c_str(),i,(180.0/M_PI)*chain[1+i].getMin(),
                      (180.0/M_PI)*chain[1+i].getMax());
            }

            s.setHeadParameters(p);
        }
    }

    /****************************************************************/
    void addState(const Vector &q, Property &state)
    {
        for (set<string>::iterator it=controlFrames.begin();
             it!=controlFrames.end(); it++)
        {
            const string &frame=*it;

            Matrix Hee;
            solver[frame].fkin(q,Hee);

            Vector pose=Hee.getCol(3).subVector(0,2);
            Vector oee; oee=dcm2axis(Hee);
            oee*=oee[3]; oee.pop_back();
            pose=cat(pose,oee);

            Bottle val; val.addList().read(pose);
            state.put(frame,val.get(0));
        }
    }

public:
    /****************************************************************/
    Controller() : gen(NULL)
    {
        controlFrames=HeadParameters::getTypes();
        for (set<string>::iterator it=controlFrames.begin();
             it!=controlFrames.end(); it++)
        {
            const string &frame=*it;
            HeadParameters p(frame);
            solver[frame].setHeadParameters(p);
        }
        control_frame="center"; 
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("cer")).asString();
        bool get_bounds=(rf.check("get-bounds",Value("on")).asString()=="on");
        verbosity=rf.check("verbosity",Value(0)).asInt();
        stop_threshold=rf.check("stop-threshold",Value(0.01)).asDouble();
        double T=rf.check("T",Value(1.0)).asDouble();
        Ts=rf.check("Ts",Value(MIN_TS)).asDouble();
        Ts=std::max(Ts,MIN_TS);

        Property option;

        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/torso_tripod").c_str());
        option.put("local","/cer_gaze-controller/torso_tripod"); 
        if (!drivers[0].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/torso_tripod").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/torso").c_str());
        option.put("local","/cer_gaze-controller/torso");
        if (!drivers[1].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/torso").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/head").c_str());
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
        targetPort.setReader(*this);

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
        setPositionDirectMode();

        if (get_bounds)
            alignJointsBounds();

        ResourceFinder rf_cameras;
        rf_cameras.setDefaultContext("cameraCalibration");
        rf_cameras.setDefaultConfigFile("cer.ini");
        rf_cameras.configure(0,NULL);
        getIntrinsics(rf_cameras);

        gen=new minJerkTrajGen(qd,Ts,T);
        closing=controlling=false;

        return true;
    }

    /****************************************************************/
    bool close()
    {
        closing=true;

        if (controlling)
            stopControl();

        if (targetPort.isOpen())
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
    bool updateModule()
    {
        LockGuard lg(mutex);
        getCurrentMode();
        setPositionDirectMode();

        double timeStamp;
        Vector q=getEncoders(&timeStamp);

        Property &state=statePort.prepare();
        state.clear();
        
        addState(q,state);
     
        if (timeStamp>=0.0)
            txInfo.update(timeStamp);
        else
            txInfo.update();

        statePort.setEnvelope(txInfo);
        statePort.write();

        if (controlling)
        {
            gen->computeNextValues(qd);
            Vector ref=gen->getPos();

            if (verbosity>1)
                yInfo("Commanding new set-points: %s",ref.toString(3,3).c_str());

            iposd->setPositions(ref.data());

            if (norm(qd-ref)<stop_threshold)
            {
                controlling=false;
                if (verbosity>0)
                    yInfo("Just stopped at: %s",ref.toString(3,3).c_str());
            }
        }

        return true;
    }

    /****************************************************************/
    bool respond(const Bottle &cmd, Bottle &reply)
    {
        LockGuard lg(mutex);
        int cmd_0=cmd.get(0).asVocab();

        if (cmd.size()==3)
        {
            if (cmd_0==Vocab::encode("set"))
            {
                int cmd_1=cmd.get(1).asVocab();
                if (cmd_1==Vocab::encode("T"))
                {
                    gen->setT(cmd.get(2).asDouble());
                    reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1==Vocab::encode("Ts"))
                {
                    Ts=cmd.get(2).asDouble();
                    Ts=std::max(Ts,MIN_TS);
                    gen->setTs(Ts);
                    reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1==Vocab::encode("verbosity"))
                {
                    verbosity=cmd.get(2).asInt();
                    reply.addVocab(Vocab::encode("ack"));
                }
            }
        }
        else if (cmd.size()==2)
        {
            if (cmd_0==Vocab::encode("get"))
            {
                int cmd_1=cmd.get(1).asVocab();
                if (cmd_1==Vocab::encode("T"))
                {
                    reply.addVocab(Vocab::encode("ack"));
                    reply.addDouble(gen->getT());
                }
                else if (cmd_1==Vocab::encode("Ts"))
                {
                    reply.addVocab(Vocab::encode("ack"));
                    reply.addDouble(Ts);
                }
                else if (cmd_1==Vocab::encode("verbosity"))
                {
                    reply.addVocab(Vocab::encode("ack"));
                    reply.addInt(verbosity);
                }
            }
        }
        else if (cmd_0==Vocab::encode("stop"))
        {
            stopControl();
            reply.addVocab(Vocab::encode("ack"));
        }

        if (reply.size()==0)
            reply.addVocab(Vocab::encode("nack")); 
        
        return true;
    }
};


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

