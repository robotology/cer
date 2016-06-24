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

#include <algorithm>
#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/minJerkCtrl.h>
#include <cer_kinematics/arm.h>

#define MIN_TS  0.01    // [s]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace cer::kinematics;


/****************************************************************/
class Controller : public RFModule, public PortReader
{
    PolyDriver         drivers[4];
    VectorOf<int>      jointsIndexes[4];
    IControlMode2*     imod[4];
    IEncodersTimed*    ienc[4];
    IPositionControl2* ipos[4];
    IPositionDirect*   iposd[4];
    
    VectorOf<int> posDirectMode;
    VectorOf<int> curMode;

    ArmSolver solver;
    minJerkTrajGen* gen;
    
    BufferedPort<Vector> statePort;
    Port targetPort;
    RpcServer rpcPort;
    RpcClient solverPort;
    Stamp txInfo;

    Mutex mutex;
    string orientation_type;
    int verbosity;
    bool closing;
    bool controlling;
    double stop_threshold;
    double Ts;
    Vector qd;    

    /****************************************************************/
    bool read(ConnectionReader &connection)
    {
        Property target;
        target.read(connection);

        if (closing)
            return true;

        if (verbosity>0)
            yInfo("Received target request: %s",target.toString().c_str());

        if (!target.check("q"))
        {            
            Vector q=getEncoders();
            Bottle b; b.addList().read(q);
            target.put("q",b.get(0));
        }

        if (verbosity>0)
            yInfo("Forwarding request to solver: %s",target.toString().c_str());

        Bottle reply;
        bool latch_controlling=controlling;
        if (solverPort.write(target,reply))
        {
            if (verbosity>0)
                yInfo("Received reply from solver: %s",reply.toString().c_str());

            if (reply.get(0).asVocab()==Vocab::encode("ack"))
            {
                if (reply.size()>1)
                {
                    if (Bottle *payLoad=reply.get(1).asList())
                    {
                        LockGuard lg(mutex);
                        // process only if we didn't receive
                        // a stop request in the meanwhile
                        if (controlling==latch_controlling)
                        {
                            for (size_t i=0; i<qd.length(); i++)
                                qd[i]=payLoad->get(i).asDouble();

                            if (!controlling)
                                gen->init(getEncoders());

                            setPositionDirectMode();
                            controlling=true;
                            if (verbosity>0)
                                yInfo("Going to: %s",qd.toString(3,3).c_str());
                        }
                    }
                }
            }
            else
                yError("Malformed target type!");
        }
        else
            yError("Unable to communicate with the solver");

        return true;
    }

    /****************************************************************/
    Vector getEncoders(double *timeStamp=NULL)
    {
        Vector encs(12,0.0);
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

        ienc[3]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(9,encs_.subVector(0,2));
        stamps.setSubvector(9,stamps_.subVector(0,2));

        if (timeStamp!=NULL)
            *timeStamp=findMax(stamps);

        return encs;
    }

    /****************************************************************/
    void getCurrentMode()
    {
        imod[0]->getControlModes(jointsIndexes[0].size(),jointsIndexes[0].getFirst(),&curMode[0]);
        imod[1]->getControlModes(jointsIndexes[1].size(),jointsIndexes[1].getFirst(),&curMode[3]);
        imod[2]->getControlModes(jointsIndexes[2].size(),jointsIndexes[2].getFirst(),&curMode[4]);
        imod[3]->getControlModes(jointsIndexes[3].size(),jointsIndexes[3].getFirst(),&curMode[9]);
    }

    /****************************************************************/
    void setPositionDirectMode()
    {
        for (size_t i=0; i<3; i++)
        {
            if (curMode[i]!=posDirectMode[i])
            {
                imod[0]->setControlModes(jointsIndexes[0].size(),jointsIndexes[0].getFirst(),&posDirectMode[0]);
                break;
            }
        }

        for (size_t i=3; i<4; i++)
        {
            if (curMode[i]!=posDirectMode[i])
            {
                imod[1]->setControlModes(jointsIndexes[1].size(),jointsIndexes[1].getFirst(),&posDirectMode[3]);
                break;
            }
        }

        for (size_t i=4; i<9; i++)
        {
            if (curMode[i]!=posDirectMode[i])
            {
                imod[2]->setControlModes(jointsIndexes[2].size(),jointsIndexes[2].getFirst(),&posDirectMode[4]);
                break;
            }
        }

        for (size_t i=9; i<curMode.size(); i++)
        {
            if (curMode[i]!=posDirectMode[i])
            {
                imod[3]->setControlModes(jointsIndexes[3].size(),jointsIndexes[3].getFirst(),&posDirectMode[9]);
                break;
            }
        }
    }

    /****************************************************************/
    void stopControl()
    {        
        for (int i=0; i<4; i++)
            ipos[i]->stop(jointsIndexes[i].size(),jointsIndexes[i].getFirst());
        controlling=false;
    }

public:
    /****************************************************************/
    Controller() : gen(NULL)
    {
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("cer")).asString();
        string arm_type=rf.check("arm-type",Value("left")).asString();
        orientation_type=rf.check("orientation-type",Value("axis-angle")).asString();
        verbosity=rf.check("verbosity",Value(0)).asInt();
        stop_threshold=rf.check("stop-threshold",Value(0.1)).asDouble();
        double T=rf.check("T",Value(2.0)).asDouble();
        Ts=rf.check("Ts",Value(MIN_TS)).asDouble();
        Ts=std::max(Ts,MIN_TS);

        Property option;

        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/torso_tripod").c_str());
        option.put("local",("/cer_reaching-controller/"+arm_type+"/torso_tripod").c_str()); 
        if (!drivers[0].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/torso_tripod").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/torso").c_str());
        option.put("local",("/cer_reaching-controller/"+arm_type+"/torso").c_str());
        if (!drivers[1].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/torso").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/"+arm_type+"_arm").c_str());
        option.put("local",("/cer_reaching-controller/"+arm_type+"/"+arm_type+"_arm").c_str());
        if (!drivers[2].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/"+arm_type+"_arm").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/"+arm_type+"_wrist_tripod").c_str());
        option.put("local",("/cer_reaching-controller/"+arm_type+"/"+arm_type+"_wrist_tripod").c_str());
        if (!drivers[3].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/"+arm_type+"_wrist_tripod").c_str());
            close();
            return false;
        }

        for (int i=0; i<4; i++)
        {
            drivers[i].view(imod[i]);
            drivers[i].view(ienc[i]);
            drivers[i].view(ipos[i]);
            drivers[i].view(iposd[i]);
        }

        // torso_tripod
        jointsIndexes[0].push_back(0);
        jointsIndexes[0].push_back(1);
        jointsIndexes[0].push_back(2);

        // torso (yaw)
        jointsIndexes[1].push_back(3);

        // arm
        jointsIndexes[2].push_back(0);
        jointsIndexes[2].push_back(1);
        jointsIndexes[2].push_back(2);
        jointsIndexes[2].push_back(3);
        jointsIndexes[2].push_back(4);

        // wrist_tripod
        jointsIndexes[3].push_back(0);
        jointsIndexes[3].push_back(1);
        jointsIndexes[3].push_back(2);

        statePort.open(("/cer_reaching-controller/"+arm_type+"/state:o").c_str());
        solverPort.open(("/cer_reaching-controller/"+arm_type+"/solver:rpc").c_str());

        targetPort.open(("/cer_reaching-controller/"+arm_type+"/target:i").c_str());
        targetPort.setReader(*this);

        rpcPort.open(("/cer_reaching-controller/"+arm_type+"/rpc").c_str());
        attach(rpcPort);

        transform(orientation_type.begin(),orientation_type.end(),
                  orientation_type.begin(),::tolower);
        if ((orientation_type!="axis-angle") && (orientation_type!="rpy"))
        {
            yWarning("Unrecognized Orientation Type \"%s\"",orientation_type.c_str());
            orientation_type="axis-angle";
        }
        yInfo("Orientation Type is \"%s\"",orientation_type.c_str()); 
        
        qd=getEncoders();
        for (size_t i=0; i<qd.length(); i++)
            posDirectMode.push_back(VOCAB_CM_POSITION_DIRECT);
        curMode=posDirectMode;

        getCurrentMode();
        setPositionDirectMode();
        
        ArmParameters arm(arm_type);
        arm.upper_arm.setAllConstraints(false);
        solver.setArmParameters(arm);

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

        if (!solverPort.asPort().isOpen())
            solverPort.close();

        if (!rpcPort.asPort().isOpen())
            rpcPort.close();
                
        for (int i=0; i<4; i++)
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

        Matrix Hee;
        double timeStamp;
        solver.fkin(getEncoders(&timeStamp),Hee);

        Vector &pose=statePort.prepare();
        pose=Hee.getCol(3).subVector(0,2);

        Vector oee;
        if (orientation_type=="rpy")
            oee=dcm2rpy(Hee);
        else
        {
            oee=dcm2axis(Hee);
            oee*=oee[3]; oee.pop_back();
        }

        pose=cat(pose,oee);

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

            iposd[0]->setPositions(jointsIndexes[0].size(),jointsIndexes[0].getFirst(),&ref[0]);
            iposd[1]->setPositions(jointsIndexes[1].size(),jointsIndexes[1].getFirst(),&ref[3]);
            iposd[2]->setPositions(jointsIndexes[2].size(),jointsIndexes[2].getFirst(),&ref[4]);
            iposd[3]->setPositions(jointsIndexes[3].size(),jointsIndexes[3].getFirst(),&ref[9]);

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

