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

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/iKin/iKinFwd.h>
#include <cer_kinematics/arm.h>

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
    PolyDriver drivers[4];
    IControlMode2    *imod[4];
    IEncodersTimed   *ienc[4];
    IPositionControl *ipos[4];
    IPositionDirect  *iposd[4];

    VectorOf<int> posDirectMode;
    VectorOf<int> curMode;

    ArmSolver solver;
    minJerkTrajGen *gen;

    BufferedPort<Bottle> targetPort;
    BufferedPort<Vector> statePort;
    RpcServer rpcPort;
    RpcClient solverPort;
    Stamp txInfo;

    Mutex mutex;
    int verbosity;
    bool controlling;
    double Ts;
    Vector qd;    

    /****************************************************************/
    bool read(ConnectionReader &connection)
    {
        Bottle target,reply;
        target.read(connection);

        if (verbosity>0)
            yInfo("Sending request to solver: %s",target.toString().c_str());

        bool latch_controlling=controlling;
        if (solverPort.write(target,reply))
        {
            if (verbosity>0)
                yInfo("Received reply from solver: %s",reply.toString().c_str());

            if (reply.get(0).asVocab()==Vocab::encode("ack"))
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

        ienc[0]->getEncodersTimed(&encs[0],&stamps[0]);
        ienc[1]->getEncodersTimed(&encs[3],&stamps[3]);
        ienc[2]->getEncodersTimed(&encs[4],&stamps[4]);
        ienc[3]->getEncodersTimed(&encs[9],&stamps[9]);

        if (timeStamp!=NULL)
            *timeStamp=findMax(stamps);

        return encs;
    }

    /****************************************************************/
    void getCurrentMode()
    {
        imod[0]->getControlModes(&curMode[0]);
        imod[1]->getControlModes(&curMode[3]);
        imod[2]->getControlModes(&curMode[4]);
        imod[3]->getControlModes(&curMode[9]);
    }

    /****************************************************************/
    void setPositionDirectMode()
    {
        for (size_t i=0; i<3; i++)
        {
            if (curMode[i]!=posDirectMode[0])
            {
                imod[0]->setControlModes(posDirectMode.getFirst()); 
                break;
            }
        }

        if (curMode[3]!=posDirectMode[0])
            imod[1]->setControlModes(posDirectMode.getFirst()); 

        for (size_t i=4; i<9; i++)
        {
            if (curMode[i]!=posDirectMode[0])
            {
                imod[2]->setControlModes(posDirectMode.getFirst()); 
                break;
            }
        }

        for (size_t i=9; i<curMode.size(); i++)
        {
            if (curMode[i]!=posDirectMode[0])
            {
                imod[3]->setControlModes(posDirectMode.getFirst()); 
                break;
            }
        }
    }

public:
    /****************************************************************/
    Controller() : gen(NULL)
    {
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("cer")).asString().c_str();
        string arm_type=rf.check("arm-type",Value("left")).asString().c_str();
        verbosity=rf.check("verbosity",Value(0)).asInt();
        double T=rf.check("T",Value(2.0)).asDouble();
        Ts=rf.check("Ts",Value(MIN_TS)).asDouble();
        Ts=std::max(Ts,MIN_TS);

        Property option;

        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/torso_tripod").c_str());
        option.put("local",("/cer_controller/"+arm_type+"/torso_tripod").c_str()); 
        if (!drivers[0].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/torso_tripod").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/torso_yaw").c_str());
        option.put("local",("/cer_controller/"+arm_type+"/torso_yaw").c_str());
        if (!drivers[1].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/torso_yaw").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/"+arm_type+"_upper_arm").c_str());
        option.put("local",("/cer_controller/"+arm_type+"/"+arm_type+"_upper_arm").c_str());
        if (!drivers[2].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/"+arm_type+"_upper_arm").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/"+arm_type+"_wrist_tripod").c_str());
        option.put("local",("/cer_controller/"+arm_type+"/"+arm_type+"_wrist_tripod").c_str());
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

        targetPort.open(("/cer_controller/"+arm_type+"/target:i").c_str());
        targetPort.setReader(*this);

        statePort.open(("/cer_controller/"+arm_type+"/state:o").c_str());
        solverPort.open(("/cer_controller/"+arm_type+"/solver:rpc").c_str());

        rpcPort.open(("/cer_controller/"+arm_type+"/rpc").c_str());
        attach(rpcPort);
        
        Vector qd=getEncoders();
        for (size_t i=0; i<qd.length(); i++)
            posDirectMode.push_back(VOCAB_CM_POSITION_DIRECT);
        curMode=posDirectMode;

        getCurrentMode();
        setPositionDirectMode();
        
        ArmParameters arm(arm_type);
        arm.upper_arm.setAllConstraints(false);
        solver.setArmParameters(arm);

        gen=new minJerkTrajGen(qd,Ts,T);
        controlling=false;

        return true;
    }

    /****************************************************************/
    bool close()
    {
        if (!targetPort.isClosed())
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
        Vector oee=dcm2axis(Hee);
        oee*=oee[3]; oee.pop_back();
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
            const Vector &ref=gen->getPos();

            iposd[0]->setPositions(&ref[0]);
            iposd[1]->setPositions(&ref[3]);
            iposd[2]->setPositions(&ref[4]);
            iposd[3]->setPositions(&ref[9]);

            if (verbosity>1)
                yInfo("Commanding new set-points: %s",qd.toString(3,3).c_str());
        }

        return true;
    }

    /****************************************************************/
    bool respond(const Bottle &cmd, Bottle &reply)
    {
        LockGuard lg(mutex);
        int cmd_0=cmd.get(0).asVocab();

        if (cmd.size()>=3)
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
            }
        }
        else if (cmd_0==Vocab::encode("stop"))
        {
            controlling=false;
            for (int i=0; i<4; i++)
                ipos[i]->stop();
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

