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
    PolyDriver drivers[4];
    IControlMode2   *imod[4];
    IEncodersTimed  *ienc[4];
    IPositionDirect *ipos[4];

    minJerkTrajGen *gen;

    BufferedPort<Bottle> targetPort;
    RpcServer rpcPort;
    RpcClient solverPort;

    Mutex mutex;
    double Ts;
    Vector qd;

    /****************************************************************/
    bool read(ConnectionReader &connection)
    {
        Bottle target;
        target.read(connection);

        Bottle reply;
        solverPort.write(target,reply);

        if (reply.get(0).asVocab()==Vocab::encode("ack"))
        {
            if (Bottle *payLoad=reply.get(1).asList())
            {
                LockGuard lg(mutex);
                for (size_t i=0; i<qd.length(); i++)
                    qd[i]=payLoad->get(i).asDouble();
            }
        }
        else
            yError("Malformed target type!");

        return true;
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
        double T=rf.check("T",Value(2.0)).asDouble();
        Ts=rf.check("Ts",Value(MIN_TS)).asDouble();
        Ts=std::max(Ts,MIN_TS);

        Property option;

        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/torso_tripod").c_str());
        option.put("local","/cer_controller/torso_tripod");
        if (!drivers[0].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/torso_tripod").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/torso_yaw").c_str());
        option.put("local","/cer_controller/torso_yaw");
        if (!drivers[1].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/torso_yaw").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/upper_"+arm_type+"_arm").c_str());
        option.put("local",("/cer_controller/upper_"+arm_type+"_arm").c_str());
        if (!drivers[2].open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/upper_"+arm_type+"_arm").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote",("/"+robot+"/"+arm_type+"_wrist_tripod").c_str());
        option.put("local",("/cer_controller/"+arm_type+"_wrist_tripod").c_str());
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
        }

        targetPort.open(("/cer_controller/"+arm_type+"/target:i").c_str());
        targetPort.setReader(*this);

        solverPort.open(("/cer_controller/"+arm_type+"/solver:rpc").c_str());

        rpcPort.open(("/cer_controller/"+arm_type+"/rpc").c_str());
        attach(rpcPort);

        qd.resize(12,0.0);
        VectorOf<int> posDirectMode;
        for (size_t i=0; i<qd.length(); i++)
            posDirectMode.push_back(VOCAB_CM_POSITION_DIRECT);

        imod[0]->setControlModes(posDirectMode.getFirst()); 
        imod[1]->setControlModes(posDirectMode.getFirst());
        imod[2]->setControlModes(posDirectMode.getFirst());
        imod[3]->setControlModes(posDirectMode.getFirst());
        
        ienc[0]->getEncoders(&qd[0]);
        ienc[1]->getEncoders(&qd[3]);
        ienc[2]->getEncoders(&qd[4]);
        ienc[3]->getEncoders(&qd[10]);

        gen=new minJerkTrajGen(qd,Ts,T);
        return true;
    }

    /****************************************************************/
    bool close()
    {
        if (!targetPort.isClosed())
            targetPort.close(); 

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
        gen->computeNextValues(qd);

        const Vector &ref=gen->getPos();
        ipos[0]->setPositions(&ref[0]);
        ipos[1]->setPositions(&ref[3]);
        ipos[2]->setPositions(&ref[4]);
        ipos[3]->setPositions(&ref[10]);

        return true;
    }

    /****************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        LockGuard lg(mutex);
        
        if (command.size()>=3)
        {
            if (command.get(0).asVocab()==Vocab::encode("set"))
            {
                int subcmd=command.get(1).asVocab();
                if (subcmd==Vocab::encode("T"))
                {
                    gen->setT(command.get(2).asDouble());
                    reply.addVocab(Vocab::encode("ack"));
                }
                else if (subcmd==Vocab::encode("Ts"))
                {
                    Ts=command.get(2).asDouble();
                    Ts=std::max(Ts,MIN_TS);
                    gen->setTs(Ts);
                    reply.addVocab(Vocab::encode("ack"));
                }
            }
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

