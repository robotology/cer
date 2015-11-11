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
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("cer")).asString().c_str();
        string arm_type=rf.check("arm-type",Value("left")).asString().c_str();
        double T=rf.check("T",Value(2.0)).asDouble();
        Ts=rf.check("Ts",Value(0.01)).asDouble();
        Ts=std::max(Ts,0.01);

        targetPort.open(("/cer_controller/"+arm_type+"/target:i").c_str());
        targetPort.setReader(*this);

        solverPort.open(("/cer_controller/"+arm_type+"/solver:rpc").c_str());

        rpcPort.open(("/cer_controller/"+arm_type+"/rpc").c_str());
        attach(rpcPort);

        qd.resize(12,0.0);
        gen=new minJerkTrajGen(qd,Ts,T);

        return true;
    }

    /****************************************************************/
    bool close()
    {
        targetPort.close();
        solverPort.close();
        rpcPort.close();

        delete gen;
        for (int i=0; i<4; i++)
            drivers[i].close();

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
        Vector ref=gen->getPos();
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
                    Ts=std::max(Ts,0.01);
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

