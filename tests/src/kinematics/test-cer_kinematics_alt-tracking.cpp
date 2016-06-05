/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Alessandro Scalzo
 * email:  alessandro.scalzo@iit.it
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

#include <time.h>
#include <string>

#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <cer_kinematics_alt/Solver.h>

#include <cmath>

using namespace cer::kinematics_alt;

class RobotThread : public yarp::os::RateThread
{
public:
    RobotThread();

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();
    virtual void onStop();

    void sendConfig(yarp::sig::Vector &q);
    void sendCOM(std::string& name,int R,int G,int B,yarp::sig::Vector P,double size,double alpha);
    void sendTarget(const std::string& name,int R,int G,int B,double x,double y,double z,double size,double alpha,double rx=0.0,double ry=0.0,double rz=0.0);

protected:
    KinR1 mRobot;

    yarp::sig::Vector qsol;

    double finger[9];

    yarp::os::BufferedPort<yarp::sig::Vector> portEncBase;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncTorso;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncHead;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncLeftArm;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncRightArm;

    yarp::os::BufferedPort<yarp::os::Bottle> portObjects;
};


RobotThread::RobotThread() : RateThread(int(PERIOD*1000.0)),qsol(22) //RateThread(int(PERIOD*1000.0))
{
    finger[0]=30.0;
    finger[1]=60.0;

    for (int i=2; i<9; ++i) finger[i]=10.0;
}

bool RobotThread::threadInit()
{
    portEncBase.open("/CERControl/base:o");
    portEncTorso.open("/CERControl/torso:o");
    portEncHead.open("/CERControl/head:o");
    portEncLeftArm.open("/CERControl/left_arm:o");
    portEncRightArm.open("/CERControl/right_arm:o");
    portObjects.open("/CERControl/objects:o");

    yarp::os::Network::connect("/CERControl/base:o","/CERGui/base:i");
    yarp::os::Network::connect("/CERControl/torso:o","/CERGui/torso:i");
    yarp::os::Network::connect("/CERControl/head:o","/CERGui/head:i");
    yarp::os::Network::connect("/CERControl/left_arm:o","/CERGui/left_arm:i");
    yarp::os::Network::connect("/CERControl/right_arm:o","/CERGui/right_arm:i");
    yarp::os::Network::connect("/CERControl/objects:o","/CERGui/objects:i");

    yarp::os::Bottle& bot=portObjects.prepare();
    bot.clear();
    bot.addString("reset");
    portObjects.write();

    srand((unsigned)time(NULL));

    mRobot.getConfig(qsol);

    return true;
}

void RobotThread::onStop()
{
}

void RobotThread::threadRelease()
{
    portEncBase.interrupt();
    portEncTorso.interrupt();
    portEncHead.interrupt();
    portEncLeftArm.interrupt();
    portEncRightArm.interrupt();
    portObjects.interrupt();

    portEncBase.close();
    portEncTorso.close();
    portEncHead.close();
    portEncLeftArm.close();
    portEncRightArm.close();
    portObjects.close();
}

/*
void RobotThread::run()
{
    yarp::sig::Matrix Tl(4,4),Tr(4,4);

    static double alfa=0.0;
    
    alfa+=PERIOD*0.1;

    Tl(0,0)= 1.0; Tl(0,1)= 0.0; Tl(0,2)= 0.0; Tl(0,3)=0.5;
    Tl(1,0)= 0.0; Tl(1,1)= 1.0; Tl(1,2)= 0.0; Tl(1,3)= 0.2+0.2*sin(2.0*M_PI*alfa);
    Tl(2,0)= 0.0; Tl(2,1)= 0.0; Tl(2,2)= 1.0; Tl(2,3)= 0.8+0.2*cos(2.0*M_PI*alfa);
    Tl(3,0)= 0.0; Tl(3,1)= 0.0; Tl(3,2)= 0.0; Tl(3,3)=1.0;

    Tr(0,0)= 0.0; Tr(0,1)= 0.0; Tr(0,2)= 1.0; Tr(0,3)=0.4;
    Tr(1,0)= 0.0; Tr(1,1)= 1.0; Tr(1,2)= 0.0; Tr(1,3)=-0.2+0.2*sin(3.0*M_PI*alfa);
    Tr(2,0)=-1.0; Tr(2,1)= 0.0; Tr(2,2)= 0.0; Tr(2,3)= 0.5+0.2*cos(3.0*M_PI*alfa);
    Tr(3,0)= 0.0; Tr(3,1)= 0.0; Tr(3,2)= 0.0; Tr(3,3)=1.0;

    sendTarget(std::string("targetL"),0,64,255,Tl(0,3),Tl(1,3),Tl(2,3),16.0,0.666);
    sendTarget(std::string("targetR"),0,64,255,Tr(0,3),Tr(1,3),Tr(2,3),16.0,0.666);

    yarp::sig::Vector qvel(22);

    //mRobot.ikin_2hand_ctrl(Tl,Tr,qsol,qvel,0.02,0.02,0.1);
    //mRobot.ikin_left_ctrl(Tl,qsol,qvel,0.02,0.02,0.1);
    mRobot.ikin_right_ctrl(Tr,qsol,qvel,0.02,0.02,0.1);

    for (int j=0; j<22; ++j) qsol(j)+=PERIOD*qvel(j);
    
    sendConfig(qsol);

    yarp::sig::Vector COM=mRobot.getCOM(); 
    COM[2]=-0.160;
    sendCOM(std::string("G"),192,192,0,COM,16.0,1.0);
}
*/


void RobotThread::run()
{
    yarp::sig::Matrix Tl(4,4),Tr(4,4);

    static double alfa=0.0;
    
    alfa+=PERIOD*0.1;

    Tl(0,0)= 1.0; Tl(0,1)= 0.0; Tl(0,2)= 0.0; Tl(0,3)=0.4;
    Tl(1,0)= 0.0; Tl(1,1)= 1.0; Tl(1,2)= 0.0; Tl(1,3)= 0.2+0.2*sin(2.0*M_PI*alfa);
    Tl(2,0)= 0.0; Tl(2,1)= 0.0; Tl(2,2)= 1.0; Tl(2,3)= 0.8+0.2*cos(2.0*M_PI*alfa);
    Tl(3,0)= 0.0; Tl(3,1)= 0.0; Tl(3,2)= 0.0; Tl(3,3)=1.0;

    Tr(0,0)= 0.0; Tr(0,1)= 0.0; Tr(0,2)= 1.0; Tr(0,3)=0.4;
    Tr(1,0)= 0.0; Tr(1,1)= 1.0; Tr(1,2)= 0.0; Tr(1,3)=-0.2+0.2*sin(3.0*M_PI*alfa);
    Tr(2,0)=-1.0; Tr(2,1)= 0.0; Tr(2,2)= 0.0; Tr(2,3)= 0.5+0.2*cos(3.0*M_PI*alfa);
    Tr(3,0)= 0.0; Tr(3,1)= 0.0; Tr(3,2)= 0.0; Tr(3,3)=1.0;

    sendTarget(std::string("targetL"),0,64,255,Tl(0,3),Tl(1,3),Tl(2,3),16.0,0.666);
    sendTarget(std::string("targetR"),0,64,255,Tr(0,3),Tr(1,3),Tr(2,3),16.0,0.666);

    mRobot.ikin_2hand_solver(Tl,Tr,qsol,0.02,0.02,0.1);
    //mRobot.ikin_left_solver(Tl,qsol,0.02,0.02,0.1);
    //mRobot.ikin_right_solver(Tr,qsol,0.02,0.02,0.1);

    sendConfig(qsol);

    yarp::sig::Vector COM=mRobot.getCOM(); 
    COM[2]=-0.160;
    sendCOM(std::string("G"),192,192,0,COM,16.0,1.0);
}

void RobotThread::sendConfig(yarp::sig::Vector &q)
{
    if (portEncBase.getOutputCount()>0)
    {
        yarp::sig::Vector& enc=portEncBase.prepare();
        enc.clear();

        //Vec3 P=1000.*mRobot.getT().Pj();
        //Vec3 R=mRobot.getT().Rj().rpy();

        //enc.push_back(R.x);
        //enc.push_back(R.y);
        //enc.push_back(R.z);

        //enc.push_back(P.x);
        //enc.push_back(P.y);
        //enc.push_back(P.z);

        enc.push_back(0.0);
        enc.push_back(0.0);
        enc.push_back(0.0);
        enc.push_back(0.0);
        enc.push_back(0.0);
        enc.push_back(0.0);

        portEncBase.write();
    }

    if (portEncTorso.getOutputCount()>0)
    {
        yarp::sig::Vector& enc=portEncTorso.prepare();
        enc.clear();
        enc.push_back(360.0+1000.0*q(0));
        enc.push_back(360.0+1000.0*q(1));
        enc.push_back(360.0+1000.0*q(2));
        enc.push_back(q(3));
        portEncTorso.write();
    }

    if (portEncLeftArm.getOutputCount()>0)
    {
        yarp::sig::Vector& enc=portEncLeftArm.prepare();
        enc.clear();
        for (int i=4; i<9; ++i)  enc.push_back(q(i));
        for (int i=9; i<12; ++i) enc.push_back(220.0+1000.0*q(i));
        //for (int i=0; i<9; ++i) enc.push_back(finger[i]);
        portEncLeftArm.write();
    }

    if (portEncRightArm.getOutputCount()>0)
    {
        yarp::sig::Vector& enc=portEncRightArm.prepare();
        enc.clear();
        for (int i=12; i<17; ++i) enc.push_back(q(i));
        for (int i=17; i<20; ++i) enc.push_back(220.0+1000.0*q(i));
        //for (int i=0; i<9; ++i) enc.push_back(finger[i]);
        portEncRightArm.write();
    }

    if (portEncHead.getOutputCount()>0)
    {
        yarp::sig::Vector& enc=portEncHead.prepare();
        enc.clear();
        for (int i=20; i<22; ++i) enc.push_back(q(i));
        portEncHead.write();
    }
}

void RobotThread::sendCOM(std::string& name,int R,int G,int B,yarp::sig::Vector P,double size,double alpha)
{
    yarp::os::Bottle& botR=portObjects.prepare();
    botR.clear();
    botR.addString("object_with_label");
    botR.addString(name); botR.addString(name);
    botR.addDouble(size);
    botR.addDouble(0.0);
    botR.addDouble(0.0);
    botR.addDouble(P[0]*1000.0);
    botR.addDouble(P[1]*1000.0);
    botR.addDouble(P[2]*1000.0);

    botR.addDouble(0.0); botR.addDouble(0.0); botR.addDouble(0.0);
    
    botR.addInt(R); botR.addInt(G); botR.addInt(B);
    botR.addDouble(alpha);
    //botR.addString("WORLD");
    portObjects.writeStrict();
}

void RobotThread::sendTarget(const std::string& name,int R,int G,int B,double x,double y,double z,double size,double alpha,double rx,double ry,double rz)
{
    yarp::os::Bottle& botR=portObjects.prepare();
    botR.clear();
    botR.addString("object"); botR.addString(name);
    botR.addDouble(size);
    botR.addDouble(0.0);
    botR.addDouble(0.0);
    botR.addDouble(x*1000.0);
    botR.addDouble(y*1000.0);
    botR.addDouble(z*1000.0);

    botR.addDouble(rx); 
    botR.addDouble(ry); 
    botR.addDouble(rz);

    botR.addInt(R); botR.addInt(G); botR.addInt(B);
    botR.addDouble(alpha);
    //botR.addString("WORLD");
    portObjects.writeStrict();
}




class CerTestModule : public yarp::os::RFModule
{
protected:
    RobotThread *mRobotThread;
    yarp::os::Port mHandlerPort;

public:
    CerTestModule()
    {
        mRobotThread=NULL;
    }

    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        yarp::os::Time::turboBoost();

        mRobotThread=new RobotThread();

        if (!mRobotThread->start())
        {
            delete mRobotThread;
            return false;
        }

        mHandlerPort.open("/icubwalk");
        attach(mHandlerPort);
        //attachTerminal();

        return true;
    }

    virtual bool interruptModule()
    {
        mHandlerPort.interrupt();

        return true;
    }

    virtual bool close()
    {
        mHandlerPort.interrupt();
        mHandlerPort.close();

        mRobotThread->stop();
        delete mRobotThread;
        mRobotThread=NULL;

        return true;
    }

    virtual double getPeriod()
    {
        return 1.0;
    }

    virtual bool updateModule()
    {
        if (isStopping())
        {
            mRobotThread->stop();
            return false;
        }

        return true;
    }

    virtual bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply)
    {
        if (command.get(0).asString()=="quit") return false;

        return true;
    }
};

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr,"ERROR: check Yarp network.\n");
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("iCubWalk.ini");
    rf.setDefaultContext("iCubWalk");
    rf.configure(argc,argv);

    CerTestModule CER;

    return CER.runModule(rf);
}

    //sendCOM(std::string("LUarm"),192,192,0,mRobot.G[3],30.0,0.5);
    //sendCOM(std::string("LLarm"),192,192,0,mRobot.G[4],30.0,0.5);
    //sendCOM(std::string("Lhand"),192,192,0,mRobot.G[5],30.0,0.5);

    //sendCOM(std::string("base"),192,192,0,mRobot.G[0],30.0,0.5);
    //sendCOM(std::string("body"),192,192,0,mRobot.G[1],30.0,0.5);
    //sendCOM(std::string("head"),192,192,0,mRobot.G[2],30.0,0.5);

    //sendCOM(std::string("RUarm"),192,192,0,mRobot.G[6],30.0,0.5);
    //sendCOM(std::string("RLarm"),192,192,0,mRobot.G[7],30.0,0.5);
    //sendCOM(std::string("Rhand"),192,192,0,mRobot.G[8],30.0,0.5);