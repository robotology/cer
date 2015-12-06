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

using namespace cer::kinematics2;

class RobotThread : public yarp::os::RateThread
{
public:
    RobotThread();

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();
    virtual void onStop();

    void sendConfig(yarp::sig::Vector &q);
	void sendCOM(std::string& name,int R,int G,int B,Vec3& P,double size,double alpha);
	void sendTarget(const std::string& name,int R,int G,int B,double x,double y,double z,double size,double alpha);

protected:
    LeftSideSolver mRobot;

	double finger[9];

    yarp::os::BufferedPort<yarp::sig::Vector> portEncBase;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncTorso;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncHead;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncLeftArm;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncRightArm;

	yarp::os::BufferedPort<yarp::os::Bottle> portObjects;
};


RobotThread::RobotThread() : RateThread(int(PERIOD*1000.0)) //RateThread(int(PERIOD*1000.0))
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

void RobotThread::run()
{
	yarp::sig::Matrix T(4,4);

	static double ti=0.0;

	T(0,0)= 0.0; T(0,1)= 0.0; T(0,2)= 1.0; T(0,3)=0.35;//radius*cos(DEG2RAD*alfa);
	T(1,0)= 0.0; T(1,1)= 1.0; T(1,2)= 0.0; T(1,3)=0.1*cos(0.4*2.0*M_PI*ti);//radius*sin(DEG2RAD*alfa);
	T(2,0)=-1.0; T(2,1)= 0.0; T(2,2)= 0.0; T(2,3)=0.1*sin(0.4*2.0*M_PI*ti);//0.7-0.63;
	T(3,0)= 0.0; T(3,1)= 0.0; T(3,2)= 0.0; T(3,3)=1.0;

	ti+=PERIOD;

	sendTarget(std::string("target"),0,64,255,T(0,3),T(1,3),T(2,3),10.0,0.666);

	yarp::sig::Vector qsol(22);

	double timeA,timeB;

	timeA=yarp::os::Time::now();

	bool success=mRobot.ikin(T,qsol);

	timeB=yarp::os::Time::now();

	sendConfig(qsol);

	printf("time = %d ms\n",int(1000.0*(timeB-timeA)));
	//fflush(stdout);

	//Vec3 COM=mRobot.getCOM();
	//COM.z=-0.63;
	//sendCOM(std::string("G"),192,192,0,COM,16.0,1.0);

	//sendCOM(std::string("LUarm"),192,192,0,mRobot.G[3],30.0,0.5);
	//sendCOM(std::string("LLarm"),192,192,0,mRobot.G[4],30.0,0.5);
	//sendCOM(std::string("Lhand"),192,192,0,mRobot.G[5],30.0,0.5);

	//sendCOM(std::string("base"),192,192,0,mRobot.G[0],30.0,0.5);
	//sendCOM(std::string("body"),192,192,0,mRobot.G[1],30.0,0.5);
	//sendCOM(std::string("head"),192,192,0,mRobot.G[2],30.0,0.5);

	//sendCOM(std::string("RUarm"),192,192,0,mRobot.G[6],30.0,0.5);
	//sendCOM(std::string("RLarm"),192,192,0,mRobot.G[7],30.0,0.5);
	//sendCOM(std::string("Rhand"),192,192,0,mRobot.G[8],30.0,0.5);

	/*
	if (success)
	{
		printf("SUCCESS\n");
		sendTarget(target_name,0,200,0,T(0,3),T(1,3),T(2,3),10.0,1.0);
		if (dumper)
		{
			fprintf(dumper,"1 %.3f %.3f %.3f ",T(0,3),T(1,3),T(2,3));
			//fprintf(dumper,"%.3f %.3f %.3f\n",COM.x,COM.y,COM.z);
		}
	}
	else
	{
		printf("OUT_OF_REACH\n");
		sendTarget(target_name,255,0,32,T(0,3),T(1,3),T(2,3),10.0,1.0);
		if (dumper)
		{
			fprintf(dumper,"0 %.3f %.3f %.3f ",T(0,3),T(1,3),T(2,3));
			//fprintf(dumper,"%.3f %.3f %.3f\n",COM.x,COM.y,COM.z);
		}
	}
	*/
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
		enc.push_back(410.0+1000.0*q(0));
        enc.push_back(410.0+1000.0*q(1));
        enc.push_back(410.0+1000.0*q(2));
        enc.push_back(q(3));
		portEncTorso.write();
    }

    if (portEncLeftArm.getOutputCount()>0)
    {
        yarp::sig::Vector& enc=portEncLeftArm.prepare();
        enc.clear();
        for (int i=4; i<9; ++i) enc.push_back(q(i));
		for (int i=9; i<12; ++i) enc.push_back(1000.0*q(i));
		//for (int i=0; i<9; ++i) enc.push_back(finger[i]);
        portEncLeftArm.write();
    }

    if (portEncRightArm.getOutputCount()>0)
    {
        yarp::sig::Vector& enc=portEncRightArm.prepare();
        enc.clear();
        for (int i=12; i<17; ++i) enc.push_back(q(i));
		for (int i=17; i<20; ++i) enc.push_back(1000.0*q(i));
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

void RobotThread::sendCOM(std::string& name,int R,int G,int B,Vec3& P,double size,double alpha)
{
	yarp::os::Bottle& botR=portObjects.prepare();
	botR.clear();
	botR.addString("object_with_label"); 
	botR.addString(name); botR.addString(name);
	botR.addDouble(size);
	botR.addDouble(0.0); 
	botR.addDouble(0.0);
	botR.addDouble(P.x*1000.0); 
	botR.addDouble(P.y*1000.0);
	botR.addDouble(P.z*1000.0);
	//botR.addDouble(-630.0);

	botR.addDouble(0.0); botR.addDouble(0.0); botR.addDouble(0.0);
	botR.addInt(R); botR.addInt(G); botR.addInt(B);
	botR.addDouble(alpha);
	//botR.addString("WORLD");
	portObjects.writeStrict();
}

void RobotThread::sendTarget(const std::string& name,int R,int G,int B,double x,double y,double z,double size,double alpha)
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
		
	botR.addDouble(0.0); botR.addDouble(0.0); botR.addDouble(0.0);
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
    rf.setDefaultContext("/conf/iCubWalk");
    rf.configure("ICUB_ROOT",argc,argv);

    CerTestModule CER;

    return CER.runModule(rf);
}
