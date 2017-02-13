/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
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

#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <SelfCollisionLib.h>

using namespace cer::robot_model::self_collision;

class SelfCollisionThread : public yarp::os::Thread
{
public:
	SelfCollisionThread(yarp::os::ConstString name, int robot_type);

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();
    virtual void onStop();

	bool freeSpace()
	{
		return bFreeSpace;
	}

protected:
	SelfCollisionLib collisionCheck;

	yarp::os::ConstString robotName;

	bool bFreeSpace;

	yarp::os::BufferedPort<yarp::sig::Vector> portCheckI;
	yarp::os::BufferedPort<yarp::sig::Vector> portMarginO;
	yarp::os::BufferedPort<yarp::sig::Matrix> portJacobianO;
};

SelfCollisionThread::SelfCollisionThread(yarp::os::ConstString name, int robot_type) : Thread(), robotName(name), collisionCheck(robot_type)
{
	bFreeSpace = true;
}

bool SelfCollisionThread::threadInit()
{
	if (!collisionCheck.isOk()) return false;

	portCheckI.open(yarp::os::ConstString("/") + robotName + "/self/check:i");
	portMarginO.open(yarp::os::ConstString("/") + robotName + "/self/margin:o");
	portJacobianO.open(yarp::os::ConstString("/") + robotName + "/self/jacobian:o");

    return true;
}

void SelfCollisionThread::onStop()
{
	portCheckI.interrupt();
	portMarginO.interrupt();
	portJacobianO.interrupt();
}

void SelfCollisionThread::threadRelease()
{
	portCheckI.interrupt();
	portMarginO.interrupt();
	portJacobianO.interrupt();

	portCheckI.close();
	portMarginO.close();
	portJacobianO.close();
}

void SelfCollisionThread::run()
{
	while (isRunning())
	{
		yarp::sig::Vector *qnext = portCheckI.read();

		if (isStopping()) return;

		if (!qnext) continue;

		yarp::sig::Vector *margin = (portMarginO.getOutputCount() > 0) ? &(portMarginO.prepare()) : NULL;

		yarp::sig::Matrix *jacobian = (portJacobianO.getOutputCount() > 0) ? &(portJacobianO.prepare()) : NULL;

		if (margin || jacobian)
		{
			bFreeSpace = collisionCheck.checkNextConfiguration(*qnext, margin, jacobian);

			if (margin) portMarginO.writeStrict();

			if (jacobian) portJacobianO.writeStrict();
		}
	}
}




class SelfCollisionModule : public yarp::os::RFModule
{
public:
	SelfCollisionModule()
	{
		selfCollisionThread = NULL;
	}

	~SelfCollisionModule()
	{
		if (selfCollisionThread) delete selfCollisionThread;
	}

	double getPeriod(){ return 1.0; }

	bool configure(yarp::os::ResourceFinder &rf)
	{
		if (!rf.check("model") || !rf.check("name"))
		{
			printf("usage:\n % selfCollision --name <myname> --model <R1 | iCub | iCub3>\n");
			return false;
		}

		yarp::os::ConstString sRobotType = rf.find("model").asString();
		yarp::os::ConstString sRobotName = rf.find("name").asString();

		if (sRobotType == "R1")
			selfCollisionThread = new SelfCollisionThread(sRobotName, SelfCollisionLib::R1_MODEL);
		else if (sRobotType == "iCub")
			selfCollisionThread = new SelfCollisionThread(sRobotName, SelfCollisionLib::ICUB_MODEL);
		else if (sRobotType == "iCub3")
			selfCollisionThread = new SelfCollisionThread(sRobotName, SelfCollisionLib::ICUB3_MODEL);
		else 		
		{
			printf("Usage:\n % selfCollision --name <myname> --model <R1 | iCub | iCub3>\n");
			return false;
		}

		if (!selfCollisionThread->start())
		{
			printf("Thread could not start, aborting.\n");
			return false;
		}

		return true;
	}

	bool updateModule()
	{
		if (selfCollisionThread->freeSpace())
		{
			printf("freespace\n");
		}
		else
		{
			printf("COLLISION\n");
		}

		return true;
	}

	bool interruptModule()
	{
		selfCollisionThread->stop();

		return true;
	}

protected:
	SelfCollisionThread *selfCollisionThread;
};



int main(int argc, char** argv)
{
	yarp::os::Network yarp;

	argc = 5;
	char* fargv[] = {"selfCollision","--name", "r1", "--model","R1"};

	yarp::os::ResourceFinder rf;
	rf.setVerbose(true);
	rf.configure(argc, fargv);

	SelfCollisionModule module;

	module.runModule(rf);

	return 0;
}