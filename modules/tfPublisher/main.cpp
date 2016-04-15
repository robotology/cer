/*
* Copyright (C)2015  iCub Facility - Istituto Italiano di Tecnologia
* Author: Marco Randazzo
* email:  marco.randazzo@iit.it
* website: www.robotcub.org
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
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include "include/geometry_msgs_TransformStamped.h"
#include "include/tf_tfMessage.h"
#include <yarp/os/Publisher.h>
#include <limits>

#include <iostream>
#include <iomanip>
#include <string>

using namespace std;
using namespace yarp::os;

inline TickTime normalizeSecNSec(double yarpTimeStamp)
{
    uint64_t time = (uint64_t)(yarpTimeStamp * 1000000000UL);
    uint64_t nsec_part = (time % 1000000000UL);
    uint64_t sec_part = (time / 1000000000UL);
    TickTime ret;

    if (sec_part > std::numeric_limits<unsigned int>::max())
    {
        yWarning() << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }

    ret.sec = (yarp::os::NetUint32) sec_part;
    ret.nsec = (yarp::os::NetUint32) nsec_part;
    return ret;
}

class tf
{
    public:
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    string name;
    string parent_frame;
    string child_frame;
    enum tf_type_enum {fixed =0, external=1} type;
    yarp::os::BufferedPort<yarp::os::Bottle>* tfport;
    tf()
    {
        tfport = 0;
        type = fixed;
        x = 0; y = 0; z = 0; roll = 0; pitch = 0; yaw = 0;
    }
    ~tf()
    {
        if (tfport)
        {
            delete tfport;
            tfport = 0;
        }
    }
    void read()
    {
        if (tfport)
        {
            Bottle* bot = tfport->read(false);
            if (bot)
            {
                parent_frame=bot->get(0).asString();
                child_frame=bot->get(1).asString();
                x = bot->get(2).asDouble();
                y = bot->get(3).asDouble();
                z = bot->get(4).asDouble();
                roll = bot->get(5).asDouble();
                pitch = bot->get(6).asDouble();
                yaw = bot->get(7).asDouble();
            }
        }
    }

};

class tfModule: public RFModule
{
protected:
    Port                                      rpcPort;
    std::vector<tf>                           tfVector;
    yarp::os::Publisher<tf_tfMessage>         rosPublisherPort_tf;
    int                                       rosMsgCounter;
    double                                    period;

public:
    tfModule() 
    {
        rosMsgCounter = 0;
        period = 0.010;     //ms
    }

    virtual bool configure(ResourceFinder &rf)
    {
        string slash="/";
        string ctrlName;
        string robotName;
        string partName;
        string remoteName;
        string localName;

        Time::turboBoost();

        // get params from the RF
        ctrlName=rf.check("local",Value("baseControl")).asString();
        robotName=rf.check("robot",Value("cer")).asString();
        partName = rf.check("part", Value("mobile_base")).asString();

        remoteName=slash+robotName+slash+partName;
        localName=slash+ctrlName;
        
        //reads the configuration file
        Property ctrl_options;

        ConstString configFile=rf.findFile("from");
        if (configFile=="") //--from baseCtrl.ini
        {
            yError("Cannot find .ini configuration file. By default I'm searching for baseCtrl.ini");
            return false;
        }
        else
        {
            ctrl_options.fromConfigFile(configFile.c_str());
        }

        ctrl_options.put("remote", remoteName.c_str());
        ctrl_options.put("local", localName.c_str());

        //check for robotInterface availablity
        yInfo("Checking for robotInterface availability");
        Port startport;
        startport.open ("/baseControl/robotInterfaceCheck:rpc");
        

        Bottle cmd; cmd.addString("is_ready");
        Bottle response;
        int rc_count =0;
        int rp_count =0;
        int rf_count =0;
        double start_time=yarp::os::Time::now();
        bool not_yet_connected=true;

        //set the thread rate
        int rate = rf.check("rate",Value(20)).asInt();
        yInfo("baseCtrl thread rate: %d ms.",rate);

        rpcPort.open((localName+"/rpc").c_str());
        attach(rpcPort);

        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) 
    {
        reply.clear(); 
        if (command.get(0).asString()=="help")
        {
            reply.addVocab(yarp::os::Vocab::encode("many"));
            reply.addString("Available commands are:");
            reply.addString("run");

            return true;
        }
        else if (command.get(0).asString() == "create_frame")
        {
            if (command.size() == 11)
            {
                tf temp;
                temp.name = command.get(1).asString();
                if (command.get(2).asString() == "fixed") temp.type = tf::fixed;
                if (command.get(2).asString() == "external") temp.type = tf::external;
                temp.parent_frame = command.get(3).asString();
                temp.child_frame = command.get(4).asString();
                temp.x = command.get(5).asDouble();
                temp.y = command.get(6).asDouble();
                temp.z = command.get(7).asDouble();
                temp.roll = command.get(8).asDouble();
                temp.pitch = command.get(9).asDouble();
                temp.yaw = command.get(10).asDouble();
                for (size_t i = 0; i < tfVector.size(); i++)
                {
                    if (tfVector[i].name == temp.name)
                    {
                        reply.addString("already exists!");
                        return true;
                    }
                }
                reply.addString("ok");
                tfVector.push_back(temp);
                return true;
            }
            reply.addString("invalid params");
            return true;
        }
        else if (command.get(0).asString()=="delete_frame")
        {
            for (size_t i = 0; i < tfVector.size(); i++)
            {
                if (tfVector[i].name == command.get(1).asString())
                {
                    tfVector.erase(tfVector.begin()+i);
                    reply.addString("ok");
                    return true;
                }
            }
            reply.addString("not found");
            return true;

        }
        reply.addString("Unknown command.");
        return true;
    }

    virtual bool close()
    {

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()
    {
        return period;
    }

    virtual bool   updateModule()
    {
        tf_tfMessage &rosData = rosPublisherPort_tf.prepare();
        geometry_msgs_TransformStamped transform;

        if (rosData.transforms.size() != tfVector.size())
        {
            rosData.transforms.resize(tfVector.size());
        }

        for (size_t i = 0; i < this->tfVector.size(); i++)
        {
            
            transform.child_frame_id = tfVector[i].child_frame;
            transform.header.frame_id = tfVector[i].parent_frame;
            transform.header.seq = rosMsgCounter;
            transform.header.stamp = normalizeSecNSec(yarp::os::Time::now());

            geometry_msgs_TransformStamped transform;
            transform.transform.rotation.x = 0;
            transform.transform.rotation.y = 0;
            transform.transform.rotation.z = 0;
            transform.transform.rotation.w = 0;
            transform.transform.translation.x = 0;
            transform.transform.translation.y = 0;
            transform.transform.translation.z = 0;

            rosData.transforms[i] = transform;
        }

        rosMsgCounter++;
        return true;
    }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("cer");
    rf.setDefaultConfigFile("baseCtrl.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo("Possible options: ");
        yInfo("'rate <r>' sets the threads rate (default 20ms).");
        yInfo("'no_filter' disables command filtering.");
        yInfo("'joystick_connect' tries to automatically connect to the joystickCtrl output.");
        yInfo("'skip_robot_interface_check' does not connect to robotInterface/rpc (useful for simulator)");
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    tfModule mod;

    return mod.runModule(rf);
}
