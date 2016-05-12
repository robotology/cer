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
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Log.h>


#include "tfModule.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::math;
using namespace yarp::sig;



/*class tf
{
    public:
    
    enum      tf_type_enum { fixed = 0, external = 1 } type;
    double    tX;
    double    tY;
    double    tZ;
    double    rX;
    double    rY;
    double    rZ;
    double    rW;
    string    name;
    string    parent_frame;
    string    child_frame;
    
    yarp::os::BufferedPort<yarp::os::Bottle>* tfport;
    
    tf()
    {
        tfport  = 0;
        type    = fixed;
        tX      = tY = tZ = rX = rY = rZ = rW = 0;
    }

    ~tf()
    {
        if ( tfport )
        {
            delete  tfport;
            tfport  = 0;
        }
    }
    void read()
    {
        if ( tfport )
        {
            Bottle* bot;
            bot     = tfport->read( false );
            if ( bot )
            {
                parent_frame  = bot->get(0).asString();
                child_frame   = bot->get(1).asString();
                tX            = bot->get(2).asDouble();
                tY            = bot->get(3).asDouble();
                tZ            = bot->get(4).asDouble();
                rX            = bot->get(5).asDouble();
                rY            = bot->get(6).asDouble();
                rZ            = bot->get(7).asDouble();
                rW            = bot->get(8).asDouble();
            }
        }
    }
    void transFromVec(double X, double Y, double Z)
    {
        tX = X;
        tY = Y;
        tZ = Z;
    }
    void rotFromRPY(double R, double P, double Y)
    {
        double    rot[3];
        size_t    i;
        Vector    rotV, rotQ;
        Matrix    rotM;
        i         = 3;
        rot[0]    = R; rot[1] = P; rot[2] = Y;
        rotV      = Vector(i, rot);
        rotM      = rpy2dcm( rotV );
        rotQ      = dcm2quat( rotM );
        rW        = rotQ[0];
        rX        = rotQ[1]; 
        rY        = rotQ[2]; 
        rZ        = rotQ[3];
        return;
    }

};

class tfModule: public RFModule
{
protected:
    Port                                 rpcPort;
    std::vector<tf>                      tfVector;
    yarp::os::Publisher<tf_tfMessage>    rosPublisherPort_tf;
    int                                  rosMsgCounter;
    double                               period;
    yarp::os::Node*                      rosNode;

public:
    tfModule() 
    {
        rosMsgCounter   = 0;
        period          = PERIOD;     //ms
    }

    ~tfModule()
    {
        close();
    }

    virtual bool configure( ResourceFinder &rf )
    {
        Time::turboBoost();

        int       rate;
        Port      startport;
        Bottle    cmd, response;
        string    localName;
        double    start_time;

        localName = string(RPCPORTNAME);
        start_time    = yarp::os::Time::now();
        rate          = rf.check("rate", Value( 20 )).asInt(); //set the thread rate
        rosNode = new yarp::os::Node(ROSNODENAME);

        yInfo("tfPublisher thread rate: %d ms.", rate);

        rpcPort.open( ( localName+"/rpc" ).c_str() );
        attach( rpcPort );

        if (!rosPublisherPort_tf.topic(ROSTOPICNAM))
        {
            yError() << " opening " << ROSTOPICNAM << " Topic, check your yarp-ROS network configuration\n";
            return false;
        }

        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) 
    {
        reply.clear();

        if ( command.get(0).asString()=="help" )
        {
            reply.addVocab( yarp::os::Vocab::encode( "many" ) );
            reply.addString( "Available commands are:" );
            reply.addString( "create_fixed_frame   -   usage: create_fixed_frame <frameName> <parentName> <childName> <x> <y> <z> <roll> <pitch> <yaw>" );
            reply.addString( "delete_fixed_frame   -   usage: 1. delete_fixed_frame <frameName> 2. delete_fixed_frame <parentName> <childName>");

            return true;
        }
        else if ( command.get(0).asString() == "create_fixed_frame" )
        {
            if (command.size() != 10)
            {
                reply.addString("invalid params");
                return true;
            }
            
            tf        temp;
            double    x, y, z, roll, pitch, yaw;

            temp.name           = command.get( 1 ).asString();
            temp.type           = tf::fixed;                
            temp.parent_frame   = command.get( 2 ).asString();
            temp.child_frame    = command.get( 3 ).asString();
            x                   = command.get( 4 ).asDouble();
            y                   = command.get( 5 ).asDouble();
            z                   = command.get( 6 ).asDouble();
            roll                = command.get( 7 ).asDouble();
            pitch               = command.get( 8 ).asDouble();
            yaw                 = command.get( 9 ).asDouble();

            temp.transFromVec( x, y, z );
            temp.rotFromRPY( roll, pitch, yaw );
            
            for ( size_t i = 0; i < tfVector.size(); i++ )
            {
                if ( tfVector[i].name == temp.name )
                {
                    reply.addString( "already exists!" );
                    return true;
                }

                if (
                    tfVector[i].parent_frame == temp.parent_frame && tfVector[i].child_frame == temp.child_frame ||
                    tfVector[i].parent_frame == temp.child_frame && tfVector[i].child_frame == temp.parent_frame
                   )
                {
                    reply.addString("tf with the same hierarchy already exists!");
                    return true;
                }
            }

            reply.addString( "ok" );
            tfVector.push_back( temp );
            return true;
        }
        else if ( command.get(0).asString() == "delete_frame" )
        {
            for ( size_t i = 0; i < tfVector.size(); i++ )
            {
                if ( tfVector[i].name == command.get(1).asString() )
                {
                    tfVector.erase( tfVector.begin() + i );
                    reply.addString( "ok" );
                    return true;
                }
            }
            reply.addString( "not found" );
            return true;

        }
        reply.addString( "Unknown command." );
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
        tf_tfMessage&                     rosData = rosPublisherPort_tf.prepare();
        geometry_msgs_TransformStamped    transform;
        int                               tfVecSize;

        tfVecSize = tfVector.size();
        
        if (rosData.transforms.size() != tfVecSize)
        {
            rosData.transforms.resize( tfVecSize );
        }

        for (size_t i = 0; i < tfVecSize; i++)
        {
            
            transform.child_frame_id             = tfVector[i].child_frame;
            transform.header.frame_id            = tfVector[i].parent_frame;
            transform.header.seq                 = rosMsgCounter;
            transform.header.stamp               = normalizeSecNSec(yarp::os::Time::now());
            transform.transform.rotation.x       = tfVector[i].rX;
            transform.transform.rotation.y       = tfVector[i].rY;
            transform.transform.rotation.z       = tfVector[i].rZ;
            transform.transform.rotation.w       = tfVector[i].rW;
            transform.transform.translation.x    = tfVector[i].tX;
            transform.transform.translation.y    = tfVector[i].tY;
            transform.transform.translation.z    = tfVector[i].tZ;

            rosData.transforms[i] = transform;
        }
        rosPublisherPort_tf.write();
        rosMsgCounter++;
        return true;
    }
};*/



int main(int argc, char* argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("cer");
    rf.configure(argc,argv);

    if ( rf.check("help") )
    {
        yInfo( "Possible options: " );
        yInfo( "'rate <r>' sets the threads rate (default 20ms)." );
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
