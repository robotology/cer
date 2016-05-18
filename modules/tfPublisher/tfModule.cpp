
#include "tfModule.h"
#include <yarp/os/Nodes.h>

tfModule::tfModule()
{
    rosMsgCounter   = 0;
    period          = PERIOD;     //ms
    rosNode         = 0;
}

tfModule::~tfModule()
{
    close();
}

bool tfModule::configure(ResourceFinder &rf)
{
    Time::turboBoost();

    int         rate;
    Port        startport;
    Bottle      cmd, response;
    string      localName;
    double      start_time;

    localName   = string(RPCPORTNAME);
    start_time  = yarp::os::Time::now();
    rate        = rf.check("rate", Value(20)).asInt(); //set the thread rate
    rosNode     = new yarp::os::Node(ROSNODENAME);

    yInfo("tfPublisher thread rate: %d ms.", rate);

    rpcPort.open((localName + "/rpc").c_str());
    attach(rpcPort);

    if (!rosPublisherPort_tf.topic(ROSTOPICNAM))
    {
        yError() << " Unable to publish data on " << ROSTOPICNAM << " topic, check your yarp-ROS network configuration\n";
        return false;
    }

    if (!rosSubscriberPort_tf.topic(ROSTOPICNAM))
    {
        yError() << " Unable to subscribe to " << ROSTOPICNAM << " topic, check your yarp-ROS network configuration\n";
        return false;
    }



    return true;
}

bool tfModule::interruptModule()
{
    rpcPort.interrupt();
    rosPublisherPort_tf.interrupt();
    rosSubscriberPort_tf.interrupt();
    return true;
}

bool tfModule::respond(const Bottle& command, Bottle& reply)
{
    reply.clear();
    reply.addVocab(yarp::os::Vocab::encode("many"));
    string  request;
    request = command.get(0).asString();
    if ( request == "help")
    {
        return helpCmd(reply);
    }
    else if ( request == "create_fixed_frame")
    {

        if (command.size() != 10)
        {
            reply.addString("invalid params");
            return true;
        }

        if (rosHasFrame(command.get(2).asString(), command.get(3).asString()))
        {
            reply.addString("someone else is already injecting this frame");
            return true;
        }
        return createFixedFrameCmd(command, reply);
    }
    else if ( request == "delete_fixed_frame")
    {
        return deleteFixedFrameCmd(command, reply);
    }
    else if (request == "list")
    {
        return listCmd( reply );
    }
    else if (request == "get_frame")
    {
        return getFrameCmd(command.get(1).asString(), reply);
    }
    reply.addString("Unknown command.");
    return true;
}

bool tfModule::helpCmd( Bottle& reply )
{
    reply.addString("Available commands are:");
    reply.addString("  "); reply.addString("  ");
    reply.addString("'create_fixed_frame'   -   usage: create_fixed_frame <frameName> <parentName> <childName> <x> <y> <z> <roll> <pitch> <yaw>");
    reply.addString("  ");
    reply.addString("'delete_fixed_frame'   -   usage: 1. delete_fixed_frame <frameName> 2. delete_fixed_frame <parentName> <childName>");
    reply.addString("  ");
    reply.addString("'list' output all current frames name ");
    reply.addString("  ");
    reply.addString("'get_frame' output specified frame's data  -  usage: get_frame <frameName> ");
    return true;
}
bool tfModule::createFixedFrameCmd(const Bottle& command, Bottle& reply)
{
    tf        temp;
    double    x, y, z, roll, pitch, yaw;

    temp.name           = command.get(1).asString();
    temp.type           = tf::fixed;
    temp.parent_frame   = command.get(2).asString();
    temp.child_frame    = command.get(3).asString();
    x                   = command.get(4).asDouble();
    y                   = command.get(5).asDouble();
    z                   = command.get(6).asDouble();
    roll                = command.get(7).asDouble();
    pitch               = command.get(8).asDouble();
    yaw                 = command.get(9).asDouble();

    temp.transFromVec(x, y, z);
    temp.rotFromRPY(TORAD( roll ), TORAD( pitch ), TORAD( yaw ) );

    for (size_t i = 0; i < tfVector.size(); i++)
    {
        if (tfVector[i].name == temp.name)
        {
            reply.addString("already exists!");
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

    reply.addString("ok");
    tfVector.push_back(temp);
    return true;
}
bool tfModule::deleteFixedFrameCmd(const Bottle& command, Bottle& reply)
{
    bool    ret;
    int     cmdsize;
    cmdsize = int(command.size());
    switch ( cmdsize )
    {
        case 2:
            ret = deleteFrame( command.get(1).asString() );
            break;
        case 3:
            ret = deleteFrame( command.get(1).asString(), command.get(2).asString() );
            break;
        default:
            reply.addString("invalid params");
            return true;
    }

    if ( ret )
    {
        reply.addString("ok");
        return true;
    }
    else
    {
        reply.addString("not found");
        return true;
    }

}
bool tfModule::listCmd( Bottle& reply )
{
    string  tfCount;
    //char    buffer[100]; memset(buffer, 0, 100);

    tfCount = to_string(tfVector.size());//itoa(tfVector.size(), buffer, 10);

    reply.addString( "there are currently "+ tfCount + " frame/s" );

    for (size_t i = 0; i < tfVector.size(); i++)
    {
            reply.addString( tfVector[i].name + ", " );
    }

    reply.addString("  "); reply.addString("ros Tf:"); reply.addString("  ");

    for (size_t i = 0; i < rosTf.size(); i++)
    {
        reply.addString(rosTf[i].header.frame_id + ", " + rosTf[i].child_frame_id);
    }


    return true;
}
bool tfModule::getFrameCmd(const string& name, Bottle& reply)
{
    for (size_t i = 0; i < tfVector.size(); i++)
    {
        if (tfVector[i].name == name)
        {
            replyFrameInfo( tfVector[i], reply );
        }
    }
    return true;
}

void tfModule::replyFrameInfo(const tf& frame, Bottle& reply)
{
    Vector rpyVec;
    string sep;
    sep = ", ";
    rpyVec = frame.getRPYRot();
    reply.addString( "Name: " + frame.name );
    reply.addString( "Parent: " + frame.parent_frame );
    reply.addString( "Child: " + frame.child_frame );
    reply.addString( "Traslation(X - Y - Z): " + TEXT( frame.tX ) + sep + TEXT( frame.tY ) + sep + TEXT( frame.tZ ) );
    reply.addString(
                    "Rotation(R - P - Y in degrees): " +
                    TEXT( TODEG( rpyVec[0] ) ) + sep +
                    TEXT( TODEG( rpyVec[1] ) ) + sep +
                    TEXT( TODEG( rpyVec[2] ) )
                   );
}

bool tfModule::deleteFrame( const string& name )
{
    bool ret;
    ret  = false;

    for (size_t i = 0; i < tfVector.size(); i++)
    {
        if ( tfVector[i].name == name )
        {
            tfVector.erase( tfVector.begin() + i );
            ret = true;
        }
    }
    return ret;
}

bool tfModule::deleteFrame( const string& parent, const string& child )
{
    bool ret;
    ret  = false;

    for (size_t i = 0; i < tfVector.size(); i++)
    {
        if (
            tfVector[i].parent_frame == parent && tfVector[i].child_frame == child ||
            tfVector[i].parent_frame == child && tfVector[i].child_frame == parent
           )
        {
            tfVector.erase( tfVector.begin() + i );
            ret = true;
        }
    }
    return ret;
}

bool tfModule::close()
{
    rpcPort.close();
    rosPublisherPort_tf.close();
    rosSubscriberPort_tf.close();
    if (rosNode)
    {
        delete  rosNode;
        rosNode = 0;
    }

    return true;
}

double tfModule::getPeriod()
{
    return period;
}

bool tfModule::rosHasFrame( string parent, string child )
{
    for (size_t i = 0; i < rosTf.size(); i++)
    {
        if (
            rosTf[i].header.frame_id == parent && rosTf[i].child_frame_id == child ||
            rosTf[i].header.frame_id == child && rosTf[i].child_frame_id == parent
           )
        {
            return true;
        }
    }

    return false;
}

bool tfModule::updateModule()
{
    tf_tfMessage*                     rosInData  = rosSubscriberPort_tf.read( false );
    tf_tfMessage&                     rosOutData = rosPublisherPort_tf.prepare();
    geometry_msgs_TransformStamped    transform;
    unsigned int                      tfVecSize;

    if( rosInData != 0 )
    {
        rosTf = rosInData->transforms;
    }

    tfVecSize = tfVector.size();

    if (rosOutData.transforms.size() != tfVecSize)
    {
        rosOutData.transforms.resize(tfVecSize);
    }

    for( size_t i = 0; i < tfVecSize; i++ )
    {

        transform.child_frame_id            = tfVector[i].child_frame;
        transform.header.frame_id           = tfVector[i].parent_frame;
        transform.header.seq                = rosMsgCounter;
        transform.header.stamp              = normalizeSecNSec(yarp::os::Time::now());
        transform.transform.rotation.x      = tfVector[i].rX;
        transform.transform.rotation.y      = tfVector[i].rY;
        transform.transform.rotation.z      = tfVector[i].rZ;
        transform.transform.rotation.w      = tfVector[i].rW;
        transform.transform.translation.x   = tfVector[i].tX;
        transform.transform.translation.y   = tfVector[i].tY;
        transform.transform.translation.z   = tfVector[i].tZ;

        rosOutData.transforms[i]            = transform;
    }
    rosPublisherPort_tf.write();
    rosMsgCounter++;
    return true;
}
