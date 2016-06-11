
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
#include <yarp/os/Node.h>
#include "include/geometry_msgs_TransformStamped.h"
#include "include/tf_tfMessage.h"
#include <yarp/os/Publisher.h>
#include <yarp/os/Subscriber.h>
#include <limits>
#include <iostream>
#include <iomanip>
#include <string>
#include "tf.h"

#define PI          3.14159265359
#define RPCPORTNAME "/tfpublisher"
#define PERIOD      0.010
#define ROSTOPICNAM "/tf"
#define TEXT( x )   to_string( x )
#define TODEG( x )  x * ( 180 / PI )
#define TORAD( x )  (PI / 180) * x
typedef yarp::os::Publisher<tf_tfMessage> tfPub;
typedef yarp::os::Subscriber<tf_tfMessage> tfSub;
#define YOS yarp::os


typedef geometry_msgs_TransformStamped tfStamped;

class tfModule : public YOS::RFModule
{
protected:
	YOS::Port               rpcPort, rosReaderPort;
    std::vector<tf>         tfVector;
    std::vector<tf>         extTfVector;
    tfPub                   rosPublisherPort_tf;
    tfSub                   rosSubscriberPort_tf;
    int                     rosMsgCounter;
    double                  period;
    yarp::os::Node*         rosNode;
    std::vector<tfStamped>  rosTf;
    bool                    useSubscriber;
    bool                    usePublisher;
    void                    importTf();

public:
                    tfModule();
                    ~tfModule();
    virtual bool    configure(YOS::ResourceFinder& rf );
    bool            respond( const YOS::Bottle& command, YOS::Bottle& reply );
    bool            helpCmd(YOS::Bottle& reply );
    bool            createFixedFrameCmd(const YOS::Bottle& command, YOS::Bottle& reply );
    bool            createFixedFrame( tf tf_frame );
    bool            deleteFixedFrameCmd(const YOS::Bottle& command, YOS::Bottle& reply );
    bool            listCmd(YOS::Bottle& reply );
    bool            getFrameCmd( const std::string& name, YOS::Bottle& reply );
    void            replyFrameInfo( const tf& frame, YOS::Bottle& reply );
    bool            deleteFrame( const std::string& name );
    bool            deleteFrame( const std::string& parent, const std::string& child );
    bool            rosHasFrame( std::string parent, std::string child );
    virtual bool    close();
    virtual double  getPeriod();
    virtual bool    updateModule();
    virtual bool    interruptModule();
};