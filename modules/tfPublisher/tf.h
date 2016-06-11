
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
#include <yarp/math/Math.h>
#include <limits>
#include <iostream>
#include <iomanip>
#include <string>

#define X_AX 0
#define Y_AX 1
#define Z_AX 2
#define W_AX 3

#define ROLL  0
#define PITCH 1
#define YAW   2
#define STRNG std::string

inline TickTime normalizeSecNSec(double yarpTimeStamp)
{
    uint64_t time;
    uint64_t nsec_part;
    uint64_t sec_part;
    TickTime ret;

    time        = (uint64_t)(yarpTimeStamp * 1000000000UL);
    nsec_part   = (time % 1000000000UL);
    sec_part    = (time / 1000000000UL);

    if (sec_part > std::numeric_limits<unsigned int>::max())
    {
        yWarning() << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }

    ret.sec     = (yarp::os::NetUint32) sec_part;
    ret.nsec    = (yarp::os::NetUint32) nsec_part;
    return ret;
}

struct tf
{ 
    enum      tf_type_enum { fixed = 0, external = 1 } type;
    double    tX;
    double    tY;
    double    tZ;
    double    rX;
    double    rY;
    double    rZ;
    double    rW;
    STRNG     name;
    STRNG     parent_frame;
    STRNG     child_frame;

    //yarp::os::BufferedPort<yarp::os::Bottle>* tfport;

                       tf();
                       tf
                       (
                            const STRNG& inName,
                            const STRNG& parent,
                            const STRNG& child,
                            double       inTX,
                            double       inTY,
                            double       inTZ,
                            double       inRX,
                            double       inRY,
                            double       inRZ,
                            double       inRW
                       );
                       ~tf();
    //void   read();
    void               transFromVec(double X, double Y, double Z);
    void               rotFromRPY(double R, double P, double Y);
    yarp::sig::Vector  getRPYRot() const;
};