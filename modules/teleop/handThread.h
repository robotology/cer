#include <string>
#include <yarp/os/RateThread.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/RpcClient.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Publisher.h>
#include "ros_messages/visualization_msgs_MarkerArray.h"
#include "yarp/os/LockGuard.h"

class HandThread : public yarp::os::RateThread
{
public:
    struct CommandData
    {
        yarp::sig::Vector pos;
        yarp::sig::Vector rpy;
        bool              button0;
        bool              button1;
    };

    enum TeleOp_hand
    {
        left_hand  = 0,
        right_hand = 1,
        hand_count = 2
    };

    HandThread(TeleOp_hand hand) : RateThread(10), arm_type(hand){}
    virtual      ~HandThread(){}
    void         setData(CommandData newdata)
    {
        mutex.lock();
        criticalSection = newdata;
        mutex.unlock();
    }

    void         setGrabTrigger();
    void         setDragTrigger();
    virtual bool threadInit()    YARP_OVERRIDE;
    virtual void threadRelease() YARP_OVERRIDE;
    virtual void run()           YARP_OVERRIDE;
    bool         openControlBoards(yarp::os::Searchable& rf);
    void         reachingHandler(const bool dragging_switch, const yarp::sig::Vector& pos, const yarp::sig::Vector& rpy);
    void         handHandler(const bool hand_grip_switch);

private:
    typedef yarp::os::BufferedPort<yarp::os::Bottle>   BottlePort;
    typedef yarp::os::BufferedPort<yarp::os::Property> PropertyPort;
    typedef yarp::os::BufferedPort<yarp::sig::Vector>  VectorPort;
    typedef yarp::sig::Vector                          Vector;
    typedef yarp::sig::VectorOf<int>                   IntVector;
    typedef yarp::os::RpcClient                        RpcClient;
    typedef yarp::dev::IVelocityControl                IVelocityControl;
    typedef yarp::dev::IControlMode2                   IControlMode2;
    typedef yarp::dev::IEncoders                       IEncoders;
    typedef yarp::dev::PolyDriver                      PolyDriver;
    typedef yarp::os::Publisher<visualization_msgs_MarkerArray>  MarkerArrayPort;

    enum TeleOp_state
    {
        idle,
        triggered,
        running
    };

    //Properties
    TeleOp_hand        arm_type;
    std::string        handFrame;
    std::string        rootFrame;
    Vector             cur_x;
    Vector             cur_o;
    Vector             pos0;
    Vector             rpy0;
    Vector             x0;
    Vector             o0;
    Vector             fixedPosition;
    Vector             fixedOrientation;
    PropertyPort       robotTargetPort;
    VectorPort         robotStatePort;
    RpcClient          robotCmdPort;
    bool               reachState;
    TeleOp_state       handDraggingStatus;
    TeleOp_state       handGripStatus;
    int                b0_pressedCount;
    int                b1_pressedCount;
    IEncoders*         ienc;
    IControlMode2*     imod;
    IVelocityControl*  ivel;
    PolyDriver         drvHand;
    IntVector          modes;
    Vector             vels;
    bool               grabTrigger;
    Vector             pos;
    Vector             rpy;
    bool               button0;
    bool               button1;
    bool               dragTrigger;
    yarp::os::Mutex    mutex;
    double             gain;
    double             wrist_heave;
    std::string        mode;
    BottlePort         gazeboPort;
    MarkerArrayPort    rosPublisherPort;
    CommandData        criticalSection;

    //method
    void   printState();
    void   updateRVIZ(const Vector &xd, const Vector &od);
    void   updateGazebo(const Vector& xd, const Vector& od);
    void   stopReaching();
    void   goToPose(const Vector &xd, const Vector &od);
    double getPeriod();
    void   getData()
    {
        mutex.lock();
        pos     = criticalSection.pos;
        rpy     = criticalSection.rpy;
        button0 = criticalSection.button0;
        button1 = criticalSection.button1;
        mutex.unlock();
    }

};
