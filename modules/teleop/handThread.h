#include <string>
#include <vector>
#include <yarp/sig/Matrix.h>
#include <yarp/os/PeriodicThread.h>
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
#include <yarp/rosmsg/visualization_msgs/MarkerArray.h>
#include <yarp/os/LockGuard.h>

class HandThread : public yarp::os::PeriodicThread
{
public:
    struct CommandData
    {
        yarp::sig::Matrix pose{4, 4};
        bool              button0{false};
        double            button1{ 0 };
        bool              button2{ false };
        double            targetDistance{0.0};
        int               controlMode{VOCAB_CM_POSITION_DIRECT};
        bool              singleButton{false};
        bool              simultMovRot{false};
        bool              absoluteRotation{false};
        yarp::sig::Matrix tf{4, 4};
        CommandData()
        {
            tf[0][0] = 1;
            tf[1][1] = 1;
            tf[2][2] = 1;
            tf[3][3] = 1;
        }
    };

    enum TeleOp_hand
    {
        left_hand  = 0,
        right_hand = 1,
        hand_count = 2
    };

    //method
    HandThread(TeleOp_hand hand) : PeriodicThread(0.01), arm_type(hand){}
    virtual      ~HandThread(){}

    yarp::sig::Matrix getMatrix()
    {
        yarp::os::LockGuard l(mutex);
        return criticalSection.tf;
    }

    void         setData(CommandData newdata)
    {
        mutex.lock();
        criticalSection.absoluteRotation = newdata.absoluteRotation;
        criticalSection.button0          = newdata.button0         ;
        criticalSection.button1          = newdata.button1         ;
        criticalSection.button2          = newdata.button2         ;
        criticalSection.controlMode      = newdata.controlMode     ;
        criticalSection.pose             = newdata.pose            ;
        criticalSection.simultMovRot     = newdata.simultMovRot    ;
        criticalSection.singleButton     = newdata.singleButton    ;
        criticalSection.targetDistance   = newdata.targetDistance  ;
        mutex.unlock();
    }
    void         setGrabTrigger();
    void         setDragTrigger();
    virtual bool threadInit()    YARP_OVERRIDE;
    virtual void threadRelease() YARP_OVERRIDE;
    virtual void run()           YARP_OVERRIDE;
    bool         openControlBoards(yarp::os::Searchable& rf);
    void         reachingHandler(const bool dragging_switch, const yarp::sig::Matrix &pose);
    void         handHandler(const bool hand_grip_switch);

    //properties
    double       targetRadius;

private:
    struct ctrlRange
    {
        double min;
        double size;
    };

    typedef yarp::os::BufferedPort<yarp::os::Bottle>             BottlePort;
    typedef yarp::os::BufferedPort<yarp::os::Property>           PropertyPort;
    typedef yarp::os::BufferedPort<yarp::sig::Vector>            VectorPort;
    typedef yarp::sig::Vector                                    Vector;
    typedef std::vector<int>                                     IntVector;
    typedef yarp::os::RpcClient                                  RpcClient;
    typedef yarp::dev::IVelocityControl                          IVelocityControl;
    typedef yarp::dev::IPositionDirect                           IPositionDirect;
    typedef yarp::dev::IControlLimits                            IControlLimits;
    typedef yarp::dev::IControlMode2                             IControlMode2;
    typedef yarp::dev::IEncoders                                 IEncoders;
    typedef yarp::dev::PolyDriver                                PolyDriver;
    typedef yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray>  MarkerArrayPort;
    typedef std::vector<ctrlRange>                               vecCtrlRanges;
    typedef yarp::sig::Matrix                                    Matrix;

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
    Vector             pos;
    Matrix             pose0;
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
    IPositionDirect*   ipos;
    IControlLimits*    ilim;
    PolyDriver         drvHand;
    IntVector          modes;
    Vector             vels;
    Matrix             pose;
    bool               button0;
    double             button1;
    bool               button2;
    int                controlMode;
    yarp::os::Mutex    mutex;
    double             gain;
    double             wrist_heave;
    std::string        mode;
    BottlePort         gazeboPort;
    MarkerArrayPort    rosPublisherPort;
    CommandData        criticalSection;
    double             targetDistance;
    vecCtrlRanges      controlRanges;
    bool               singleButton;
    bool               simultMovRot;
    bool               absoluteRotation;
    Vector             xd;

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
        pose             = criticalSection.pose;
        button0          = criticalSection.button0;
        button1          = criticalSection.button1;
        button2          = criticalSection.button2;
        targetDistance   = criticalSection.targetDistance;
        controlMode      = criticalSection.controlMode;
        singleButton     = criticalSection.singleButton;
        simultMovRot     = criticalSection.simultMovRot;
        absoluteRotation = criticalSection.absoluteRotation;
        mutex.unlock();
    }

};
