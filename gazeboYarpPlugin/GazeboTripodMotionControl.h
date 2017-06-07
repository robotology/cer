/**
 * \defgroup tripod motion device
 *
 * Copyright (C) 2015  iCub Facility, Istituto Italiano di Tecnologia
 *
 * Author: Alberto Cardellino
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This device is designed to extend the functionality of Gazebo simulator to emulate
 * the functionality of the tripod mechanism. This emulation will be done by converting
 * input references from the three linear elongations into 'user' coordinate of
 * elevation, pitch and roll by mean of a Inverse Kinematic (IK) library.
 * The result will then be sent Gazebo simulator to control the simulatd joints
 */


#ifndef GAZEBOYARP_TRIPOD_CONTROLBOARD_DRIVER_HH
#define GAZEBOYARP_TRIPOD_CONTROLBOARD_DRIVER_HH

#include <yarp/os/Property.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/IPidControl.h>
#include <string>
#include <vector>

#include <cer_kinematics/tripod.h>

#include <boost/shared_ptr.hpp>

#include <gazebo/math/Angle.hh>

namespace cer {
    namespace dev {
        class GazeboTripodMotionControl;
    }
}

namespace gazebo {
    namespace common {
        class UpdateInfo;
    }

    namespace physics {
        class Model;
        class Joint;
        typedef boost::shared_ptr<Joint> JointPtr;
    }

    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }

    namespace transport {
        class Node;
        class Publisher;

        typedef boost::shared_ptr<Node> NodePtr;
        typedef boost::shared_ptr<Publisher> PublisherPtr;
    }

    namespace msgs {
        class JointCmd;
    }
}

class cer::dev::GazeboTripodMotionControl:
    public yarp::dev::DeviceDriver,
    public yarp::dev::IPositionControl2,
    public yarp::dev::IVelocityControl2,
    public yarp::dev::IEncodersTimed,
    public yarp::dev::IControlLimits2,
    public yarp::dev::IInteractionMode,
    public yarp::dev::IControlMode2,
    public yarp::dev::ITorqueControl,
    public yarp::dev::IPositionDirect,
//     public yarp::dev::IImpedanceControl,
    public yarp::dev::IPidControl,
    public yarp::dev::IAxisInfo
{
public:

    GazeboTripodMotionControl();
    virtual ~GazeboTripodMotionControl();

    /**
     * Gazebo stuff
     */
    bool gazebo_init();
    void onUpdate(const gazebo::common::UpdateInfo&);

    /**
     * Yarp interfaces start here
     */

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();


    // AXIS IAxisInfo
    virtual bool getAxisName(int axis, yarp::os::ConstString& name);
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum& type);

    //ENCODERS
    virtual bool getEncoder(int j, double* v);
    virtual bool getEncoders(double* encs);
    virtual bool resetEncoder(int j);
    virtual bool resetEncoders();
    virtual bool setEncoder(int j, double val);
    virtual bool setEncoders(const double* vals);

    virtual bool getEncoderSpeed(int j, double* sp);
    virtual bool getEncoderSpeeds(double* spds);

    virtual bool getEncoderAcceleration(int j, double* spds);
    virtual bool getEncoderAccelerations(double* accs);

    // ENCODERS TIMED
    virtual bool getEncodersTimed(double* encs, double* time);
    virtual bool getEncoderTimed(int j, double* encs, double* time);

    //POSITION CONTROL
    virtual bool stop(int j);
    virtual bool stop();
    virtual bool positionMove(int j, double ref);
    virtual bool getAxes(int* ax); // WORKS
    virtual bool positionMove(const double* refs);
    /// @arg sp [deg/sec]
    virtual bool setRefSpeed(int j, double sp);
    virtual bool getRefSpeed(int j, double* ref);
    virtual bool getRefSpeeds(double* spds);

    virtual bool relativeMove(int j, double delta);
    virtual bool relativeMove(const double* deltas);
    virtual bool checkMotionDone(int j, bool* flag);
    virtual bool checkMotionDone(bool* flag);
    virtual bool setPositionMode();

    // Position Control 2
    virtual bool positionMove(const int n_joint, const int* joints, const double* refs);
    virtual bool relativeMove(const int n_joint, const int* joints, const double* deltas);
    virtual bool checkMotionDone(const int n_joint, const int* joints, bool* flags);
    virtual bool setRefSpeeds(const int n_joint, const int* joints, const double* spds);
    virtual bool setRefAccelerations(const int n_joint, const int* joints, const double* accs);
    virtual bool getRefSpeeds(const int n_joint, const int *joints, double *spds);
    virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs);
    virtual bool stop(const int n_joint, const int *joints);
    virtual bool getTargetPosition(const int joint, double *ref);
    virtual bool getTargetPositions(double *refs);
    virtual bool getTargetPositions(const int n_joint, const int *joints, double *refs);
    
        // IPOSITION DIRECT
    virtual bool setPositionDirectMode();
    virtual bool setPosition(int j, double ref);
    virtual bool setPositions(const int n_joint, const int *joints, double *refs);
    virtual bool setPositions(const double *refs);

    /// @arg spds [deg/sec]
    virtual bool setRefSpeeds(const double *spds);

    virtual bool setRefAcceleration(int j, double acc);
    virtual bool setRefAccelerations(const double *accs);
    virtual bool getRefAcceleration(int j, double *acc);
    virtual bool getRefAccelerations(double *accs);

    //VELOCITY CONTROL 2
    virtual bool setVelocityMode();
    virtual bool velocityMove(int j, double sp);
    virtual bool velocityMove(const double *sp);
    virtual bool velocityMove(const int n_joint, const int *joints, const double *spds);

    virtual bool setVelPid(int j, const yarp::dev::Pid &pid);
    virtual bool setVelPids(const yarp::dev::Pid *pids);
    virtual bool getVelPid(int j, yarp::dev::Pid *pid);
    virtual bool getVelPids(yarp::dev::Pid *pids);
    virtual bool getRefVelocity(const int joint, double *vel);
    virtual bool getRefVelocities(double *vels);
    virtual bool getRefVelocities(const int n_joint, const int *joints, double *vels);
    
    //CONTROL MODE
    virtual bool setPositionMode(int j);
    virtual bool setVelocityMode(int j);
    virtual bool getControlMode(int j, int *mode);

    virtual bool setTorqueMode(int j);
    virtual bool getControlModes(int *modes);

    virtual bool setImpedancePositionMode(int j);
    virtual bool setImpedanceVelocityMode(int j);
    virtual bool setOpenLoopMode(int j);

    // CONTROL MODE 2
    virtual bool getControlModes(const int n_joint, const int *joints, int *modes);
    virtual bool setControlMode(const int j, const int mode);
    virtual bool setControlModes(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModes(int *modes);

    // CONTROL LIMITS2
    virtual bool getLimits(int axis, double *min, double *max);
    virtual bool setLimits(int axis, double min, double max);
    virtual bool getVelLimits(int axis, double *min, double *max);
    virtual bool setVelLimits(int axis, double min, double max);

    /*
     * IPidControl Interface methods
     */
    virtual bool setPid (int j, const yarp::dev::Pid &pid);
    virtual bool setPids (const yarp::dev::Pid *pids);
    virtual bool setReference (int j, double ref);
    virtual bool setReferences (const double *refs);
    virtual bool setErrorLimit (int j, double limit);
    virtual bool setErrorLimits (const double *limits);
    virtual bool getError (int j, double *err);
    virtual bool getErrors (double *errs);
    virtual bool getPid (int j, yarp::dev::Pid *pid);
    virtual bool getPids (yarp::dev::Pid *pids);
    virtual bool getReference (int j, double *ref);
    virtual bool getReferences (double *refs);
    virtual bool getErrorLimit (int j, double *limit);
    virtual bool getErrorLimits (double *limits);
    virtual bool resetPid (int j);
    virtual bool disablePid (int j);
    virtual bool enablePid (int j);
    virtual bool setOffset (int j, double v);
    virtual bool getOutput(int j, double *v);
    virtual bool getOutputs(double *v);
    
    // INTERACTION MODE interface
    virtual bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode);
    virtual bool getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool getInteractionModes(yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode);
    virtual bool setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModes(yarp::dev::InteractionModeEnum* modes);

    //TORQUE CONTROL
    virtual bool getTorque(int j, double *t);
    virtual bool getTorques(double *t);
    virtual bool getRefTorque(int j, double *t);
    virtual bool getRefTorques(double *t);
    virtual bool setRefTorque(int j, double t);
    virtual bool setRefTorques(const double *t);
    virtual bool setTorqueMode();
    virtual bool getBemfParam(int j, double *bemf);
    virtual bool setBemfParam(int j, double bemf);
    virtual bool setTorquePid(int j, const yarp::dev::Pid &pid);
    virtual bool getTorqueRange(int j, double *min, double *max);
    virtual bool getTorqueRanges(double *min, double *max);
    virtual bool setTorquePids(const yarp::dev::Pid *pids);
    virtual bool setTorqueErrorLimit(int j, double limit);
    virtual bool setTorqueErrorLimits(const double *limits);
    virtual bool getTorqueError(int j, double *err);
    virtual bool getTorqueErrors(double *errs);
    virtual bool getTorquePidOutput(int j, double *out);
    virtual bool getTorquePidOutputs(double *outs);
    virtual bool getTorquePid(int j, yarp::dev::Pid *pid);
    virtual bool getTorquePids(yarp::dev::Pid *pids);
    virtual bool getTorqueErrorLimit(int j, double *limit);
    virtual bool getTorqueErrorLimits(double *limits);
    virtual bool resetTorquePid(int j);
    virtual bool disableTorquePid(int j);
    virtual bool enableTorquePid(int j);
    virtual bool setTorqueOffset(int j, double v);

    #if 0
    //IMPEDANCE CTRL
    virtual bool getImpedance(int j, double *stiffness, double *damping); // [Nm/deg] & [Nm*sec/deg]
    virtual bool setImpedance(int j, double stiffness, double damping); // [Nm/deg] & [Nm*sec/deg]
    virtual bool setImpedanceOffset(int j, double offset);
    virtual bool getImpedanceOffset(int j, double* offset);
    virtual bool getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp);

    //IOpenLoopControl interface methods
    virtual bool setRefOutput(int j, double v);
    virtual bool setRefOutputs(const double *v);
    virtual bool getRefOutput(int j, double *v);
    virtual bool getRefOutputs(double *v);
    virtual bool getOutput(int j, double *v);
    virtual bool getOutputs(double *v);
    virtual bool setOpenLoopMode();

    /*
     * Probably useless stuff here
     */
    //AMPLIFIER CONTROL (inside comanOthers.cpp)
    virtual bool enableAmp(int j);
    virtual bool disableAmp(int j);
    virtual bool getCurrent(int j, double *val);
    virtual bool getCurrents(double *vals);
    virtual bool setMaxCurrent(int j, double v);
    virtual bool getMaxCurrent(int j, double *v);
    virtual bool getAmpStatus(int *st);
    virtual bool getAmpStatus(int k, int *v);

    //CONTROL CALIBRATION (inside comanOthers.cpp)
    virtual bool calibrate2(int j, unsigned int iv, double v1, double v2, double v3);
    virtual bool done(int j); // NOT IMPLEMENTED

    /*
     * End of useless stuff
     */
#endif

private:

    /* PID structures */

    /**
     * Internal PID gains structure used in
     * GazeboYarpPlugins.
     * The gains are stored in radians based units
     * for joint whose configuration space is given
     * by an anguar quantity, and meter based units
     * for joints whose configuratios space is given
     * by a linear quantity.
     *
     * \note The gains for angular quantities in YARP
     *       are instead usually expressed in degrees-
     *       based quantity, so an appropriate conversion
     *       is used in the setPids/getPids functions.
     */
    struct PID {
        double p;
        double i;
        double d;
        double maxInt;
        double maxOut;
    };

    enum PIDFeedbackTerm {
        PIDFeedbackTermProportionalTerm = 1,
        PIDFeedbackTermIntegrativeTerm = 1 << 1,
        PIDFeedbackTermDerivativeTerm = 1 << 2,
        PIDFeedbackTermAllTerms = PIDFeedbackTermProportionalTerm | PIDFeedbackTermIntegrativeTerm | PIDFeedbackTermDerivativeTerm
    };

    enum JointType
    {
        JointType_Unknown = 0,
        JointType_Revolute,
        JointType_Prismatic
    };

    struct Range {
        Range() : min(0), max(0){}
        double min;
        double max;
    };

    bool verbose;
    std::string deviceName;
    gazebo::physics::Model* m_robot;
    gazebo::event::ConnectionPtr m_updateConnection;

    yarp::os::Property m_pluginParameters; /**< Contains the parameters of the device contained in the yarpConfigurationFile .ini file */

    unsigned int m_robotRefreshPeriod; //ms
    unsigned int m_numberOfJoints; /**< number of joints controlled by the control board */
    std::vector<Range> m_jointLimits;

    /**
     * The zero position is the position of the GAZEBO joint that will be read as the starting one
     * i.e. getEncoder(j)=m_zeroPosition+gazebo.getEncoder(j);
     */
    yarp::sig::Vector m_zeroPosition;

    yarp::sig::Vector m_positions; /**< joint positions [Degrees] */
    yarp::sig::Vector m_velocities; /**< joint velocities [Degrees/Seconds] */
    yarp::sig::Vector m_speedLimits; /**< joint velocities [Degrees/Seconds] */
    yarp::sig::Vector m_torques; /**< joint torques [Netwon Meters] */
    yarp::sig::Vector m_last_measJointPos;  /** encoder reading zeroed */
    yarp::sig::Vector m_last_motorElongat;  /** encoder reading zeroed */

    yarp::os::Stamp m_lastTimestamp; /**< timestamp, updated with simulation time at each onUpdate call */

    yarp::sig::Vector amp;
    yarp::sig::VectorOf<JointType> m_jointTypes;

    //Desired Control variables
    yarp::sig::Vector m_referenceElongations;   /**< desired reference positions as elongations. [Meters]*/
    yarp::sig::Vector m_referencePositions;     /**< desired reference positions. Depending on the position mode, they can be
                                                     set directly or indirectly through the trajectory generator. [Degrees] */
    yarp::sig::Vector m_oldReferencePositions;  // used to store last reference and check if a new ref has been commanded [Degrees]
    yarp::sig::Vector m_referenceTorques;       /**< desired reference torques for torque control mode [NetwonMeters] */
    yarp::sig::Vector m_referenceVelocities;    /**< desired reference velocities for velocity control mode [Degrees/Seconds] */
    yarp::sig::Vector m_positionThreshold;      // Threshold under which trajectory generator stops computing new values

    //trajectory generator
    yarp::sig::Vector m_trajectoryGenerationReferencePosition; /**< reference position for trajectory generation in position mode [Degrees] */
    yarp::sig::Vector m_trajectoryGenerationReferenceSpeed; /**< reference speed for trajectory generation in position mode [Degrees/Seconds]*/
    yarp::sig::Vector m_trajectoryGenerationReferenceAcceleraton; /**< reference acceleration for trajectory generation in position mode. Currently NOT USED in trajectory generation! [Degrees/Seconds^2] */

    std::vector<std::string> m_jointNames;
    std::vector<std::string> controlboard_joint_names;
    std::vector<gazebo::physics::JointPtr> m_jointPointers; /* pointers for each joint, avoiding several calls to getJoint(joint_name) */
    gazebo::transport::NodePtr m_gazeboNode;
    gazebo::transport::PublisherPtr m_jointCommandPublisher;
    std::vector<GazeboTripodMotionControl::PID> m_positionPIDs;
    std::vector<GazeboTripodMotionControl::PID> m_velocityPIDs;
    std::vector<GazeboTripodMotionControl::PID> m_impedancePosPDs;

    yarp::sig::Vector m_torqueOffsett;
    yarp::sig::Vector m_minStiffness;
    yarp::sig::Vector m_minDamping;
    yarp::sig::Vector m_maxStiffness;
    yarp::sig::Vector m_maxDamping;

    bool* m_isMotionDone;
    int * m_controlMode;
    int * m_interactionMode;

    bool started;
    int m_clock;
    int _T_controller;

    /*
     * TRIPOD Kinematic data
     */
    yarp::sig::Matrix  _baseTransformation;
    cer::kinematics::TripodSolver solver;
    bool init_kinematics();

    /**
     * Private methods
     */
    bool NOT_YET_IMPLEMENTED(const char *txt);
    bool extractGroup(yarp::os::Bottle &input, yarp::os::Bottle &out, const std::string &key1, const std::string &txt, int size);
    bool tripod_client2Sim(yarp::sig::Vector &client, yarp::sig::Vector &sim);
    bool tripod_Sim2client(yarp::sig::Vector &sim, yarp::sig::Vector &client);

    void setMinMaxPos();
    bool setJointNames();
    bool configureJointType();
    void setPIDsForGroup(std::string, std::vector<GazeboTripodMotionControl::PID>&, enum PIDFeedbackTerm pidTerms);
    void setMinMaxImpedance();
    void setPIDs();
    bool sendPositionsToGazebo(yarp::sig::Vector& refs);
    bool sendPositionToGazebo(int j,double ref);
    void prepareJointMsg(gazebo::msgs::JointCmd& j_cmd, const int joint_index, const double ref);
    bool sendVelocitiesToGazebo(yarp::sig::Vector& refs);
    bool sendVelocityToGazebo(int j,double ref);
    void prepareJointVelocityMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref);
    bool sendTorquesToGazebo(yarp::sig::Vector& refs);
    bool sendTorqueToGazebo(const int j,const double ref);
    void prepareJointTorqueMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref);
    void sendImpPositionToGazebo ( const int j, const double des );
    void sendImpPositionsToGazebo ( yarp::sig::Vector& dess );
    void computeTrajectory(const int j);
    void prepareResetJointMsg(int j);

        /**
     * \brief convert data read from Gazebo to user unit sistem,
     *  e.g. degrees for revolute joints and meters for prismatic joints
     * \param joint joint number
     * \param value Raw value read from Gazebo function like 'GetAngle'
     * \return value in user units
     */
    double convertGazeboToUser(int joint, gazebo::math::Angle value);
    double convertGazeboToUser(int joint, double value);
    double *convertGazeboToUser(double *values);
    double convertGazeboGainToUserGain(int joint, double value);

    /**
     * \brief convert data read from user unit sistem to Gazebo one
     *  e.g. radiants for revolute joints and meters for prismatic joints
     * \param joint joint number
     * \param value Raw value read from Gazebo function like 'GetAngle'
     * \return value in Gazebo units (SI)
     */
    double convertUserToGazebo(int joint, double value);
    double convertUserGainToGazeboGain(int joint, double value);

    double convertUserToGazebo(double value);
    double *convertUserToGazebo(double *values);


    bool setPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, const yarp::dev::Pid &pid);
    bool setPids(const yarp::dev::PidControlTypeEnum& pidtype, const yarp::dev::Pid *pids);
    bool setPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double ref);
    bool setPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, const double *refs);
    bool setPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double limit);
    bool setPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, const double *limits);
    bool getPidError(const yarp::dev::PidControlTypeEnum& pidtype, int j, double *err);
    bool getPidErrors(const yarp::dev::PidControlTypeEnum& pidtype, double *errs);
    bool getPidOutput(const yarp::dev::PidControlTypeEnum& pidtype, int j, double *out);
    bool getPidOutputs(const yarp::dev::PidControlTypeEnum& pidtype, double *outs);
    bool getPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, yarp::dev::Pid *pid);
    bool getPids(const yarp::dev::PidControlTypeEnum& pidtype, yarp::dev::Pid *pids);
    bool getPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double *ref);
    bool getPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, double *refs);
    bool getPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double *limit);
    bool getPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, double *limits);
    bool resetPid(const yarp::dev::PidControlTypeEnum& pidtype, int j);
    bool disablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j);
    bool enablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j);
    bool setPidOffset(const yarp::dev::PidControlTypeEnum& pidtype, int j, double v);
    bool isPidEnabled(const yarp::dev::PidControlTypeEnum& pidtype, int j, bool* enabled);

};



#endif //GAZEBOYARP_TRIPOD_CONTROLBOARD_DRIVER_HH
