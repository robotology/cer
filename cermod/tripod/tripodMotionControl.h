/* Copyright (C) 2015  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://github.com/robotology/cer/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#ifndef __tripodMotionControlh__
#define __tripodMotionControlh__

//  Yarp stuff
#include <stdint.h>
#include <mutex>
#include <vector>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>

#include <cer_kinematics/tripod.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

namespace cer {
    namespace dev  {
    class tripodMotionControl;
        namespace impl {
            class HW_deviceHelper;
        }
    }
}



using namespace yarp::dev;

class cer::dev::impl::HW_deviceHelper
{
public:
    bool _subDevVerbose;

    yarp::dev::IPidControl           *pid;
    yarp::dev::IPositionControl      *pos;
    yarp::dev::IVelocityControl      *vel;
    yarp::dev::IEncodersTimed        *iJntEnc;
    yarp::dev::IMotorEncoders        *iMotEnc;
    yarp::dev::IAmplifierControl     *amp;
    yarp::dev::IControlLimits        *lim;
    yarp::dev::IControlCalibration   *calib;
    yarp::dev::IControlCalibration   *calib2;
    yarp::dev::ITorqueControl        *iTorque;
    yarp::dev::IImpedanceControl     *iImpedance;
    yarp::dev::IPWMControl           *iPWM;
    yarp::dev::IControlMode          *iMode;
    yarp::dev::IAxisInfo             *info;
    yarp::dev::IPositionDirect       *posDir;
    yarp::dev::IInteractionMode      *iInteract;
    yarp::dev::IMotor                *imotor;
    yarp::dev::IRemoteVariables      *iVar;

    HW_deviceHelper();
    ~HW_deviceHelper();

    bool isConfigured();
    bool attach(yarp::dev::PolyDriver *d);
    void detach();

private:
    bool configured;
};


/**
 *
 * \section TripodMotionControl Description of input parameters
 * This device implements a kinematic conversion between a 'user space' in terms of heave+roll+pitch to
 * a 'hardware space' in terms of three elongations and then propagate the commands to the low-level
 * device in charge of handling the real motors.
 * Therefore each command sent to this device will be converted into commands for motors while the encoder
 * readings will be converted back from 'hardware space' (elongations) into 'user space' (heave+roll+pitch).
 *
 * Using the parameter 'HW2user', the direction of conversion can be reversed, therefore commands will be
 * converted from 'hardware space' into 'user space' while encoders will be converted from 'user space'
 * into 'hardware space'. This is meant to be used with simulators which are not able to simulate the tripod
 * mechanism.
 *
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value | Required                     | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:-------------:|:---------------------------: |:-----------------------------------------------------------------:|:-----:|
 * | -              |  jointNames    | string  | -              |   -           | Yes                          | name of each joint in sequence                    | optional, default 20ms |
 * | GENERAL        |      -         | string  | -              |   -           | Yes                          | Name of the group, mandatory      |  |
 * | -              |  Joints        | int     | -              |   -           | Yes                          | number of joints                    | for this tripod device it must be 3 |
 * | -              |  AxisMap       | int     | -              |   -           | Yes                          | vector used to remap axis indexes                    |  |
 * | -              |  Encoder       | double  | -              |   -           | Yes                          | conversion factor between input and output unit measure                     | fot this tripod device it must be 3 |
 * | -              |  Verbose       | string  | -              |   -           | No                           | enable verbose message                              | |
 * | -              |  HW2user       | bool    | -              |   -           | No                           | if set to true, the device will reverse the direction of operation, converting from hardware space into user space.                             |  |
 * | TRIPOD         |      -         | group   | -              |   -           | Yes                          | - | - |
 * | -              |  Radius        | double  | meter          |   -           | Yes                          | - | - |
 * | -              |  Min_el        | double  | meter          |   -           | Yes                          | Lower value of elongation for all motors | One value for all motors |
 * | -              |  Max_el        | double  | meter          |   -           | Yes                          | Upper value of elongation for all motors | One value for all motors |
 * | -              |  Max_alpha     | double  | degrees        |   -           | Yes                          | Max angle to which the tripod can be inclined | Single value |
 * | -              |  BASE_TRANSFORMATION   | matrix 4x4 |  -  |   -           | Yes                          | Transformation used to re-allign the base of tripod device to desired position/orientation | - |
 * | LIMITS         |  -             | group   |  -             |   -           | Yes                          | - |  - |
 * |   -            | JntVelocityMax | double  | m/s            |   -           | Yes                          | max velocity for motors| one value for all joints |
 * | CONNECTION     |      -         | group   | -              |   -           | Alternative to network group | This group is used when the device needs to connect to a remote low-level hardware controller or simulator | - |
 * | -              |  local         | string  | -              |   -           | if connection group is used  | open local port for the remote control board- | - |
 * | -              |  remote        | double  | meter          |   -           | if connection group is used  | Lower value of elongation for all motors | One value for all motors |
 * | networks       |      -         | group   | -              |   -           | Alternative to CONNECTION    | This needs to be used to directly attach to low-level hardware controller, like in the yarp robotInterface | - |
 * | -              | networkName_1  | string  | -              |   -           |   if networks is used        | Name of the device to attach to | The name has to match the one used while creating the device |
 *
 *
 * Config file example in XML format, with mandatory parameters.
 *
 * \code{.xml}
 *
 *    <group name="GENERAL">
 *     <param name="Joints">    3       </param>
 *     <param name="AxisMap">   0 1 2   </param>
 *     <param name="Encoder">   1 1 1   </param>
 *     <param name="HW2user">   false   </param>  <!-- optional -->
 *     <param name="Verbose">   true    </param>  <!-- optional -->
 *   </group>
 *
 *   <group name="TRIPOD">
 *     <param name="Radius">      0.09   </param>
 *     <param name="Max_el">      0.2   </param>
 *     <param name="Min_el">      0.0   </param>
 *     <param name="Max_alpha">   25.0   </param>
 *
 *     <param name="BASE_TRANSFORMATION">  -1.0    0.0     0.0    0.0
 *                                          0.0   -1.0     0.0    0.0
 *                                          0.0    0.0     1.0    0.0
 *                                          0.0    0.0     0.0    1.0
 *     </param>
 *   </group>
 *
 *   <group name="LIMITS">
 *       <param name="JntVelocityMax">   0.05  </param>
 *   </group>
 * \endcode
 *
 * Following parameters are meaningful ONLY for yarprobotinterface, or when the
 * low level device is available.
 *
 * \code{.xml}
 *
 *   <action phase="startup" level="5" type="attach">
 *     <paramlist name="networks">
 *         <elem name="FirstSetOfJoints"> cer_torso_mc </elem>
 *     </paramlist>
 *   </action>
 *
 *  <action phase="shutdown" level="5" type="detach" />
 * \endcode
 *
 * Following section is meant to be used when the low-level device is not directly
 * accessible or to be used with a simulator.
 *
 * \code{.xml}
 *
 *   <group name="CONNECTION">
 *     <param name="local">         /icubGazeboSim/torso_out  </param>
 *     <param name="remote">        /icubGazeboSim/torso      </param>
 *     <param name="writeStrict">   off                       </param>
 *   </group>
 * \endcode
 */

class cer::dev::tripodMotionControl:    public DeviceDriver,
                                        public IMultipleWrapper,
                                        public IAxisInfo,
                                        public IControlCalibrationRaw,
                                        public ImplementControlCalibration,
                                        public IEncodersTimedRaw,
                                        public ImplementEncodersTimed,
                                        public IMotorEncodersRaw,
                                        public ImplementMotorEncoders,
                                        public IMotorRaw,
                                        public ImplementMotor,
                                        public IPositionControlRaw,
                                        public ImplementPositionControl,
                                        public IVelocityControlRaw,
                                        public ImplementVelocityControl,
                                        public IControlModeRaw,
                                        public ImplementControlMode,
                                        public IControlLimitsRaw,
                                        public ImplementControlLimits,
                                        public IAmplifierControlRaw,
                                        public ImplementAmplifierControl,
//                                         public IImpedanceControlRaw,
//                                         public IPidControlRaw,
//                                         public ImplementPidControl,
//                                         public ImplementVelocityControl,
                                        public IPositionDirectRaw,
                                        public ImplementPositionDirect,
                                        public IInteractionModeRaw,
                                        public ImplementInteractionMode,
                                        public IRemoteVariablesRaw,
                                        public ImplementRemoteVariables,
                                        public IPWMControlRaw,
                                        public ImplementPWMControl
{
private:
    bool verbose;
    bool useRemoteCB;                 /** if TRUE it means we want to connect the tripodMotionControl to real HW device using yarp network.
                                       * This allows also to connect to a simulator
                                       * if FALSE then we wait for the 'attachAll' function to be called in order to get the pointer to the
                                       * low-level device like canBus/embObjMotionControl. */

    std::mutex                              _mutex;

    /* Set the direction of conversion: user2HW true means commands are converted from user perspective to
     * low-level HW implementation, i.e. from heave+angles into 3 elongations.
     * user2HW set to false, means the commands are intended to be 3 elongations and converted into heave+angles.
     * Feedback from encoders is converted in the opposite way.
     * By default user2HW is true, the opposite is meant to be used with Gazebo simulator.
     */
    bool                                    _directionHW2User;
    cer::dev::impl::HW_deviceHelper         _device;
    yarp::dev::PolyDriver                   *_polyDriverDevice;

    int     *_axisMap;                              /** axis remapping lookup-table */
    double  *_angleToEncoder;                    /** angle conversion factor, if any */
    double  *_encodersStamp;                    /** keep information about acquisition time for encoders read */
    
    double mRadius;

    double *_limitsMin;                         /** joint limits, max*/
    double *_limitsMax;                         /** joint limits, min*/
    double *_kinematic_mj;                      /** the kinematic coupling matrix from joints space to motor space */
    double *_currentLimits;                     /** current limits */
//     bool   *checking_motiondone;                 /* flag telling if I'm already waiting for motion done */
    bool    useRawEncoderData;

    // basic knowledge of my joints
    int   _njoints;                             // Number of joints handled by this device; this values will be extracted by the config file

    // internal stuff
    bool    *_calibrated;       // Flag to know if the calibrate function has been called for the joint
    double  *_stamps;
    double   _refSpeed;         // For the tripod device, only one velocity can be defined, it'll be used by all the joints
    double   _velLimitsMax;
    yarp::sig::Vector  _userRef_positions;     // used for position control.
    yarp::sig::Vector  _robotRef_positions;    // used for position control.
    yarp::sig::Vector  _lastUser_encoders;     // used for position control.
    yarp::sig::Vector  _lastRobot_encoders;    // used for position control.
    yarp::sig::Vector  _robotRef_speeds;       // used for positionMove.
    yarp::sig::Vector  _posDeltas;             // used to compute _robotRef_speeds on the fly.
    std::vector<std::string> _jointNames;     // holds joint names

    yarp::sig::Matrix  _baseTransformation;

    // Kinematics stuff
    cer::kinematics::TripodSolver solver;

private:

    inline bool NOT_YET_IMPLEMENTED(const char *txt);
    inline bool DEPRECATED(const char *txt);

    bool extractGroup(Searchable &input, Bottle &out, const std::string &key1, const std::string &txt, int size);
    bool parsePositionPidsGroup(Bottle& pidsGroup, Pid myPid[]);
    bool parseTorquePidsGroup(Bottle& pidsGroup, Pid myPid[], double kbemf[], double ktau[], int filterType[]);

    bool alloc(int njoints);
    bool dealloc();

    bool fromConfig(yarp::os::Searchable &config);
    bool init(void);

    void copyPid_iCub2eo(const Pid *in, Pid *out);
    void copyPid_eo2iCub(Pid *in, Pid *out);

    bool initKinematics();

    bool tripod_user2HW(yarp::sig::Vector &user,  yarp::sig::Vector &robot);
    bool tripod_HW2user(yarp::sig::Vector &robot, yarp::sig::Vector &user);
    bool compute_speeds(yarp::sig::Vector &reference,  yarp::sig::Vector &encoders);

    bool refreshPositionTargets(const int controlMode);

public:

    tripodMotionControl();
    ~tripodMotionControl();

    // Device Driver
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();

    // IMultipleWrapper interface
    virtual bool attachAll(const PolyDriverList &p);
    virtual bool detachAll();

    bool refreshEncoders(double *times);
    std::string deviceDescription;

    /////////   Axis info INTERFACE   /////////
    virtual bool getAxisName(int axis, std::string& name) override;
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum& type) override;
#if 0
    /////////   PID INTERFACE   /////////
    virtual bool setPidRaw(int j, const Pid &pid) override;
    virtual bool setPidsRaw(const Pid *pids) override;
    virtual bool setReferenceRaw(int j, double ref) override;
    virtual bool setReferencesRaw(const double *refs) override;
    virtual bool setErrorLimitRaw(int j, double limit) override;
    virtual bool setErrorLimitsRaw(const double *limits) override;
    virtual bool getErrorRaw(int j, double *err) override;
    virtual bool getErrorsRaw(double *errs) override;
    virtual bool getPidRaw(int j, Pid *pid)override;
    virtual bool getPidsRaw(Pid *pids) override;
    virtual bool getReferenceRaw(int j, double *ref) override;
    virtual bool getReferencesRaw(double *refs override;
    virtual bool getErrorLimitRaw(int j, double *limit)override;
    virtual bool getErrorLimitsRaw(double *limits) override;
    virtual bool resetPidRaw(int j) override;
    virtual bool disablePidRaw(int j) override;
    virtual bool enablePidRaw(int j) override;
    virtual bool setOffsetRaw(int j, double v) override;
#endif

    /////////// POSITION CONTROL INTERFACE RAW
    virtual bool getAxes(int *ax) override;
    virtual bool positionMoveRaw(int j, double ref)  override;
    virtual bool positionMoveRaw(const double *refs)  override;
    virtual bool relativeMoveRaw(int j, double delta)  override;
    virtual bool relativeMoveRaw(const double *deltas)  override;
    virtual bool checkMotionDoneRaw(bool *flag)  override;
    virtual bool checkMotionDoneRaw(int j, bool *flag) override;
    virtual bool setRefSpeedRaw(int j, double sp) override;
    virtual bool setRefSpeedsRaw(const double *spds) override;
    virtual bool setRefAccelerationRaw(int j, double acc) override;
    virtual bool setRefAccelerationsRaw(const double *accs) override;
    virtual bool getRefSpeedRaw(int j, double *ref) override;
    virtual bool getRefSpeedsRaw(double *spds) override;
    virtual bool getRefAccelerationRaw(int j, double *acc) override;
    virtual bool getRefAccelerationsRaw(double *accs) override;
    virtual bool stopRaw(int j) override;
    virtual bool stopRaw() override;
    virtual bool getTargetPositionRaw(const int joint, double *ref) override;
    virtual bool getTargetPositionsRaw(double *refs) override;
    virtual bool getTargetPositionsRaw(const int n_joint, const int *joints, double *refs) override;

    // Position Control2 Interface
    virtual bool positionMoveRaw(const int n_joint, const int *joints, const double *refs) override;
    virtual bool relativeMoveRaw(const int n_joint, const int *joints, const double *deltas) override;
    virtual bool checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags) override;
    virtual bool setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds) override;
    virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs) override;
    virtual bool getRefSpeedsRaw(const int n_joint, const int *joints, double *spds) override;
    virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs) override;
    virtual bool stopRaw(const int n_joint, const int *joints) override;

    //  Velocity control interface raw
    virtual bool velocityMoveRaw(int j, double sp) override;
    virtual bool velocityMoveRaw(const double *sp) override;

    // calibration2raw
    virtual bool setCalibrationParametersRaw(int axis, const CalibrationParameters& params) override;
    virtual bool calibrateAxisWithParamsRaw(int axis, unsigned int type, double p1, double p2, double p3) override;
    virtual bool calibrationDoneRaw(int j) override;

    // ControlMode
    virtual bool getControlModeRaw(int j, int *v) override;
    virtual bool getControlModesRaw(int *v) override;

    // ControlMode 2
    virtual bool getControlModesRaw(const int n_joint, const int *joints, int *modes) override;
    virtual bool setControlModeRaw(const int j, const int mode) override;
    virtual bool setControlModesRaw(const int n_joint, const int *joints, int *modes) override;
    virtual bool setControlModesRaw(int *modes) override;

    //////////////////////// BEGIN EncoderInterface
    virtual bool resetEncoderRaw(int j) override;
    virtual bool resetEncodersRaw() override;
    virtual bool setEncoderRaw(int j, double val) override;
    virtual bool setEncodersRaw(const double *vals) override;
    virtual bool getEncoderRaw(int j, double *v) override;
    virtual bool getEncodersRaw(double *encs) override;
    virtual bool getEncoderSpeedRaw(int j, double *sp) override;
    virtual bool getEncoderSpeedsRaw(double *spds) override;
    virtual bool getEncoderAccelerationRaw(int j, double *spds) override;
    virtual bool getEncoderAccelerationsRaw(double *accs) override;
    ///////////////////////// END Encoder Interface

    virtual bool getEncodersTimedRaw(double *encs, double *stamps) override;
    virtual bool getEncoderTimedRaw(int j, double *encs, double *stamp) override;


    //////////////////////// BEGIN MotorEncoderInterface
    virtual bool getNumberOfMotorEncodersRaw(int * num) override;
    virtual bool resetMotorEncoderRaw(int m) override;
    virtual bool resetMotorEncodersRaw() override;
    virtual bool setMotorEncoderRaw(int m, const double val) override;
    virtual bool setMotorEncodersRaw(const double *vals) override;
    virtual bool getMotorEncoderRaw(int m, double *v) override;
    virtual bool getMotorEncodersRaw(double *encs) override;
    virtual bool getMotorEncoderSpeedRaw(int m, double *sp) override;
    virtual bool getMotorEncoderSpeedsRaw(double *spds) override;
    virtual bool getMotorEncoderAccelerationRaw(int m, double *spds) override;
    virtual bool getMotorEncoderAccelerationsRaw(double *accs) override;
    virtual bool getMotorEncodersTimedRaw(double *encs, double *stamps) override;
    virtual bool getMotorEncoderTimedRaw(int m, double *encs, double *stamp) override;
    virtual bool getMotorEncoderCountsPerRevolutionRaw(int m, double *v) override;
    virtual bool setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr) override;
    ///////////////////////// END MotorEncoder Interface

    //////////////////////// BEGIN RemoteVariables Interface
    virtual bool getRemoteVariableRaw(std::string key, yarp::os::Bottle& val) override;
    virtual bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val) override;
    virtual bool getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys)  override;
    ///////////////////////// END RemoteVariables Interface

    ////// Amplifier interface
    virtual bool getPWMLimitRaw (int axis, double* val) override;
    virtual bool setPWMLimitRaw (int axis, const double val) override;
    virtual bool enableAmpRaw(int j) override;
    virtual bool disableAmpRaw(int j) override;
    virtual bool getCurrentsRaw(double *vals) override;
    virtual bool getCurrentRaw(int j, double *val) override;
    virtual bool setMaxCurrentRaw(int j, double val) override;
    virtual bool getMaxCurrentRaw(int j, double *val) override;
    virtual bool getAmpStatusRaw(int *st) override;
    virtual bool getAmpStatusRaw(int j, int *st) override;
    /////////////// END AMPLIFIER INTERFACE

    // Limits
    bool setLimitsRaw(int axis, double min, double max) override;
    bool getLimitsRaw(int axis, double *min, double *max) override;

    // Limits 2
    bool setVelLimitsRaw(int axis, double min, double max) override;
    bool getVelLimitsRaw(int axis, double *min, double *max) override;

    // Torque control
#if 0
    bool getTorqueRaw(int j, double *t) override;
    bool getTorquesRaw(double *t) override;
    bool getBemfParamRaw(int j, double *bemf) override;
    bool setBemfParamRaw(int j, double bemf) override;
    bool getTorqueRangeRaw(int j, double *min, double *max)override;
    bool getTorqueRangesRaw(double *min, double *max) override;
    bool setRefTorquesRaw(const double *t)override;
    bool setRefTorqueRaw(int j, double t)override;
    bool setRefTorquesRaw(const int n_joint, const int *joints, const double *t)override;
    bool getRefTorquesRaw(double *t)override;
    bool getRefTorqueRaw(int j, double *t)override;
    bool setTorquePidRaw(int j, const Pid &pid)override;
    bool setTorquePidsRaw(const Pid *pids)override;
    bool setTorqueErrorLimitRaw(int j, double limit)override;
    bool setTorqueErrorLimitsRaw(const double *limits)override;
    bool getTorqueErrorRaw(int j, double *err)override;
    bool getTorqueErrorsRaw(double *errs)override;
    bool getTorquePidOutputRaw(int j, double *out)override;
    bool getTorquePidOutputsRaw(double *outs)override;
    bool getTorquePidRaw(int j, Pid *pid)override;
    bool getTorquePidsRaw(Pid *pids)override;
    bool getTorqueErrorLimitRaw(int j, double *limit)override;
    bool getTorqueErrorLimitsRaw(double *limits)override;
    bool resetTorquePidRaw(int j)override;
    bool disableTorquePidRaw(int j)override;
    bool enableTorquePidRaw(int j)override;
    bool setTorqueOffsetRaw(int j, double v)override;
    bool getMotorTorqueParamsRaw(int j, MotorTorqueParameters *params)override;
    bool setMotorTorqueParamsRaw(int j, const MotorTorqueParameters params)override;
#endif 

    // IVelocityControl2
    bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds) override;

#if 0
    bool getImpedanceRaw(int j, double *stiffness, double *damping) override;
    bool setImpedanceRaw(int j, double stiffness, double damping) override;
    bool setImpedanceOffsetRaw(int j, double offset) override;
    bool getImpedanceOffsetRaw(int j, double *offset) override;
    bool getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp) override;
#endif 

    // PositionDirect Interface
    bool setPositionRaw(int j, double ref) override;
    bool setPositionsRaw(const int n_joint, const int *joints, const double *refs) override;
    bool setPositionsRaw(const double *refs) override;

    // InteractionMode interface
    bool getInteractionModeRaw(int j, yarp::dev::InteractionModeEnum* _mode) override;
    bool getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    bool getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes) override;
    bool setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum _mode) override;
    bool setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    bool setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes) override;

    // IMotor interface
    bool getNumberOfMotorsRaw(int * num) override;
    bool getTemperatureRaw(int m, double* val) override;
    bool getTemperaturesRaw(double *vals) override;
    bool getTemperatureLimitRaw(int m, double *temp) override;
    bool setTemperatureLimitRaw(int m, const double temp) override;
    
    // OPENLOOP interface
    bool setRefDutyCycleRaw(int j, double v) override;
    bool setRefDutyCyclesRaw(const double *v) override;
    bool getRefDutyCycleRaw(int j, double *out) override;
    bool getRefDutyCyclesRaw(double *outs) override;
    bool getDutyCycleRaw(int j, double *out) override;
    bool getDutyCyclesRaw(double *outs) override;

};

#endif // include guard

