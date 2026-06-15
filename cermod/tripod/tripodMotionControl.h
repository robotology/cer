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
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ReturnValue.h>

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

    inline yarp::dev::ReturnValue NOT_YET_IMPLEMENTED(const char *txt);
    inline yarp::dev::ReturnValue DEPRECATED(const char *txt);

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
    virtual yarp::dev::ReturnValue getAxisName(int axis, std::string& name) override;
    virtual yarp::dev::ReturnValue getJointType(int axis, yarp::dev::JointTypeEnum& type) override;
#if 0
    /////////   PID INTERFACE   /////////
    virtual yarp::dev::ReturnValue setPidRaw(int j, const Pid &pid) override;
    virtual yarp::dev::ReturnValue setPidsRaw(const Pid *pids) override;
    virtual yarp::dev::ReturnValue setReferenceRaw(int j, double ref) override;
    virtual yarp::dev::ReturnValue setReferencesRaw(const double *refs) override;
    virtual yarp::dev::ReturnValue setErrorLimitRaw(int j, double limit) override;
    virtual yarp::dev::ReturnValue setErrorLimitsRaw(const double *limits) override;
    virtual yarp::dev::ReturnValue getErrorRaw(int j, double *err) override;
    virtual yarp::dev::ReturnValue getErrorsRaw(double *errs) override;
    virtual yarp::dev::ReturnValue getPidRaw(int j, Pid *pid)override;
    virtual yarp::dev::ReturnValue getPidsRaw(Pid *pids) override;
    virtual yarp::dev::ReturnValue getReferenceRaw(int j, double *ref) override;
    virtual yarp::dev::ReturnValue getReferencesRaw(double *refs override;
    virtual yarp::dev::ReturnValue getErrorLimitRaw(int j, double *limit)override;
    virtual yarp::dev::ReturnValue getErrorLimitsRaw(double *limits) override;
    virtual yarp::dev::ReturnValue resetPidRaw(int j) override;
    virtual yarp::dev::ReturnValue disablePidRaw(int j) override;
    virtual yarp::dev::ReturnValue enablePidRaw(int j) override;
    virtual yarp::dev::ReturnValue setOffsetRaw(int j, double v) override;
#endif

    /////////// POSITION CONTROL INTERFACE RAW
    virtual yarp::dev::ReturnValue getAxes(int *ax) override;
    virtual yarp::dev::ReturnValue positionMoveRaw(int j, double ref)  override;
    virtual yarp::dev::ReturnValue positionMoveRaw(const double *refs)  override;
    virtual yarp::dev::ReturnValue relativeMoveRaw(int j, double delta)  override;
    virtual yarp::dev::ReturnValue relativeMoveRaw(const double *deltas)  override;
    virtual yarp::dev::ReturnValue checkMotionDoneRaw(bool& flag)  override;
    virtual yarp::dev::ReturnValue checkMotionDoneRaw(int j, bool& flag) override;
    virtual yarp::dev::ReturnValue setTrajSpeedRaw(int j, double sp) override;
    virtual yarp::dev::ReturnValue setTrajSpeedsRaw(const double *spds) override;
    virtual yarp::dev::ReturnValue setTrajAccelerationRaw(int j, double acc) override;
    virtual yarp::dev::ReturnValue setTrajAccelerationsRaw(const double *accs) override;
    virtual yarp::dev::ReturnValue getTrajSpeedRaw(int j, double *ref) override;
    virtual yarp::dev::ReturnValue getTrajSpeedsRaw(double *spds) override;
    virtual yarp::dev::ReturnValue getTrajAccelerationRaw(int j, double *acc) override;
    virtual yarp::dev::ReturnValue getTrajAccelerationsRaw(double *accs) override;
    virtual yarp::dev::ReturnValue stopRaw(int j) override;
    virtual yarp::dev::ReturnValue stopRaw() override;
    virtual yarp::dev::ReturnValue getTargetPositionRaw(const int joint, double *ref) override;
    virtual yarp::dev::ReturnValue getTargetPositionsRaw(double *refs) override;
    virtual yarp::dev::ReturnValue getTargetPositionsRaw(const int n_joint, const int *joints, double *refs) override;

    // Position Control2 Interface
    virtual yarp::dev::ReturnValue positionMoveRaw(const int n_joint, const int *joints, const double *refs) override;
    virtual yarp::dev::ReturnValue relativeMoveRaw(const int n_joint, const int *joints, const double *deltas) override;
    virtual yarp::dev::ReturnValue checkMotionDoneRaw(const std::vector<int>& joints, bool& flags) override;
    virtual yarp::dev::ReturnValue setTrajSpeedsRaw(const int n_joint, const int *joints, const double *spds) override;
    virtual yarp::dev::ReturnValue setTrajAccelerationsRaw(const int n_joint, const int *joints, const double *accs) override;
    virtual yarp::dev::ReturnValue getTrajSpeedsRaw(const int n_joint, const int *joints, double *spds) override;
    virtual yarp::dev::ReturnValue getTrajAccelerationsRaw(const int n_joint, const int *joints, double *accs) override;
    virtual yarp::dev::ReturnValue stopRaw(const int n_joint, const int *joints) override;

    //  Velocity control interface raw
    virtual yarp::dev::ReturnValue velocityMoveRaw(int j, double sp) override;
    virtual yarp::dev::ReturnValue velocityMoveRaw(const double *sp) override;

    // calibration2raw
    virtual yarp::dev::ReturnValue setCalibrationParametersRaw(int axis, const CalibrationParameters& params) override;
    virtual yarp::dev::ReturnValue calibrateAxisWithParamsRaw(int axis, unsigned int type, double p1, double p2, double p3) override;
    virtual yarp::dev::ReturnValue calibrationDoneRaw(int j) override;

    // ControlMode
    virtual yarp::dev::ReturnValue getAvailableControlModesRaw(int j, std::vector<yarp::dev::SelectableControlModeEnum>& avail) override;
    virtual yarp::dev::ReturnValue getControlModeRaw(int j, yarp::dev::ControlModeEnum& v) override;
    virtual yarp::dev::ReturnValue getControlModesRaw(std::vector<yarp::dev::ControlModeEnum>& v) override;

    // ControlMode 2
    virtual yarp::dev::ReturnValue getControlModesRaw(const std::vector<int>& joints, std::vector<yarp::dev::ControlModeEnum>& modes) override;
    virtual yarp::dev::ReturnValue setControlModeRaw(int j, yarp::dev::SelectableControlModeEnum mode) override;
    virtual yarp::dev::ReturnValue setControlModesRaw(const std::vector<int>& joints, const std::vector<yarp::dev::SelectableControlModeEnum>& modes) override;
    virtual yarp::dev::ReturnValue setControlModesRaw(const std::vector<yarp::dev::SelectableControlModeEnum>& modes) override;

    //////////////////////// BEGIN EncoderInterface
    virtual yarp::dev::ReturnValue resetEncoderRaw(int j) override;
    virtual yarp::dev::ReturnValue resetEncodersRaw() override;
    virtual yarp::dev::ReturnValue setEncoderRaw(int j, double val) override;
    virtual yarp::dev::ReturnValue setEncodersRaw(const double *vals) override;
    virtual yarp::dev::ReturnValue getEncoderRaw(int j, double *v) override;
    virtual yarp::dev::ReturnValue getEncodersRaw(double *encs) override;
    virtual yarp::dev::ReturnValue getEncoderSpeedRaw(int j, double *sp) override;
    virtual yarp::dev::ReturnValue getEncoderSpeedsRaw(double *spds) override;
    virtual yarp::dev::ReturnValue getEncoderAccelerationRaw(int j, double *spds) override;
    virtual yarp::dev::ReturnValue getEncoderAccelerationsRaw(double *accs) override;
    ///////////////////////// END Encoder Interface

    virtual yarp::dev::ReturnValue getEncodersTimedRaw(double *encs, double *stamps) override;
    virtual yarp::dev::ReturnValue getEncoderTimedRaw(int j, double *encs, double *stamp) override;


    //////////////////////// BEGIN MotorEncoderInterface
    virtual yarp::dev::ReturnValue getNumberOfMotorEncodersRaw(int * num) override;
    virtual yarp::dev::ReturnValue resetMotorEncoderRaw(int m) override;
    virtual yarp::dev::ReturnValue resetMotorEncodersRaw() override;
    virtual yarp::dev::ReturnValue setMotorEncoderRaw(int m, const double val) override;
    virtual yarp::dev::ReturnValue setMotorEncodersRaw(const double *vals) override;
    virtual yarp::dev::ReturnValue getMotorEncoderRaw(int m, double *v) override;
    virtual yarp::dev::ReturnValue getMotorEncodersRaw(double *encs) override;
    virtual yarp::dev::ReturnValue getMotorEncoderSpeedRaw(int m, double *sp) override;
    virtual yarp::dev::ReturnValue getMotorEncoderSpeedsRaw(double *spds) override;
    virtual yarp::dev::ReturnValue getMotorEncoderAccelerationRaw(int m, double *spds) override;
    virtual yarp::dev::ReturnValue getMotorEncoderAccelerationsRaw(double *accs) override;
    virtual yarp::dev::ReturnValue getMotorEncodersTimedRaw(double *encs, double *stamps) override;
    virtual yarp::dev::ReturnValue getMotorEncoderTimedRaw(int m, double *encs, double *stamp) override;
    virtual yarp::dev::ReturnValue getMotorEncoderCountsPerRevolutionRaw(int m, double *v) override;
    virtual yarp::dev::ReturnValue setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr) override;
    ///////////////////////// END MotorEncoder Interface

    //////////////////////// BEGIN RemoteVariables Interface
    virtual yarp::dev::ReturnValue getRemoteVariableRaw(std::string key, yarp::os::Bottle& val) override;
    virtual yarp::dev::ReturnValue setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val) override;
    virtual yarp::dev::ReturnValue getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys)  override;
    ///////////////////////// END RemoteVariables Interface

    ////// Amplifier interface
    virtual yarp::dev::ReturnValue getPWMLimitRaw (int axis, double* val) override;
    virtual yarp::dev::ReturnValue setPWMLimitRaw (int axis, const double val) override;
    virtual yarp::dev::ReturnValue enableAmpRaw(int j) override;
    virtual yarp::dev::ReturnValue disableAmpRaw(int j) override;
    virtual yarp::dev::ReturnValue getCurrentsRaw(double *vals) override;
    virtual yarp::dev::ReturnValue getCurrentRaw(int j, double *val) override;
    virtual yarp::dev::ReturnValue setMaxCurrentRaw(int j, double val) override;
    virtual yarp::dev::ReturnValue getMaxCurrentRaw(int j, double *val) override;
    virtual yarp::dev::ReturnValue getAmpStatusRaw(int *st) override;
    virtual yarp::dev::ReturnValue getAmpStatusRaw(int j, int *st) override;
    /////////////// END AMPLIFIER INTERFACE

    // Limits
    yarp::dev::ReturnValue setPosLimitsRaw(int axis, double min, double max) override;
    yarp::dev::ReturnValue getPosLimitsRaw(int axis, double *min, double *max) override;

    // Limits 2
    yarp::dev::ReturnValue setVelLimitsRaw(int axis, double min, double max) override;
    yarp::dev::ReturnValue getVelLimitsRaw(int axis, double *min, double *max) override;

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
    yarp::dev::ReturnValue velocityMoveRaw(const int n_joint, const int *joints, const double *spds) override;
    yarp::dev::ReturnValue getTargetVelocityRaw(const int joint, double *vel) override;
    yarp::dev::ReturnValue getTargetVelocitiesRaw(double *vels) override;
    yarp::dev::ReturnValue getTargetVelocitiesRaw(const int n_joint, const int *joints, double *vels) override;

#if 0
    bool getImpedanceRaw(int j, double *stiffness, double *damping) override;
    bool setImpedanceRaw(int j, double stiffness, double damping) override;
    bool setImpedanceOffsetRaw(int j, double offset) override;
    bool getImpedanceOffsetRaw(int j, double *offset) override;
    bool getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp) override;
#endif

    // PositionDirect Interface
    yarp::dev::ReturnValue setPositionRaw(int j, double ref) override;
    yarp::dev::ReturnValue setPositionsRaw(const int n_joint, const int *joints, const double *refs) override;
    yarp::dev::ReturnValue setPositionsRaw(const double *refs) override;
    yarp::dev::ReturnValue getRefPositionRaw(const int joint, double *ref) override;
    yarp::dev::ReturnValue getRefPositionsRaw(double *refs) override;
    yarp::dev::ReturnValue getRefPositionsRaw(const int n_joint, const int *joints, double *refs) override;

    // InteractionMode interface
    yarp::dev::ReturnValue getInteractionModeRaw(int j, yarp::dev::InteractionModeEnum& _mode) override;
    yarp::dev::ReturnValue getInteractionModesRaw(const std::vector<int>& joints, std::vector<yarp::dev::InteractionModeEnum>& modes) override;
    yarp::dev::ReturnValue getInteractionModesRaw(std::vector<yarp::dev::InteractionModeEnum>& modes) override;
    yarp::dev::ReturnValue setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum _mode) override;
    yarp::dev::ReturnValue setInteractionModesRaw(const std::vector<int>& joints, const std::vector<yarp::dev::InteractionModeEnum>& modes) override;
    yarp::dev::ReturnValue setInteractionModesRaw(const std::vector<yarp::dev::InteractionModeEnum>& modes) override;

    // IMotor interface
    yarp::dev::ReturnValue getNumberOfMotorsRaw(int * num) override;
    yarp::dev::ReturnValue getTemperatureRaw(int m, double* val) override;
    yarp::dev::ReturnValue getTemperaturesRaw(double *vals) override;
    yarp::dev::ReturnValue getTemperatureLimitRaw(int m, double *temp) override;
    yarp::dev::ReturnValue setTemperatureLimitRaw(int m, const double temp) override;
    yarp::dev::ReturnValue getGearboxRatioRaw(int m, double *val) override;
    yarp::dev::ReturnValue setGearboxRatioRaw(int m, const double val) override;

    // OPENLOOP interface
    yarp::dev::ReturnValue setRefDutyCycleRaw(int j, double v) override;
    yarp::dev::ReturnValue setRefDutyCyclesRaw(const double *v) override;
    yarp::dev::ReturnValue getRefDutyCycleRaw(int j, double *out) override;
    yarp::dev::ReturnValue getRefDutyCyclesRaw(double *outs) override;
    yarp::dev::ReturnValue getDutyCycleRaw(int j, double *out) override;
    yarp::dev::ReturnValue getDutyCyclesRaw(double *outs) override;
    yarp::dev::ReturnValue getPWMRaw(int j, double* val) override;
    yarp::dev::ReturnValue getPowerSupplyVoltageRaw(int j, double* val) override;
    yarp::dev::ReturnValue getNominalCurrentRaw(int m, double *val) override;
    yarp::dev::ReturnValue setNominalCurrentRaw(int m, const double val) override;
    yarp::dev::ReturnValue getPeakCurrentRaw(int m, double *val) override;
    yarp::dev::ReturnValue setPeakCurrentRaw(int m, const double val) override;

};

#endif // include guard
