/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "GazeboTripodMotionControl.h"
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/Handler.hh>

#include <cstdio>
#include <cmath>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/math/Angle.hh>

#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace cer::dev;


const double RobotPositionTolerance_revolute = 0.9;      // Degrees
const double RobotPositionTolerance_linear   = 0.004;    // Meters

GazeboTripodMotionControl::GazeboTripodMotionControl() : deviceName(""), verbose(true)
{
    _baseTransformation.resize(4,4);   // rototraslation matrix, size is fixed
    _baseTransformation.zero();
}

GazeboTripodMotionControl::~GazeboTripodMotionControl() {}

bool GazeboTripodMotionControl::NOT_YET_IMPLEMENTED(const char *txt)
{
    if(verbose)
        yError() << txt << " is not yet implemented for GazeboTripodMotionControl";
    return false;
}

bool GazeboTripodMotionControl::tripod_client2Sim(yarp::sig::Vector &client, yarp::sig::Vector &sim)
{
    // The caller must use mutex or private data
    Matrix H;
    if (solver.fkin(client,H))
    {
        Matrix H_=SE3inv(_baseTransformation)*H;
        Vector ypr=dcm2ypr(H);

        sim.resize(3);
        sim[0]=H_(2,3);                // heave
        sim[1]=(180.0/M_PI)*ypr[1];    // pitch
        sim[2]=(180.0/M_PI)*ypr[2];    // roll
        return true;
    }
    else
        return false;
}

bool GazeboTripodMotionControl::tripod_Sim2client(yarp::sig::Vector &sim, yarp::sig::Vector &client)
{
    // The caller must use mutex or private data
    client = m_referenceElongations;
    return true;
}

bool GazeboTripodMotionControl::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    size++;
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        yError () << key1.c_str() << " parameter not found";
        return false;
    }

    if(tmp.size()!=size)
    {
        yError () << key1.c_str() << " incorrect number of entries for param" << key1.c_str() << " expected size is " << size-1 << " while size of param in config file was " << tmp.size() -1;
        return false;
    }

    out=tmp;
    return true;
}

bool GazeboTripodMotionControl::open(yarp::os::Searchable& config)
{
    m_pluginParameters.fromString(config.toString().c_str());

    deviceName = m_pluginParameters.find("name").asString().c_str();
    std::cout << "Opening  GazeboYarpControlBoardDriver named " << deviceName << std::endl;

    std::string robotName(m_pluginParameters.find("robotScopedName").asString().c_str());
    std::cout << "DeviceDriver is looking for robot " << robotName << "..." << std::endl;

    m_robot = GazeboYarpPlugins::Handler::getHandler()->getRobot(robotName);
    if(!m_robot) {
        std::cout << "GazeboYarpControlBoardDriver error: robot was not found" << std::endl;
        return false;
    }

    // read param from config file, if any
    if(!gazebo_init())
    {
        yError() << "Problem initting Gazebo parameters";
        return false;
    }
    if(!init_kinematics())
    {
        yError() << "Problem initting Tripod kinematics";
        return false;
    }
    return true;
}

bool GazeboTripodMotionControl::init_kinematics()
{
    /////// Tripod solver configuration
    Bottle &tripod_description = m_pluginParameters.findGroup("TRIPOD");
    Bottle xtmp;
    if (tripod_description.isNull())
    {
        yError() << " ***TripodMotionControl detects that Group TRIPOD is not found in configuration file***";
        return false;
    }

    // radius
    double radius, lMax, lMin, alpha;
    if (!extractGroup(tripod_description, xtmp, "Radius","The radius ([m])", 1))
        return false;
    else
        radius = xtmp.get(1).asDouble();

    // max limit
        if (!extractGroup(tripod_description, xtmp, "Max_el","The minimum elongation ([m]).", 1))
        return false;
    else
        lMax = xtmp.get(1).asDouble();

    // min limit
        if (!extractGroup(tripod_description, xtmp, "Min_el","The maximum permitted bending angle ([deg]).", 1))
        return false;
    else
        lMin = xtmp.get(1).asDouble();

    // alpha max
        if (!extractGroup(tripod_description, xtmp, "Max_alpha","The maximum permitted bending angle ([deg])", 1))
        return false;
    else
        alpha = xtmp.get(1).asDouble();

    // This particular device has weird limits for joints. This is not scalable unless we duplicate some parameter in config file.
    if(m_numberOfJoints == 3)
    {
        m_jointLimits[0].max = lMax;
        m_jointLimits[0].min = lMin;
        m_jointLimits[1].max = lMax;
        m_jointLimits[1].min = lMin;
        m_jointLimits[2].max = lMax;
        m_jointLimits[2].min = lMin;
    }
    else
    {
        yError() << "Number of joints for this device is supposed to be 3!";
        return false;
    }

        // Read the base transformation Matrix
    if(!extractGroup(tripod_description, xtmp, "BASE_TRANSFORMATION", "the rototraslation matrix 4x4 which tranforms system references from root frame to the tripod base", 16))
        return false;
    else
    {
        int idx=1;
        for(int r=0; r<4; r++)
            for(int c=0; c<4; c++)
            {
                _baseTransformation[r][c] = xtmp.get(idx).asDouble();
                idx++;
            }
    }
    cer::kinematics::TripodParameters tParam(radius, lMin, lMax, alpha, _baseTransformation);
    solver.setParameters(tParam);
    return true;
}

bool GazeboTripodMotionControl::close()
{
    std::cout << "Closing device " << deviceName  << std::endl;
    //unbinding events
    if (this->m_updateConnection.get()) {
        gazebo::event::Events::DisconnectWorldUpdateBegin (this->m_updateConnection);
        this->m_updateConnection = gazebo::event::ConnectionPtr();
    }

    delete [] m_controlMode;
    delete [] m_interactionMode;
    delete [] m_isMotionDone;
    return true;
}


bool GazeboTripodMotionControl::gazebo_init()
{
    //m_robot = gazebo_pointer_wrapper::getModel();
    // std::cout<<"if this message is the last one you read, m_robot has not been set"<<std::endl;
    //assert is a NOP in release mode. We should change the error handling either with an exception or something else
    assert(m_robot);
    if (!m_robot) return false;

    this->m_robotRefreshPeriod = (unsigned)(this->m_robot->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod() * 1000.0);
    if (!setJointNames()) return false;

    m_numberOfJoints = m_jointNames.size();
    std::cout << "\nnumber of joints is " << m_numberOfJoints << std::endl;
    m_positions.resize(m_numberOfJoints);
    m_zeroPosition.resize(m_numberOfJoints);
    m_last_measJointPos.resize(m_numberOfJoints);
    m_last_motorElongat.resize(m_numberOfJoints);
    m_referenceVelocities.resize(m_numberOfJoints);
    m_velocities.resize(m_numberOfJoints);
    amp.resize(m_numberOfJoints);
    m_torques.resize(m_numberOfJoints); m_torques.zero();
    m_trajectoryGenerationReferenceSpeed.resize(m_numberOfJoints);
    m_referencePositions.resize(m_numberOfJoints);
    m_oldReferencePositions.resize(m_numberOfJoints);
    m_referenceElongations.resize(m_numberOfJoints);
    m_trajectoryGenerationReferencePosition.resize(m_numberOfJoints);
    m_trajectoryGenerationReferenceAcceleraton.resize(m_numberOfJoints);
    m_referenceTorques.resize(m_numberOfJoints);
    m_jointLimits.resize(m_numberOfJoints);
    m_positionPIDs.reserve(m_numberOfJoints);
    m_velocityPIDs.reserve(m_numberOfJoints);
    m_impedancePosPDs.reserve(m_numberOfJoints);
    m_torqueOffsett.resize(m_numberOfJoints);
    m_minStiffness.resize(m_numberOfJoints, 0.0);
    m_maxStiffness.resize(m_numberOfJoints, 1000.0);
    m_minDamping.resize(m_numberOfJoints, 0.0);
    m_maxDamping.resize(m_numberOfJoints, 100.0);
    m_positionThreshold.resize(m_numberOfJoints);
    m_jointTypes.resize(m_numberOfJoints);
    m_speedLimits.resize(m_numberOfJoints);

    // Initial zeroing of all vectors
    m_positions.zero();
    m_zeroPosition.zero();
    m_referenceVelocities.zero();
    m_last_measJointPos.zero();
    m_last_motorElongat.zero();
    m_velocities.zero();
    m_trajectoryGenerationReferenceSpeed.zero();
    m_referencePositions.zero();
    m_referenceElongations.zero();
    m_trajectoryGenerationReferencePosition.zero();
    m_trajectoryGenerationReferenceAcceleraton.zero();
    m_referenceTorques.zero();
    m_speedLimits.zero();
    amp = 1; // initially on - ok for simulator
    started = false;
    m_controlMode = new int[m_numberOfJoints];
    m_interactionMode = new int[m_numberOfJoints];
    m_isMotionDone = new bool[m_numberOfJoints];
    m_clock = 0;
    m_torqueOffsett = 0;

    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
    {
        m_controlMode[j] = VOCAB_CM_POSITION_DIRECT;
        m_interactionMode[j] = VOCAB_IM_STIFF;
        m_jointTypes[j] = JointType_Prismatic;
        m_isMotionDone[j] = true;
        // Set an old reference value surely out of range. This will force the setting of initial reference at startup
        m_oldReferencePositions[j] = m_jointLimits[j].max *2;
    }
    // End zeroing of vectors

    // This must be after zeroing of vectors
    configureJointType();

    setMinMaxPos();
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
    {
        // Set an old reference value surely out of range. This will force the setting of initial reference at startup
        // NOTE: This has to be after setMinMaxPos function
        m_oldReferencePositions[j] = m_jointLimits[j].max *2;
    }

    setMinMaxImpedance();
    setPIDs();

    this->m_updateConnection =
    gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboTripodMotionControl::onUpdate, this, _1));

    m_gazeboNode = gazebo::transport::NodePtr(new gazebo::transport::Node);
    m_gazeboNode->Init(this->m_robot->GetWorld()->GetName());
    m_jointCommandPublisher = m_gazeboNode->Advertise<gazebo::msgs::JointCmd>(std::string("~/") + this->m_robot->GetName() + "/joint_cmd");

    _T_controller = 1;

    if(m_pluginParameters.check("initialConfiguration") )
    {
        std::stringstream ss(m_pluginParameters.find("initialConfiguration").toString());
        double tmp = 0.0;
        yarp::sig::Vector initial_config(m_numberOfJoints);
        unsigned int counter = 1;
        while (ss >> tmp) {
            if(counter > m_numberOfJoints) {
                std::cout<<"To many element in initial configuration, stopping at element "<<counter<<std::endl;
                break;
            }
            initial_config[counter-1] = tmp;
            m_trajectoryGenerationReferencePosition[counter - 1] = convertGazeboToUser(counter-1, tmp);
            m_referencePositions[counter - 1] = convertGazeboToUser(counter-1, tmp);
            m_positions[counter - 1] = convertGazeboToUser(counter-1, tmp);
            counter++;
        }
        std::cout<<"INITIAL CONFIGURATION IS: "<<initial_config.toString()<<std::endl;


        // Set initial reference
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
#if GAZEBO_MAJOR_VERSION >= 4
            m_jointPointers[i]->SetPosition(0,initial_config[i]);
#else
            gazebo::math::Angle a;
            a.SetFromRadian(initial_config[i]);
            m_jointPointers[i]->SetAngle(0,a);
#endif
        }
    }

    // initialize motor encoders
    tripod_Sim2client(m_positions, m_last_motorElongat);

    Bottle &limitsGroup = m_pluginParameters.findGroup("LIMITS");
    if(limitsGroup.isNull())
    {
        yError() << "Cannot find 'LIMITS' group in config file";
        return false;
    }

    double speed = 0;
    if(! (speed = limitsGroup.check("jntVelMax")))
    {
        yError() << "Cannot find 'jntVelMax' parameter in config file";
        return false;
    }
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
        m_speedLimits[j] = speed;

    return true;
}

bool GazeboTripodMotionControl::configureJointType()
{
    bool ret = true;
    //////// determine the type of joint (rotational or prismatic)
    for(int i=0; i< m_numberOfJoints; i++)
    {
        switch( m_jointPointers[i]->GetType())
        {
            case ( gazebo::physics::Entity::HINGE_JOINT  |  gazebo::physics::Entity::JOINT):
            {
                m_jointTypes[i] = JointType_Revolute;
                m_positionThreshold[i] = RobotPositionTolerance_revolute;
                break;
            }

            case ( gazebo::physics::Entity::SLIDER_JOINT |  gazebo::physics::Entity::JOINT):
            {
                m_jointTypes[i] = JointType_Prismatic;
                m_positionThreshold[i] = RobotPositionTolerance_linear;
                break;
            }

            default:
            {
                yError() << "Error, joint type is not supported by Gazebo YARP plugin now. Supported joint types are 'revolute' and 'prismatic' \n\t(GEARBOX_JOINT and SLIDER_JOINT using Gazebo enums defined into gazebo/physic/base.hh include file, GetType() returns " << m_jointPointers[i]->GetType() ;
                m_jointTypes[i] = JointType_Unknown;
                ret = false;
                break;
            }
        }
    }
    return ret;
}

void GazeboTripodMotionControl::computeTrajectory(const int j)
{
    double step = (m_trajectoryGenerationReferenceSpeed[j] / 1000.0) * m_robotRefreshPeriod * _T_controller;
    double error_abs = fabs(m_referencePositions[j] - m_trajectoryGenerationReferencePosition[j]);

    // if delta is bigger then threshold, in some cases this will never converge to an end.
    // Check to prevent those cases
    if(error_abs)
    {
        // Watch out for problem
        if((error_abs < m_positionThreshold[j]) || ( error_abs < step) )    // This id both 'normal ending condition' and safe procedure when step > threshold causing infinite oscillation around final position
        {
            // Just go to final position
            m_referencePositions[j] = m_trajectoryGenerationReferencePosition[j];
            m_isMotionDone[j] = true;
            return;
        }

        if (m_trajectoryGenerationReferencePosition[j] > m_referencePositions[j])
        {
            m_referencePositions[j] += step;
            m_isMotionDone[j] = false;
        }
        else
        {
            m_referencePositions[j] -= step;
            m_isMotionDone[j] = false;
        }
    }
}

void GazeboTripodMotionControl::onUpdate(const gazebo::common::UpdateInfo& _info)
{
    m_clock++;

    // Sensing position & torque
    for (unsigned int jnt_cnt = 0; jnt_cnt < m_jointPointers.size(); jnt_cnt++) {
    //TODO: consider multi-dof joint ?
        m_positions[jnt_cnt] = convertGazeboToUser(jnt_cnt, m_jointPointers[jnt_cnt]->GetAngle(0));
        m_velocities[jnt_cnt] = convertGazeboToUser(jnt_cnt, m_jointPointers[jnt_cnt]->GetVelocity(0));
        m_torques[jnt_cnt] = m_jointPointers[jnt_cnt]->GetForce(0u);
    }

    // Updating timestamp
    m_lastTimestamp.update(_info.simTime.Double());

    //logger.log(m_velocities[2]);
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
    {
        m_last_measJointPos[j] = m_positions[j] - m_zeroPosition[j];

        //set pos joint value, set m_referenceVelocities joint value
        if ((m_controlMode[j] == VOCAB_CM_POSITION || m_controlMode[j] == VOCAB_CM_POSITION_DIRECT)
            && (m_interactionMode[j] == VOCAB_IM_STIFF)) {
            if (m_clock % _T_controller == 0) {
                if (m_controlMode[j] == VOCAB_CM_POSITION) {
                    computeTrajectory(j);
                }
                sendPositionToGazebo(j, m_referencePositions[j]);
//                 yInfo() << "OnUpdate, m_referencePositions are " << m_referencePositions.toString();
            }
        } else if ((m_controlMode[j] == VOCAB_CM_VELOCITY) && (m_interactionMode[j] == VOCAB_IM_STIFF)) {//set vmo joint value
            if (m_clock % _T_controller == 0) {
                sendVelocityToGazebo(j, m_referenceVelocities[j]);
            }
        } else if (m_controlMode[j] == VOCAB_CM_TORQUE) {
            if (m_clock % _T_controller == 0) {
                sendTorqueToGazebo(j, m_referenceTorques[j]);
            }
        } else if (m_controlMode[j] == VOCAB_CM_PWM) {
            //PWM control sends torques to gazebo at this moment.
            //Check if gazebo implements a "motor" entity and change the code accordingly.
            if (m_clock % _T_controller == 0) {
                sendTorqueToGazebo(j, m_referenceTorques[j]);
            }
        } else if ((m_controlMode[j] == VOCAB_CM_POSITION || m_controlMode[j] == VOCAB_CM_POSITION_DIRECT)
            && (m_interactionMode[j] == VOCAB_IM_COMPLIANT)) {
            if (m_clock % _T_controller == 0) {
                if (m_controlMode[j] == VOCAB_CM_POSITION) {
                    computeTrajectory(j);
                }
                sendImpPositionToGazebo(j, m_referencePositions[j]);
            }
        }
    }
    tripod_Sim2client(m_last_measJointPos, m_last_motorElongat);
}

void GazeboTripodMotionControl::setMinMaxPos()
{
    for(unsigned int i = 0; i < m_numberOfJoints; ++i)
    {
        m_jointLimits[i].max = convertGazeboToUser(i, m_jointPointers[i]->GetUpperLimit(0));
        m_jointLimits[i].min = convertGazeboToUser(i, m_jointPointers[i]->GetLowerLimit(0));
   }
}

bool GazeboTripodMotionControl::setJointNames()  //WORKS
{
    yarp::os::Bottle joint_names_bottle = m_pluginParameters.findGroup("jointNames");

    if (joint_names_bottle.isNull()) {
        std::cout << "GazeboTripodMotionControl::setJointNames(): Error cannot find jointNames." << std::endl;
        return false;
    }

    int nr_of_joints = joint_names_bottle.size()-1;

    m_jointNames.resize(nr_of_joints);
    m_jointPointers.resize(nr_of_joints);

    const gazebo::physics::Joint_V & gazebo_models_joints = m_robot->GetJoints();

    controlboard_joint_names.clear();
    for (unsigned int i = 0; i < m_jointNames.size(); i++) {
        bool joint_found = false;
        controlboard_joint_names.push_back(joint_names_bottle.get(i+1).asString().c_str());

        for (unsigned int gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size() && !joint_found; gazebo_joint++) {
            std::string gazebo_joint_name = gazebo_models_joints[gazebo_joint]->GetName();
            if (GazeboYarpPlugins::hasEnding(gazebo_joint_name,controlboard_joint_names[i])) {
                joint_found = true;
                m_jointNames[i] = gazebo_joint_name;
                m_jointPointers[i] = this->m_robot->GetJoint(gazebo_joint_name);
            }
        }

        if (!joint_found) {
            yError() << "GazeboTripodMotionControl::setJointNames(): cannot find joint " << m_jointNames[i]
                     << " ( " << i << " of " << nr_of_joints << " ) " << "\n";
            yError() << "jointNames are " << joint_names_bottle.toString() << "\n";
            m_jointNames.resize(0);
            m_jointPointers.resize(0);
            return false;
        }
    }
    return true;
}

void GazeboTripodMotionControl::setPIDsForGroup(std::string pidGroupName,
                                                   std::vector<GazeboTripodMotionControl::PID>& pids,
                                                   enum PIDFeedbackTerm pidTerms)
{
    yarp::os::Property prop;
    if (m_pluginParameters.check(pidGroupName.c_str())) {
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
            std::stringstream property_name;
            property_name<<"Pid";
            property_name<<i;

            yarp::os::Bottle& pid = m_pluginParameters.findGroup(pidGroupName.c_str()).findGroup(property_name.str().c_str());

            GazeboTripodMotionControl::PID pidValue = {0, 0, 0, -1, -1};
            if (pidTerms & PIDFeedbackTermProportionalTerm)
                pidValue.p = pid.get(1).asDouble();
            if (pidTerms & PIDFeedbackTermDerivativeTerm)
                pidValue.d = pid.get(2).asDouble();
            if (pidTerms & PIDFeedbackTermIntegrativeTerm)
                pidValue.i = pid.get(3).asDouble();

            pidValue.maxInt = pid.get(4).asDouble();
            pidValue.maxOut = pid.get(5).asDouble();


            pids.push_back(pidValue);
        }
    } else {
        double default_p = pidTerms & PIDFeedbackTermProportionalTerm ? 500.0 : 0;
        double default_i = pidTerms & PIDFeedbackTermIntegrativeTerm ? 0.1 : 0;
        double default_d = pidTerms & PIDFeedbackTermDerivativeTerm ? 1.0 : 0;
        std::cout<<"PID gain information not found in plugin parameters, using default gains ( "
        <<"P " << default_p << " I " << default_i << " D " << default_d << " )" <<std::endl;
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
            GazeboTripodMotionControl::PID pid = {500, 0.1, 1.0, -1, -1};
            pids.push_back(pid);
        }
    }
}

void GazeboTripodMotionControl::setMinMaxImpedance()
{

    yarp::os::Bottle& name_bot = m_pluginParameters.findGroup("WRAPPER").findGroup("networks");
    std::string name = name_bot.get(1).toString();

    yarp::os::Bottle& kin_chain_bot = m_pluginParameters.findGroup(name);
    if (kin_chain_bot.check("min_stiffness")) {
        std::cout<<"min_stiffness param found!"<<std::endl;
        yarp::os::Bottle& min_stiff_bot = kin_chain_bot.findGroup("min_stiffness");
        if(min_stiff_bot.size()-1 == m_numberOfJoints) {
            for(unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_minStiffness[i] = min_stiff_bot.get(i+1).asDouble();
        } else
            std::cout<<"Invalid number of params"<<std::endl;
    } else
        std::cout<<"No minimum stiffness value found in ini file, default one will be used!"<<std::endl;

    if (kin_chain_bot.check("max_stiffness")) {
        std::cout<<"max_stiffness param found!"<<std::endl;
        yarp::os::Bottle& max_stiff_bot = kin_chain_bot.findGroup("max_stiffness");
        if (max_stiff_bot.size()-1 == m_numberOfJoints) {
            for (unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_maxStiffness[i] = max_stiff_bot.get(i+1).asDouble();
        } else
            std::cout<<"Invalid number of params"<<std::endl;
    }
    else
        std::cout<<"No maximum stiffness value found in ini file, default one will be used!"<<std::endl;

    if (kin_chain_bot.check("min_damping")) {
        std::cout<<"min_damping param found!"<<std::endl;
        yarp::os::Bottle& min_damping_bot = kin_chain_bot.findGroup("min_damping");
        if(min_damping_bot.size()-1 == m_numberOfJoints) {
            for(unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_minDamping[i] = min_damping_bot.get(i+1).asDouble();
        } else
            std::cout<<"Invalid number of params"<<std::endl;
    } else
        std::cout<<"No minimum dampings value found in ini file, default one will be used!"<<std::endl;

    if(kin_chain_bot.check("max_damping")) {
        std::cout<<"max_damping param found!"<<std::endl;
        yarp::os::Bottle& max_damping_bot = kin_chain_bot.findGroup("max_damping");
        if (max_damping_bot.size() - 1 == m_numberOfJoints) {
            for(unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_maxDamping[i] = max_damping_bot.get(i+1).asDouble();
        } else
            std::cout<<"Invalid number of params"<<std::endl;
    } else
        std::cout<<"No maximum damping value found in ini file, default one will be used!"<<std::endl;

    std::cout<<"min_stiffness: [ "<<m_minStiffness.toString()<<" ]"<<std::endl;
    std::cout<<"max_stiffness: [ "<<m_maxStiffness.toString()<<" ]"<<std::endl;
    std::cout<<"min_damping: [ "<<m_minDamping.toString()<<" ]"<<std::endl;
    std::cout<<"max_damping: [ "<<m_maxDamping.toString()<<" ]"<<std::endl;
}

void GazeboTripodMotionControl::setPIDs()
{
    setPIDsForGroup("GAZEBO_PIDS", m_positionPIDs, PIDFeedbackTermAllTerms);
    setPIDsForGroup("GAZEBO_VELOCITY_PIDS", m_velocityPIDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermIntegrativeTerm));
    setPIDsForGroup("GAZEBO_IMPEDANCE_POSITION_PIDS", m_impedancePosPDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermDerivativeTerm));
}

bool GazeboTripodMotionControl::sendPositionsToGazebo(Vector &refs)
{
    for (unsigned int j=0; j<m_numberOfJoints; j++) {
        sendPositionToGazebo(j,refs[j]);
    }
    return true;
}

bool GazeboTripodMotionControl::sendPositionToGazebo(int j,double ref)
{
    gazebo::msgs::JointCmd j_cmd;

    if(ref != m_oldReferencePositions[j])
    {
        // std::cout << "Sending new command: new ref is " << ref << "; old ref was " << m_oldReferencePositions[j] << std::endl;
        prepareJointMsg(j_cmd,j,ref);
        m_jointCommandPublisher->WaitForConnection();
        m_jointCommandPublisher->Publish(j_cmd);
        m_oldReferencePositions[j] = ref;
    }
    return true;
}

void GazeboTripodMotionControl::prepareJointMsg(gazebo::msgs::JointCmd& j_cmd, const int joint_index, const double ref)
{
    j_cmd.set_name(m_jointPointers[joint_index]->GetScopedName());
    j_cmd.mutable_position()->set_target(convertUserToGazebo(joint_index, ref));
    // No need to set PIDS here, they are set in the corresponding simulated joint.
}

bool GazeboTripodMotionControl::sendVelocitiesToGazebo(yarp::sig::Vector& refs) //NOT TESTED
{
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        sendVelocityToGazebo(j,refs[j]);
    }
    return true;
}

bool GazeboTripodMotionControl::sendVelocityToGazebo(int j,double ref)
{
    gazebo::msgs::JointCmd j_cmd;
    prepareJointVelocityMsg(j_cmd,j,ref);
    m_jointCommandPublisher->WaitForConnection();
    m_jointCommandPublisher->Publish(j_cmd);

    return true;
}

void GazeboTripodMotionControl::prepareJointVelocityMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref) //NOT TESTED
{
    GazeboTripodMotionControl::PID velocityPID = m_velocityPIDs[j];

    j_cmd.set_name(m_jointPointers[j]->GetScopedName());
    j_cmd.mutable_velocity()->set_p_gain(velocityPID.p);
    j_cmd.mutable_velocity()->set_i_gain(velocityPID.i);
    j_cmd.mutable_velocity()->set_d_gain(velocityPID.d);
    j_cmd.mutable_velocity()->set_i_max(velocityPID.maxInt);
    j_cmd.mutable_velocity()->set_i_min(-velocityPID.maxInt);
    j_cmd.mutable_velocity()->set_limit(velocityPID.maxOut);

    j_cmd.mutable_velocity()->set_target(convertUserToGazebo(j, ref));
}

bool GazeboTripodMotionControl::sendTorquesToGazebo(yarp::sig::Vector& refs) //NOT TESTED
{
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        sendTorqueToGazebo(j,refs[j]);
    }
    return true;
}

bool GazeboTripodMotionControl::sendTorqueToGazebo(const int j,const double ref) //NOT TESTED
{
    gazebo::msgs::JointCmd j_cmd;
    prepareJointTorqueMsg(j_cmd,j,ref);
    m_jointCommandPublisher->WaitForConnection();
    m_jointCommandPublisher->Publish(j_cmd);
    return true;
}

void GazeboTripodMotionControl::prepareJointTorqueMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref) //NOT TESTED
{
    j_cmd.set_name(m_jointPointers[j]->GetScopedName());
//    j_cmd.mutable_position()->set_p_gain(0.0);
//    j_cmd.mutable_position()->set_i_gain(0.0);
//    j_cmd.mutable_position()->set_d_gain(0.0);
//    j_cmd.mutable_velocity()->set_p_gain(0.0);
//    j_cmd.mutable_velocity()->set_i_gain(0.0);
//    j_cmd.mutable_velocity()->set_d_gain(0.0);
    j_cmd.set_force(ref);
}

void GazeboTripodMotionControl::sendImpPositionToGazebo ( const int j, const double des )
{
    if(j >= 0 && j < m_numberOfJoints) {
        /*
         Here joint positions and speeds are in [deg] and [deg/sec].
         Therefore also stiffness and damping has to be [Nm/deg] and [Nm*sec/deg].
         */
        //std::cout<<"m_velocities"<<j<<" : "<<m_velocities[j]<<std::endl;
        double q = m_positions[j] - m_zeroPosition[j];
        double t_ref = -m_impedancePosPDs[j].p * (q - des) - m_impedancePosPDs[j].d * m_velocities[j] + m_torqueOffsett[j];
        sendTorqueToGazebo(j, t_ref);
    }
}

void GazeboTripodMotionControl::sendImpPositionsToGazebo ( Vector &dess )
{
    for(unsigned int i = 0; i < m_numberOfJoints; ++i)
        sendImpPositionToGazebo(i, dess[i]);
}

double GazeboTripodMotionControl::convertGazeboToUser(int joint, gazebo::math::Angle value)
{
    double newValue = 0;
    switch(m_jointTypes[joint])
    {
        case JointType_Revolute:
        {
            newValue = value.Degree();
            break;
        }

        case JointType_Prismatic:
        {
            // For prismatic joints there is no getMeter() or something like that. The only way is to use .radiant() to get internal
            // value without changes
            newValue = value.Radian();
            break;
        }

        default:
        {
            yError() << "Cannot convert measure from Gazebo to User units, type of joint not supported for axes " <<
                                m_jointNames[joint] << " type is " << m_jointTypes[joint];
            break;
        }
    }
    return newValue;
}

double GazeboTripodMotionControl::convertGazeboToUser(int joint, double value)
{
    double newValue = 0;
    switch(m_jointTypes[joint])
    {
        case JointType_Revolute:
        {
            newValue = GazeboYarpPlugins::convertRadiansToDegrees(value);
            break;
        }

        case JointType_Prismatic:
        {
            // For prismatic joints internal representation is already meter, nothing to do here.
            newValue = value;
            break;
        }

        default:
        {
            yError() << "Cannot convert measure from Gazebo to User units, type of joint not supported for axes " <<
                                m_jointNames[joint] << " type is " << m_jointTypes[joint];
            break;
        }
    }
    return newValue;
}

double * GazeboTripodMotionControl::convertGazeboToUser(double *values)
{
    for(int i=0; i<m_numberOfJoints; i++)
        values[i] = convertGazeboToUser(i, values[i]);
    return values;
}

double GazeboTripodMotionControl::convertUserToGazebo(int joint, double value)
{
    double newValue = 0;
    switch(m_jointTypes[joint])
    {
        case JointType_Revolute:
        {
            newValue = GazeboYarpPlugins::convertDegreesToRadians(value);
            break;
        }

        case JointType_Prismatic:
        {
            newValue = value;
            break;
        }

        default:
        {
            yError() << "Cannot convert measure from User to Gazebo units, type of joint not supported";
            break;
        }
    }
    return newValue;
}

double * GazeboTripodMotionControl::convertUserToGazebo(double *values)
{
    for(int i=0; i<m_numberOfJoints; i++)
        values[i] = convertGazeboToUser(i, values[i]);
    return values;
}

double GazeboTripodMotionControl::convertUserGainToGazeboGain(int joint, double value)
{
    double newValue = 0;
    switch(m_jointTypes[joint])
    {
        case JointType_Revolute:
        {
            newValue = GazeboYarpPlugins::convertRadiansToDegrees(value);
            break;
        }

        case JointType_Prismatic:
        {
            newValue = value;
            break;
        }

        default:
        {
            yError() << "Cannot convert measure from User to Gazebo units, type of joint not supported";
            break;
        }
    }
    return newValue;
}

double GazeboTripodMotionControl::convertGazeboGainToUserGain(int joint, double value)
{
    double newValue = 0;
    switch(m_jointTypes[joint])
    {
        case JointType_Revolute:
        {
            newValue = GazeboYarpPlugins::convertDegreesToRadians(value);
            break;
        }

        case JointType_Prismatic:
        {
            newValue = value;
            break;
        }

        default:
        {
            yError() << "Cannot convert measure from Gazebo gains to User gain units, type of joint not supported for axes " <<
                                m_jointNames[joint] << " type is " << m_jointTypes[joint];
            break;
        }
    }
    return newValue;
}

bool GazeboTripodMotionControl::getTorque(int j, double *t)
{
    if(j >= 0 && j < m_numberOfJoints)
    {
        *t = 0; //@@@@ THIS IS NOT IMPLEMENTED YET
	return true;
    }
    return false;
}

bool GazeboTripodMotionControl::getTorques(double *t)
{
    bool ret = true;
    for(unsigned int i = 0; i < m_numberOfJoints; ++i)
    ret |= getTorque(i,&t[i]);
    return ret;
}

bool GazeboTripodMotionControl::getRefTorque(int j, double* t)
{
    if (t && j >= 0 && j < (int)m_numberOfJoints) {
        *t = m_referenceTorques[j];
        return true;
    }
    return false;
}

bool GazeboTripodMotionControl::getRefTorques(double* t)
{
    if (!t) return false;
    for(unsigned int j = 0; j < m_numberOfJoints; ++j) {
        t[j] = m_referenceTorques[j];
    }
    return true;
}

bool GazeboTripodMotionControl::getTargetPosition(const int joint, double *ref)
{
    if (ref && joint >= 0 && joint < (int)m_numberOfJoints)
    {
      *ref = m_trajectoryGenerationReferencePosition[joint];
      return true;
    }
    return false;
}

bool GazeboTripodMotionControl::getTargetPositions(double *refs)
{
    if (!refs) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < this->m_numberOfJoints && ret; i++) {
        ret = getTargetPosition(i, &refs[i]);
    }
    return ret;
}

bool GazeboTripodMotionControl::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    if (!joints || !refs) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++) {
        ret = getTargetPosition(joints[i], &refs[i]);
    }
    return ret;
}

bool GazeboTripodMotionControl::getRefVelocity(const int joint, double *vel) 
{
    if (vel && joint >= 0 && joint < (int)m_numberOfJoints)
    {
      *vel = m_referenceVelocities[joint];
      return true;
    }
    return false;
  
}

bool GazeboTripodMotionControl::getRefVelocities(double *vels) 
{
     if (!vels) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < this->m_numberOfJoints && ret; i++) {
        ret = getRefVelocity(i, &vels[i]);
    }
    return ret; 
}

bool GazeboTripodMotionControl::getRefVelocities(const int n_joint, const int *joints, double *vels) 
{
    if (!joints || !vels) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++) {
        ret = getRefVelocity(joints[i], &vels[i]);
    }
    return ret;
}

bool GazeboTripodMotionControl::setRefTorque(int j, double t)
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        m_referenceTorques[j] = t;
        return true;
    }
    return false;
}

bool GazeboTripodMotionControl::setRefTorques(const double* t)
{
    if (!t) return false;
    for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
        m_referenceTorques[j] = t[j];
    }
    return true;
}

bool GazeboTripodMotionControl::setTorqueMode()
{
    bool ret = true;
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        ret = ret && this->setControlMode(j, VOCAB_CM_TORQUE);
    }
    return ret;
}

bool GazeboTripodMotionControl::setTorquePid(int joint, const Pid &pid){return NOT_YET_IMPLEMENTED("setTorquePid");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::setTorquePids(const Pid *newPids){return NOT_YET_IMPLEMENTED("setTorquePids");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::getTorquePid(int joint, Pid *pid){return NOT_YET_IMPLEMENTED("getTorquePid");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::getTorquePids(Pid *pids) {return NOT_YET_IMPLEMENTED("getTorquePids");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::getTorqueRange(int, double*, double *){return NOT_YET_IMPLEMENTED("getTorqueRange");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::getTorqueRanges(double *, double *){return NOT_YET_IMPLEMENTED("getTorqueRanges");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::setTorqueErrorLimit(int , double ){return NOT_YET_IMPLEMENTED("setTorqueErrorLimit");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::setTorqueErrorLimits(const double *){return NOT_YET_IMPLEMENTED("setTorqueErrorLimits");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::getTorqueError(int , double *){return NOT_YET_IMPLEMENTED("getTorqueError");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::getTorqueErrors(double *){return NOT_YET_IMPLEMENTED("getTorqueErrors");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::getTorquePidOutput(int , double *){return NOT_YET_IMPLEMENTED("getTorquePidOutput");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::getTorquePidOutputs(double *){return NOT_YET_IMPLEMENTED("getTorquePidOutputs");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::getTorqueErrorLimit(int , double *){return NOT_YET_IMPLEMENTED("getTorqueErrorLimit");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::getTorqueErrorLimits(double *){return NOT_YET_IMPLEMENTED("getTorqueErrorLimits");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::resetTorquePid(int ){return NOT_YET_IMPLEMENTED("resetTorquePid");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::disableTorquePid(int ){return NOT_YET_IMPLEMENTED("disableTorquePid");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::enableTorquePid(int ){return NOT_YET_IMPLEMENTED("enableTorquePid");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::setTorqueOffset(int , double ){return NOT_YET_IMPLEMENTED("setTorqueOffset");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::getBemfParam(int , double *){return NOT_YET_IMPLEMENTED("getBemfParam");} //NOT IMPLEMENTED
bool GazeboTripodMotionControl::setBemfParam(int , double ){return NOT_YET_IMPLEMENTED("getBemfParam");} //NOT IMPLEMENTED
