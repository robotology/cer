/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "GazeboTripodMotionControl.h"
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/Handler.hh>

#include <cstdio>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/math/Angle.hh>

#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace cer::dev;


const double RobotPositionTolerance = 0.9;

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
    return solver.fkin(client, sim);
}

bool GazeboTripodMotionControl::tripod_Sim2client(yarp::sig::Vector &sim, yarp::sig::Vector &client)
{
    // The caller must use mutex or private data
//     return solver.ikin(sim, client);
//     client = m_last_measJointPos;
    client = m_referenceElongations;
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

    bool ret;
    if(! (ret = gazebo_init()) )
    {

    }
    ret &= init_kinematics();
    return ret;
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
//     solver.setInitialGuess(m_);     // TODO: ask Ugo if it make sense
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

    std::cout<<"Robot Name: "<<m_robot->GetName() <<std::endl;
    std::cout<<"# Joints: "<<m_robot->GetJoints().size() <<std::endl;
    std::cout<<"# Links: "<<m_robot->GetLinks().size() <<std::endl;

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

    setMinMaxPos();
    setMinMaxImpedance();
    setPIDs();
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
    amp = 1; // initially on - ok for simulator
    started = false;
    m_controlMode = new int[m_numberOfJoints];
    m_interactionMode = new int[m_numberOfJoints];
    m_isMotionDone = new bool[m_numberOfJoints];
    m_clock = 0;
    m_torqueOffsett = 0;
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
        m_controlMode[j] = VOCAB_CM_POSITION_DIRECT;
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
        m_interactionMode[j] = VOCAB_IM_STIFF;

    std::cout << "gazebo_init set pid done!" << std::endl;

    this->m_updateConnection =
    gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboTripodMotionControl::onUpdate,
                                                                     this, _1));

    m_gazeboNode = gazebo::transport::NodePtr(new gazebo::transport::Node);
    m_gazeboNode->Init(this->m_robot->GetWorld()->GetName());
    m_jointCommandPublisher = m_gazeboNode->Advertise<gazebo::msgs::JointCmd>(std::string("~/") + this->m_robot->GetName() + "/joint_cmd");

    _T_controller = 1;

    std::stringstream ss(m_pluginParameters.find("initialConfiguration").toString());
    if (!(m_pluginParameters.find("initialConfiguration") == "")) {
        double tmp = 0.0;
        yarp::sig::Vector initial_config(m_numberOfJoints);
        unsigned int counter = 1;
        while (ss >> tmp) {
            if(counter > m_numberOfJoints) {
                std::cout<<"To many element in initial configuration, stopping at element "<<counter<<std::endl;
                break;
            }
            initial_config[counter-1] = tmp;
            m_trajectoryGenerationReferencePosition[counter - 1] = tmp; // here values are elongations in meter
            m_referenceElongations[counter - 1] = tmp; // here values are elongations in meter
            tripod_client2Sim(m_referenceElongations, m_referencePositions);
            m_positions[counter - 1] = tmp; // here values are elongations in meter
            counter++;
        }
        std::cout<<"INITIAL CONFIGURATION IS: "<<initial_config.toString()<<std::endl;

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
    return true;
}

void GazeboTripodMotionControl::computeTrajectory(const int j)
{
    // TODO: how to do this? Does it make sense?
    if ((m_referencePositions[j] - m_trajectoryGenerationReferencePosition[j]) < -RobotPositionTolerance) {
        if (m_trajectoryGenerationReferenceSpeed[j] !=0)
            m_referencePositions[j] += (m_trajectoryGenerationReferenceSpeed[j] / 1000.0) * m_robotRefreshPeriod * _T_controller;
        m_isMotionDone[j] = false;
    } else if ((m_referencePositions[j] - m_trajectoryGenerationReferencePosition[j]) > RobotPositionTolerance) {
        if (m_trajectoryGenerationReferenceSpeed[j] != 0)
            m_referencePositions[j]-= (m_trajectoryGenerationReferenceSpeed[j] / 1000.0) * m_robotRefreshPeriod * _T_controller;
        m_isMotionDone[j] = false;
    } else {
        m_referencePositions[j] = m_trajectoryGenerationReferencePosition[j];
        m_isMotionDone[j] = true;
    }
}

void GazeboTripodMotionControl::onUpdate(const gazebo::common::UpdateInfo& _info)
{
    m_clock++;

    if (!started) {//This is a simple way to start with the robot in standing position
        started = true;
        for (unsigned int j = 0; j < m_numberOfJoints; ++j)
            sendPositionToGazebo (j, m_positions[j]);
    }

    // Sensing position & torque
    for (unsigned int jnt_cnt = 0; jnt_cnt < m_jointPointers.size(); jnt_cnt++) {
//TODO: consider multi-dof joint ?
        m_positions[jnt_cnt] = m_jointPointers[jnt_cnt]->GetAngle (0).Degree(); // here values are degrees (gazebo user joint)
        m_velocities[jnt_cnt] = GazeboYarpPlugins::convertRadiansToDegrees(m_jointPointers[jnt_cnt]->GetVelocity(0));
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
        } else if (m_controlMode[j] == VOCAB_CM_OPENLOOP) {
            //OpenLoop control sends torques to gazebo at this moment.
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
    for(unsigned int i = 0; i < m_numberOfJoints; ++i) {
//         m_jointLimits[i].max = m_jointPointers[i]->GetUpperLimit(0).Degree();
//         m_jointLimits[i].min = m_jointPointers[i]->GetLowerLimit(0).Degree();
        std::cout << " gazebo max: " << m_jointPointers[i]->GetUpperLimit(0).Degree() <<std::endl;
        std::cout << " gazebo min: " << m_jointPointers[i]->GetLowerLimit(0).Degree() <<std::endl;
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
                yInfo() << "m_jointName [" << i << "] is " << m_jointNames[i];
                m_jointPointers[i] = this->m_robot->GetJoint(gazebo_joint_name);
            }
        }

        if (!joint_found) {
            yError() << "GazeboTripodMotionControl::setJointNames(): cannot find joint " << m_jointNames[i]
                     << " ( " << i << " of " << nr_of_joints << " ) " << "\n";
            yError() << "jointNames is " << joint_names_bottle.toString() << "\n";
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
    prepareJointMsg(j_cmd,j,ref);
    m_jointCommandPublisher->WaitForConnection();
    m_jointCommandPublisher->Publish(j_cmd);
    return true;
}

void GazeboTripodMotionControl::prepareJointMsg(gazebo::msgs::JointCmd& j_cmd, const int joint_index, const double ref)
{
    GazeboTripodMotionControl::PID positionPID = m_positionPIDs[joint_index];

    j_cmd.set_name(m_jointPointers[joint_index]->GetScopedName());
    j_cmd.mutable_position()->set_target(GazeboYarpPlugins::convertDegreesToRadians(ref));
    j_cmd.mutable_position()->set_p_gain(positionPID.p);
    j_cmd.mutable_position()->set_i_gain(positionPID.i);
    j_cmd.mutable_position()->set_d_gain(positionPID.d);
    j_cmd.mutable_position()->set_i_max(positionPID.maxInt);
    j_cmd.mutable_position()->set_i_min(-positionPID.maxInt);
    j_cmd.mutable_position()->set_limit(positionPID.maxOut);
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

    j_cmd.mutable_velocity()->set_target(GazeboYarpPlugins::convertDegreesToRadians(ref));
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
