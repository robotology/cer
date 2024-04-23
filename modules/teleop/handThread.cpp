#include "handThread.h"
#include <yarp/os/LogStream.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>
#include <limits>
#include <cmath>
using namespace yarp::dev;
using namespace yarp::os;
using namespace std;
using namespace yarp::sig;
using namespace yarp::math;



void HandThread::printState()
{
    yInfo("pose=%s; hand=%s;",
          reachState  ? "fixed-position" : "fixed-orientation",
          vels[0] > 0.0 ? "closing":"opening");
}

bool HandThread::openControlBoards(yarp::os::Searchable& rf)
{
    gain            = rf.check("gain",     Value(1)).asFloat64();
    wrist_heave     = 0.02;

    if (rf.check("wrist-heave"))
    {
        wrist_heave = rf.find("wrist-heave").asFloat64();
        mode        = "full_pose+no_torso_heave";
    }
    else
    {
        mode = "full_pose+no_torso_no_heave";
    }

    string   part  = ((arm_type == left_hand) ? "left_hand" : "right_hand");
    string   robot = rf.check("robot", Value("cer")).asString();
    Property optHand("(device remote_controlboard)");
    optHand.put("remote", ("/" + robot + "/" + part).c_str());
    optHand.put("local",  ("/cer_teleop/"    + part).c_str());

    if (!drvHand.open(optHand))
    {
        yError() << "teleop: failed to open remote control board for" << part;
        return false;
    }

    if (!drvHand.view(ienc))
    {
        yError() << "Teleoperation Module: dynamic_cast to IEncoders interface failed";
        return false;
    }

    if (!drvHand.view(imod))
    {
        yError() << "Teleoperation Module: dynamic_cast to IControlMode  interface failed";
        return false;
    }

    if (!drvHand.view(ivel))
    {
        yError() << "Teleoperation Module: dynamic_cast to IVelocityControl interface failed";
        return false;
    }

    if (!drvHand.view(ipos))
    {
        yError() << "Teleoperation Module: dynamic_cast to IPositionControl2 interface failed";
        return false;
    }

    if (!drvHand.view(ilim))
    {
        yError() << "Teleoperation Module: dynamic_cast to IControlLimits interface failed";
        return false;
    }

    int nAxes;
    ienc->getAxes(&nAxes);
    for (int i = 0; i < nAxes; i++)
    {
        modes.push_back(VOCAB_CM_VELOCITY);
        vels.push_back(40.0);
        ctrlRange range;
        ilim->getLimits(i, &range.min, &range.size);
        range.size -= range.min;
        controlRanges.push_back(range);
    }

    return true;
}

void HandThread::stopReaching()
{
    Property& req = robotTargetPort.prepare();
    req.clear();
    req.put("stop", Value());
    robotTargetPort.write();

    Bottle cmd, reply;
    cmd.addVocab32(Vocab32::encode("stop"));
    if (robotCmdPort.write(cmd, reply))
    {
        if (reply.get(0).asVocab32() != Vocab32::encode("ack"))
        {
            yError("Something went wrong while stopping");
        }
    }
    else
    {
        yError() << "Unable to communicate to controller" << ((arm_type == left_hand) ? "left hand" : "right hand");
    }
}

void HandThread::goToPose(const Vector &xd, const Vector &od)
{

    Vector payLoad;
    //payLoad.push_back(0.0); // uncontrolled torso-heave
    //payLoad.push_back(wrist_heave);
    payLoad = cat(payLoad, xd);
    payLoad = cat(payLoad, od);

    Bottle target;
    target.addList().read(payLoad);

    Bottle  params;
    Bottle& list  = params.addList();
    Bottle& bLoad = list.addList();
    bLoad.addString("mode");
    bLoad.addString(mode);

    Bottle& btorso_heave = list.addList();
    btorso_heave.addString("torso_heave");
    btorso_heave.addFloat64(0.0);

    Bottle& blower_arm_heave = list.addList();
    blower_arm_heave.addString("lower_arm_heave");
    blower_arm_heave.addFloat64(wrist_heave);

    Property& prop = robotTargetPort.prepare();
    prop.clear();
    prop.put("parameters",params.get(0));
    prop.put("target",target.get(0));
    robotTargetPort.write();
}

#ifdef ROS_MSG
void HandThread::updateRVIZ(const Vector &xd, const Vector &od)
{
    Matrix   m;
    double   yarpTimeStamp = yarp::os::Time::now();
    uint64_t time;
    uint64_t nsec_part;
    uint64_t sec_part;

    m = axis2dcm(od);
    m.setSubcol(xd, 0, 3);
    mtx.lock();
    criticalSection.tf = m;

    time      = (uint64_t)(yarpTimeStamp * 1000000000UL);
    nsec_part = (time % 1000000000UL);
    sec_part  = (time / 1000000000UL);

    if (sec_part > std::numeric_limits<unsigned int>::max())
    {
        yWarning() << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }

    yarp::rosmsg::visualization_msgs::MarkerArray& markerarray = rosPublisherPort.prepare();
    yarp::rosmsg::visualization_msgs::Marker       marker;

    markerarray.markers.clear();

    marker.header.frame_id   = "mobile_base_body_link";
    marker.header.stamp.sec  = (yarp::os::NetUint32)sec_part;
    marker.header.stamp.nsec = (yarp::os::NetUint32)nsec_part;
    marker.ns                = "cer-teleop_namespace";
    marker.type              = yarp::rosmsg::visualization_msgs::Marker::CYLINDER;
    marker.action            = yarp::rosmsg::visualization_msgs::Marker::ADD;

    //center
    Quaternion q;
    q.fromRotationMatrix(axis2dcm(od));

    marker.id                 = arm_type + 1;
    marker.pose.position.x    = xd[0];
    marker.pose.position.y    = xd[1];
    marker.pose.position.z    = xd[2];
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x            = 0.05;
    marker.scale.y            = 0.05;
    marker.scale.z            = 0.05;
    marker.color.a            = 0.5;
    marker.color.r            = 0.0;
    marker.color.g            = 1.0;
    marker.color.b            = 0.0;
    marker.lifetime.sec       = 1.0;
    marker.lifetime.nsec      = 0.0;
    marker.mesh_use_embedded_materials = false;

    markerarray.markers.push_back(marker);

    rosPublisherPort.write();
}
#endif

/**********************************************************/
void HandThread::updateGazebo(const Vector& xd, const Vector& od)
{
    if (gazeboPort.getOutputCount() > 0)
    {
        Vector  rpy = dcm2rpy(axis2dcm(od));
        Bottle& b   = gazeboPort.prepare();
        b.clear();
        b.addString("setPose");
        b.addString("frame22");
        b.addFloat64(xd[0]);
        b.addFloat64(xd[1]);
        b.addFloat64(xd[2]);
        b.addFloat64(rpy[0]);
        b.addFloat64(rpy[1]);
        b.addFloat64(rpy[2]);
        b.addString("frame11::link");
        gazeboPort.write();
    }
}

inline double getRad(double x, double y)
{
    return sqrt(x * x + y * y);
}

inline Vector mVector(double x, double y, double z)
{
    Vector r(3);
    r[0] = x;
    r[1] = y;
    r[2] = z;
    return r;
}

void HandThread::reachingHandler(const bool dragging_switch, const Matrix& pose)
{
    if (dragging_switch)
    {
        if (handDraggingStatus == idle)
        {
            if(((targetDistance < targetRadius) || targetRadius < 0.000001))
            {
                handDraggingStatus = triggered;
            }
        }
        else if (handDraggingStatus == triggered)
        {
            if (++b0_pressedCount * getPeriod() > 0.5)
            {
                pose0              = pose;
                pos0               = pose0.getCol(3);
                x0                 = cur_x;
                o0                 = cur_o;
                handDraggingStatus = running;

                pos0.pop_back();
            }
        }
        else //dragging status == running
        {
            Vector od, Target;
            //double r;
            Matrix m  = eye(4, 4);
            Matrix H0 = axis2dcm(o0);
            pos       = pose.getCol(3);
            pos.pop_back();



            Target = x0 + (pos - pos0);
            //r      = getRad(cur_x[0], cur_x[1]);

            if (getRad(Target[0], Target[1]) > 0.35)//|| getRad(Target[0], Target[1]) > getRad(x0[0], x0[1]))
            {
                xd = Target;
            }
            else
            {
                double t = atan2(Target[1], Target[0]);
                xd = x0 = mVector(0.35 * cos(t), 0.37 * sin(t), cur_x[2]);
                pose0 = pose;
                pos0  = pose.getCol(3);
                pos0.pop_back();
            }


            xd.push_back(1);
            H0(0,3)   = x0[0];
            H0(1,3)   = x0[1];
            H0(2,3)   = x0[2];
            if(absoluteRotation)
            {
                m = (pose * pose0.transposed()) * H0;
            }
            else
            {
                m = H0 * (pose0.transposed() * pose);
            }


            if(simultMovRot)
            {
                od = dcm2axis(m);
                xd.pop_back();
                if (button2)
                {

                    pose0.setCol(3, pose.getCol(3));
                    pos0 = pose.getCol(3);
                    pos0.pop_back();
                    xd = cur_x;
                    x0 = xd;

                    goToPose(cur_x, od);
                }
                else
                {
                    o0 = od = cur_o;
                    pose0.setCol(0, pose.getCol(0));
                    pose0.setCol(1, pose.getCol(1));
                    pose0.setCol(2, pose.getCol(2));
                    goToPose(xd, cur_o);
                }

            }
            else if (reachState)
            {
                od = dcm2axis(m);
                goToPose(fixedPosition, od);
            }
            else
            {
                xd.pop_back();
                goToPose(xd, fixedOrientation);
            }

            updateGazebo(xd, od);
#ifdef ROS_MSG
            updateRVIZ(xd, od);
#endif
        }
    }
    else
    {
        if (handDraggingStatus == triggered)
        {
            reachState       = !reachState;
            fixedPosition    = cur_x;
            fixedOrientation = cur_o;
            printState();
        }

        if (b0_pressedCount != 0)
        {
            stopReaching();
            updateGazebo(cur_x, cur_o);
#ifdef ROS_MSG
            updateRVIZ(cur_x, cur_o);
#endif
        }

        handDraggingStatus = idle;
        b0_pressedCount    = 0;
    }
}

void HandThread::handHandler(const bool hand_grip_switch)
{

    if (hand_grip_switch || !singleButton)
    {
        if (handGripStatus == idle)
        {
            handGripStatus = triggered;
        }
        else if (handGripStatus == triggered)
        {
            if (++b1_pressedCount * getPeriod() > 0.5)
            {

                imod->setControlMode(0, controlMode);
                imod->setControlMode(1, controlMode);
                handGripStatus = running;
            }
        }
        else // running
        {

            if(controlMode == VOCAB_CM_VELOCITY)
            {
                ivel->velocityMove((vels * button1).data());
            }
            else if(controlMode == VOCAB_CM_POSITION_DIRECT)
            {
                for(int i = 0; i < controlRanges.size(); i++)
                {
                    ipos->setPosition(i, controlRanges[i].min + controlRanges[i].size * button1);

                }
            }
        }
    }
    else
    {
        if (handGripStatus == triggered)
        {
            if(controlMode == VOCAB_CM_VELOCITY && singleButton)
            {
                vels = -1.0 * vels;
            }
            printState();
        }

        if (b1_pressedCount != 0)
        {

            ivel->stop();
        }

        handGripStatus  = idle;
        b1_pressedCount = 0;
    }
}

double HandThread::getPeriod()
{
    return 0.01;
}

bool HandThread::threadInit()
{
    string hand;
    bool   r;
    hand = arm_type == left_hand ? "left" : "right";
    reachState         = false;
    handDraggingStatus = handGripStatus  = idle;
    b0_pressedCount    = b1_pressedCount = 0;
    cur_x.resize(3, 0.0);
    cur_o.resize(4, 0.0);
    x0.resize(3, 0.0);
    o0.resize(4, 0.0);
    fixedPosition.resize(3, 0.0);
    fixedOrientation.resize(4, 0.0);

#ifdef ROS_MSG
    if (!rosPublisherPort.topic("/cer_teleop_marker_" + hand))
    {
        yError("Unable to publish data on /cer_teleop_marker topic");
        yWarning("Check your yarp-ROS network configuration");
        return false;
    }
#endif
    r = gazeboPort.open("/cer_teleop/gazebo_" + hand+":o");
    r &= robotTargetPort.open("/cer_teleop/target_" + hand + ":o");
    r &= robotStatePort.open("/cer_teleop/state_" + hand + ":i");
    r &= robotCmdPort.open("/cer_teleop/cmd_" + hand + ":rpc");

    return r;
}

void HandThread::threadRelease()
{
    ivel->stop();

    stopReaching();
    for (size_t i = 0; i < modes.size(); i++)
    {
        modes[i] = VOCAB_CM_POSITION;
    }

    imod->setControlModes(modes.data());

    robotTargetPort.interrupt();
    robotTargetPort.close();
    robotStatePort.close();
    robotCmdPort.close();

    drvHand.close();
    gazeboPort.close();
}

void HandThread::run()
{
    if (robotStatePort.getInputCount() > 0)
    {
        if (Vector* pose = robotStatePort.read(false))
        {
            cur_x = pose->subVector(0,2);
            cur_o = pose->subVector(3,6);
        }

        getData();

        reachingHandler(button0, pose);
        handHandler(button1);
    }
    else
    {
        double time = yarp::os::Time::now();
        if(fmod(time, 1.0) == 0)
        {
            yError("No robot connected!");
        }
    }
}
