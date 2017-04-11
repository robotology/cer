#include "handThread.h"
#include <yarp/os/LogStream.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>
#include <limits>
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
    gain            = rf.check("gain",     Value(1.5)).asDouble();
    wrist_heave     = 0.02;

    if (rf.check("wrist-heave"))
    {
        wrist_heave = rf.find("wrist-heave").asDouble();
        mode        = "full_pose+no_torso_no_heave";
    }
    else
    {
        mode = "full_pose+no_torso_heave";
    }

    string   part  = ((arm_type == left_hand) ? "left_hand" : "right_hand");
    string   robot = rf.check("robot", Value("cer")).asString();
    Property optHand("(device remote_controlboard)");
    optHand.put("remote", ("/" + robot + "/" + part).c_str());
    optHand.put("local",  ("/cer_teleop/"    + part).c_str());

    if (!drvHand.open(optHand))
    {
        return false;
    }

    if (!drvHand.view(ienc))
    {
        yError() << "Teleoperation Module: dynamic_cast to IEncoders interface failed";
        return false;
    }

    if (!drvHand.view(imod))
    {
        yError() << "Teleoperation Module: dynamic_cast to IControlMode2 interface failed";
        return false;
    }

    if (!drvHand.view(ivel))
    {
        yError() << "Teleoperation Module: dynamic_cast to IVelocityControl2 interface failed";
        return false;
    }

    int nAxes;
    ienc->getAxes(&nAxes);
    for (int i = 0; i < nAxes; i++)
    {
        modes.push_back(VOCAB_CM_VELOCITY);
        vels.push_back(40.0);
    }
}

void HandThread::stopReaching()
{
    Bottle cmd, reply;
    cmd.addVocab(Vocab::encode("stop"));
    if (robotCmdPort.write(cmd, reply))
    {
        if (reply.get(0).asVocab() != Vocab::encode("ack"))
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
    Vector od_ = od;
    od_       *= od_[3];

    od_.pop_back();

    Vector payLoad;
    payLoad.push_back(0.0); // uncontrolled torso-heave
    payLoad.push_back(wrist_heave);
    payLoad = cat(payLoad, xd);
    payLoad = cat(payLoad, od_);

    Bottle target;
    target.addList().read(payLoad);

    Bottle  params;
    Bottle& bLoad = params.addList().addList();
    bLoad.addString("mode");
    bLoad.addString(mode);

    Property& prop = robotTargetPort.prepare();
    prop.clear();
    prop.put("parameters",params.get(0));
    prop.put("target",target.get(0));
    robotTargetPort.write();
}

void HandThread::updateRVIZ(const Vector &xd, const Vector &od)
{
    double   yarpTimeStamp = yarp::os::Time::now();
    uint64_t time;
    uint64_t nsec_part;
    uint64_t sec_part;

    time      = (uint64_t)(yarpTimeStamp * 1000000000UL);
    nsec_part = (time % 1000000000UL);
    sec_part  = (time / 1000000000UL);

    if (sec_part > std::numeric_limits<unsigned int>::max())
    {
        yWarning() << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }

    visualization_msgs_MarkerArray& markerarray = rosPublisherPort.prepare();
    visualization_msgs_Marker       marker;

    markerarray.markers.clear();

    marker.header.frame_id   = "mobile_base_body_link";
    marker.header.stamp.sec  = (yarp::os::NetUint32)sec_part;
    marker.header.stamp.nsec = (yarp::os::NetUint32)nsec_part;
    marker.ns                = "cer-teleop_namespace";
    marker.type              = visualization_msgs_Marker::SPHERE;
    marker.action            = visualization_msgs_Marker::ADD;

    //center
    Quaternion q;
    q.fromRotationMatrix(axis2dcm(od));

    marker.id                 = 1;
    marker.pose.position.x    = xd[0];
    marker.pose.position.y    = xd[1];
    marker.pose.position.z    = xd[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = 0.05;
    marker.scale.y            = 0.05;
    marker.scale.z            = 0.05;
    marker.color.a            = 0.5;
    marker.color.r            = 0.0;
    marker.color.g            = 1.0;
    marker.color.b            = 0.0;

    markerarray.markers.push_back(marker);

    rosPublisherPort.write();
}

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
        b.addDouble(xd[0]);
        b.addDouble(xd[1]);
        b.addDouble(xd[2]);
        b.addDouble(rpy[0]);
        b.addDouble(rpy[1]);
        b.addDouble(rpy[2]);
        b.addString("frame11::link");
        gazeboPort.write();
    }
}



void HandThread::reachingHandler(const bool dragging_switch, const Vector& pos, const Vector& rpy)
{
    if (dragging_switch)
    {
        if (handDraggingStatus == idle)
        {
            handDraggingStatus = triggered;
        }
        else if (handDraggingStatus == triggered)
        {
            if (++b0_pressedCount * getPeriod() > 0.5)
            {
                pos0[0]            = pos[0];
                pos0[1]            = pos[1];
                pos0[2]            = pos[2];
                rpy0[0]            = rpy[0];
                rpy0[1]            = rpy[1];
                rpy0[2]            = rpy[2];
                x0                 = cur_x;
                o0                 = cur_o;
                handDraggingStatus = running;
            }
        }
        else
        {
            Vector xd(4, 0.0);
            Vector drpy(3);
            Vector ax(4, 0.0), ay(4, 0.0), az(4, 0.0);

            xd[0]     = gain * (pos[0] - pos0[0]);
            xd[1]     = gain * (pos[1] - pos0[1]);
            xd[2]     = gain * (pos[2] - pos0[2]);
            xd[3]     = 1.0;
            Matrix H0 = eye(4,4);
            H0(0,3)   = x0[0];
            H0(1,3)   = x0[1];
            H0(2,3)   = x0[2];
            xd        = H0 * xd;
            xd.pop_back();

            drpy[0]   = gain * (rpy[0] - rpy0[0]);
            drpy[1]   = gain * (rpy[1] - rpy0[1]);
            drpy[2]   = gain * (rpy[2] - rpy0[2]);
            ax[0]     = 1.0;
            ay[1]     = 1.0;
            az[2]     = 1.0;
            ax[3]     = drpy[2];
            ay[3]     = drpy[1] * ((arm_type == right_hand) ? -1.0 : +1.0);
            az[3]     = drpy[0] * ((arm_type == right_hand) ? -1.0 : +1.0);
            Matrix Rd = axis2dcm(o0) * axis2dcm(ax) * axis2dcm(ay) * axis2dcm(az);
            Vector od = dcm2axis(Rd);

            if (reachState)
            {
                goToPose(fixedPosition, od);
            }
            else
            {
                goToPose(xd, fixedOrientation);
            }

            updateGazebo(xd, od);
            updateRVIZ(xd, od);
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
            updateRVIZ(cur_x, cur_o);
        }

        handDraggingStatus = idle;
        b0_pressedCount    = 0;
    }
}

void HandThread::handHandler(const bool hand_grip_switch)
{
    if (hand_grip_switch)
    {
        if (handGripStatus == idle)
        {
            handGripStatus = triggered;
        }
        else if (handGripStatus == triggered)
        {
            if (++b1_pressedCount * getPeriod() > 0.5)
            {

                imod->setControlModes(modes.getFirst());
                handGripStatus = running;
            }
        }
        else
        {

            ivel->velocityMove(vels.data());
        }
    }
    else
    {
        if (handGripStatus == triggered)
        {
            vels = -1.0 * vels;
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
    hand = arm_type == left_hand ? "left" : "right";
    reachState         = false;
    handDraggingStatus = handGripStatus  = idle;
    b0_pressedCount    = b1_pressedCount = 0;
    pos0.resize(3, 0.0);
    rpy0.resize(3, 0.0);
    cur_x.resize(3, 0.0);
    cur_o.resize(4, 0.0);
    x0.resize(3, 0.0);
    o0.resize(4, 0.0);
    fixedPosition.resize(3, 0.0);
    fixedOrientation.resize(4, 0.0);


    if (!rosPublisherPort.topic("/cer_teleop_marker"))
    {
        yError("Unable to publish data on /cer_teleop_marker topic");
        yWarning("Check your yarp-ROS network configuration");
        return false;
    }

    gazeboPort.open("/cer_teleop/gazebo:o");
    robotTargetPort.open("/cer_teleop/target_" + hand + ":o");
    robotStatePort.open("/cer_teleop/state_" + hand + ":i");
    robotCmdPort.open("/cer_teleop/cmd_" + hand + ":rpc");

}

void HandThread::threadRelease()
{
    ivel->stop();

    stopReaching();
    for (size_t i = 0; i < modes.size(); i++)
    {
        modes[i] = VOCAB_CM_POSITION;
    }

    imod->setControlModes(modes.getFirst());

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
            cur_x    = pose->subVector(0,2);
            cur_o    = pose->subVector(3,5);
            double n = norm(cur_o);
            cur_o   /= (n > 0.0 ? n : 1.0);
            cur_o.push_back(n);
        }

        getData();
        reachingHandler(button0, pos, rpy);
        handHandler(button1);
    }
    else
    {
        yError("No robot connected!");
    }
}
