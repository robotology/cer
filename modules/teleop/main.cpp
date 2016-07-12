// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later.
 *
 */

#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <map>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <hapticdevice/IHapticDevice.h>

#include "ros_messages/visualization_msgs_Marker.h"
#include "ros_messages/visualization_msgs_MarkerArray.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace hapticdevice;


/**********************************************************/
class TeleOp: public RFModule
{
protected:
    Publisher<visualization_msgs_MarkerArray> rosPublisherPort;
    Node *rosNode;

    PolyDriver     drvGeomagic;
    IHapticDevice *igeo;

    PolyDriver         drvHand;
    IControlMode2     *imod;
    IVelocityControl2 *ivel;
    VectorOf<int>      modes;
    Vector             vels;
    
    BufferedPort<Bottle>   gazeboPort;
    BufferedPort<Property> robotTargetPort;
    BufferedPort<Vector>   robotStatePort;
    RpcClient              robotCmdPort;

    string arm_type;
    string mode;
    double wrist_heave;
    double gain;
    bool reachState;

    enum {
        idle,
        triggered,
        running
    };

    int s0,s1;
    int c0,c1;
    map<int,string> stateStr;

    Matrix Tsim;
    Vector cur_x,cur_o;
    Vector pos0,rpy0;
    Vector x0,o0;
    Vector fixedPosition;
    Vector fixedOrientation;

public:
    /**********************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("cer")).asString();
        string geomagic=rf.check("geomagic",Value("geomagic")).asString();
        arm_type=rf.check("arm-type",Value("right")).asString();
        gain=rf.check("gain",Value(1.5)).asDouble();        
        
        wrist_heave=0.02;        
        if (rf.check("wrist-heave"))
        {
            wrist_heave=rf.find("wrist-heave").asDouble();
            mode="full_pose+no_torso_no_heave";
        }
        else
            mode="full_pose+no_torso_heave";
        
        transform(arm_type.begin(),arm_type.end(),arm_type.begin(),::tolower);
        if ((arm_type!="left") && (arm_type!="right"))
        {
            yWarning("Unrecognized arm-type \"%s\"",arm_type.c_str());
            arm_type="right";
        }

        rosNode=new yarp::os::Node("/cer_teleop");
        if (!rosPublisherPort.topic("/cer_teleop_marker"))
        {
            yError("Unable to publish data on /cer_teleop_marker topic");
            yWarning("Check your yarp-ROS network configuration");
            return false;
        }

        Property optGeo("(device hapticdeviceclient)");
        optGeo.put("remote",("/"+geomagic).c_str());
        optGeo.put("local","/cer_teleop/geomagic");
        if (!drvGeomagic.open(optGeo))
        {
            delete rosNode;
            return false;
        }
        drvGeomagic.view(igeo);

        Property optHand("(device remote_controlboard)");
        optHand.put("remote",("/"+robot+"/"+arm_type+"_hand").c_str());
        optHand.put("local",("/cer_teleop/"+arm_type+"_hand").c_str());
        if (!drvHand.open(optHand))
        {
            drvGeomagic.close();
            delete rosNode;
            return false;
        }

        IEncoders *ienc;
        drvHand.view(ienc);
        drvHand.view(imod);
        drvHand.view(ivel);

        int nAxes;
        ienc->getAxes(&nAxes);
        for (int i=0; i<nAxes; i++)
        {
            modes.push_back(VOCAB_CM_VELOCITY);
            vels.push_back(40.0);
        }

        s0=s1=idle;
        c0=c1=0;
        reachState=false;
        
        stateStr[idle]="idle";
        stateStr[triggered]="triggered";
        stateStr[running]="running";

        Matrix T=zeros(4,4);
        T(0,1)=-1.0;
        T(1,2)=+1.0;
        T(2,0)=-1.0;
        T(3,3)=+1.0;
        igeo->setTransformation(SE3inv(T));
        
        pos0.resize(3,0.0);
        rpy0.resize(3,0.0);
        cur_x.resize(3,0.0);
        cur_o.resize(4,0.0);

        x0.resize(3,0.0);
        o0.resize(4,0.0);
        fixedPosition.resize(3,0.0);
        fixedOrientation.resize(4,0.0);

        gazeboPort.open("/cer_teleop/gazebo:o");
        robotTargetPort.open("/cer_teleop/target:o");
        robotStatePort.open("/cer_teleop/state:i");
        robotCmdPort.open("/cer_teleop/cmd:rpc");

        return true;
    }

    /**********************************************************/
    bool close()
    {
        igeo->setTransformation(eye(4,4));
        drvGeomagic.close();

        stopReaching();

        ivel->stop();
        for (size_t i=0; i<modes.size(); i++)
            modes[i]=VOCAB_CM_POSITION;
        imod->setControlModes(modes.getFirst());
        drvHand.close();

        gazeboPort.close();
        robotTargetPort.close();
        robotStatePort.close();
        robotCmdPort.close();

        delete rosNode;
        return true;
    }

    /**********************************************************/
    void stopReaching()
    {
        Bottle cmd,reply;
        cmd.addVocab(Vocab::encode("stop"));
        if (robotCmdPort.write(cmd,reply))
        {
            if (reply.get(0).asVocab()!=Vocab::encode("ack"))
                yError("Something went wrong while stopping");
        }
        else
            yError("Unable to communicate to controller");
    }

    /**********************************************************/
    void goToPose(const Vector &xd, const Vector &od)
    {
        Vector od_=od;
        od_*=od_[3]; od_.pop_back();
        
        Vector payLoad;
        payLoad.push_back(0.0); // uncontrolled torso-heave
        payLoad.push_back(wrist_heave);
        payLoad=cat(payLoad,xd);
        payLoad=cat(payLoad,od_);

        Bottle target;
        target.addList().read(payLoad);
        
        Bottle params;
        Bottle &bLoad=params.addList().addList();
        bLoad.addString("mode");
        bLoad.addString(mode);
        
        Property &prop=robotTargetPort.prepare(); prop.clear();
        prop.put("parameters",params.get(0));
        prop.put("target",target.get(0));
        robotTargetPort.write();
    }

    /**********************************************************/
    void updateRVIZ(const Vector &xd, const Vector &od)
    {
        double yarpTimeStamp = yarp::os::Time::now();
        uint64_t time;
        uint64_t nsec_part;
        uint64_t sec_part;
        TickTime ret;
        time = (uint64_t)(yarpTimeStamp * 1000000000UL);
        nsec_part = (time % 1000000000UL);
        sec_part = (time / 1000000000UL);
        if (sec_part > std::numeric_limits<unsigned int>::max())
        {
            yWarning() << "Timestamp exceeded the 64 bit representation, resetting it to 0";
            sec_part = 0;
        }

        visualization_msgs_MarkerArray& markerarray = rosPublisherPort.prepare();
        markerarray.markers.clear();
        visualization_msgs_Marker marker;
        marker.header.frame_id = "mobile_base_body_link";
        marker.header.stamp.sec = (yarp::os::NetUint32) sec_part;
        marker.header.stamp.nsec = (yarp::os::NetUint32) nsec_part;
        marker.ns = "cer-teleop_namespace";
        marker.type = visualization_msgs_Marker::SPHERE;
        marker.action = visualization_msgs_Marker::ADD;

        //center
        Vector q = dcm2quat(axis2dcm(od));
        marker.id = 1;
        marker.pose.position.x = xd[0];
        marker.pose.position.y = xd[1];
        marker.pose.position.z = xd[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markerarray.markers.push_back(marker);
        
        /*
        //x
        marker.id = 2;
        marker.pose.orientation.x = q[3];
        marker.pose.orientation.y = q[0];
        marker.pose.orientation.z = q[1];
        marker.pose.orientation.w = q[2];
        marker.scale.x = 0.5;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        markerarray.markers.push_back(marker);

        //y
        marker.id = 3;        
        marker.pose.orientation.x = q[3];
        marker.pose.orientation.y = q[0];
        marker.pose.orientation.z = q[1];
        marker.pose.orientation.w = q[2];
        marker.scale.x = 0.05;
        marker.scale.y = 0.5;
        marker.scale.z = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markerarray.markers.push_back(marker);

        //z
        marker.id = 4;        
        marker.pose.orientation.x = q[3];
        marker.pose.orientation.y = q[0];
        marker.pose.orientation.z = q[1];
        marker.pose.orientation.w = q[2];
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        markerarray.markers.push_back(marker);
        */
        rosPublisherPort.write();
    }

    /**********************************************************/
    void updateGazebo(const Vector &xd, const Vector &od)
    {
        if (gazeboPort.getOutputCount()>0)
        {
            Vector rpy=dcm2rpy(axis2dcm(od));
            Bottle &b=gazeboPort.prepare();
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

    /**********************************************************/
    void reachingHandler(const bool b, const Vector &pos,
                         const Vector &rpy)
    {
        if (b)
        {
            if (s0==idle)
                s0=triggered;
            else if (s0==triggered)
            {
                if (++c0*getPeriod()>0.5)
                {
                    pos0[0]=pos[0];
                    pos0[1]=pos[1];
                    pos0[2]=pos[2];

                    rpy0[0]=rpy[0];
                    rpy0[1]=rpy[1];
                    rpy0[2]=rpy[2];

                    x0=cur_x;
                    o0=cur_o;
                    s0=running;
                }
            }
            else
            {
                Vector xd(4,0.0);
                xd[0]=gain*(pos[0]-pos0[0]);
                xd[1]=gain*(pos[1]-pos0[1]);
                xd[2]=gain*(pos[2]-pos0[2]);
                xd[3]=1.0;

                Matrix H0=eye(4,4);
                H0(0,3)=x0[0];
                H0(1,3)=x0[1];
                H0(2,3)=x0[2];

                xd=H0*xd;
                xd.pop_back();

                Vector drpy(3);
                drpy[0]=gain*(rpy[0]-rpy0[0]);
                drpy[1]=gain*(rpy[1]-rpy0[1]);
                drpy[2]=gain*(rpy[2]-rpy0[2]);

                Vector ax(4,0.0),ay(4,0.0),az(4,0.0);
                ax[0]=1.0; ax[3]=drpy[2]*((arm_type=="right")?+1.0:-1.0);
                ay[1]=1.0; ay[3]=drpy[1]*((arm_type=="right")?-1.0:+1.0);
                az[2]=1.0; az[3]=drpy[0]*((arm_type=="right")?-1.0:+1.0);

                Matrix Rd=axis2dcm(o0)*axis2dcm(ax)*axis2dcm(ay)*axis2dcm(az);

                Vector od=dcm2axis(Rd);
                if (reachState)
                    goToPose(fixedPosition,od);
                else
                    goToPose(xd,fixedOrientation); 

                updateGazebo(xd,od);
                updateRVIZ(xd,od);
            }
        }
        else
        {
            if (s0==triggered)
            {
                reachState=!reachState;
                fixedPosition=cur_x;
                fixedOrientation=cur_o;

                yInfo("pose=%s; hand=%s;",
                      reachState?"fixed-position":"fixed-orientation",
                      vels[0]>0.0?"closing":"opening");
            }

            if (c0!=0)
            {
                stopReaching();
                updateGazebo(cur_x,cur_o);
                updateRVIZ(cur_x,cur_o);
            }

            s0=idle;
            c0=0;
        }
    }

    /**********************************************************/
    void handHandler(const bool b)
    {
        if (b)
        {
            if (s1==idle)
                s1=triggered;
            else if (s1==triggered)
            {
                if (++c1*getPeriod()>0.5)
                {
                    imod->setControlModes(modes.getFirst());
                    s1=running;
                }
            }
            else
                ivel->velocityMove(vels.data());
        }
        else
        {
            if (s1==triggered)
            {
                vels=-1.0*vels;

                yInfo("pose=%s; hand=%s;",
                      reachState?"fixed-position":"fixed-orientation",
                      vels[0]>0.0?"closing":"opening");
            }

            if (c1!=0)
                ivel->stop();

            s1=idle;
            c1=0;
        }
    }

    /**********************************************************/
    double getPeriod()
    {
        return 0.01;
    }

    /**********************************************************/
    bool updateModule()
    {
        if (robotStatePort.getInputCount()>0)
        {
            if (Vector *pose=robotStatePort.read(false))
            {
                cur_x=pose->subVector(0,2);
                cur_o=pose->subVector(3,5);
                double n=norm(cur_o);
                cur_o/=(n>0.0?n:1.0);
                cur_o.push_back(n);
            }

            Vector buttons,pos,rpy;
            igeo->getButtons(buttons);
            igeo->getPosition(pos);
            igeo->getOrientation(rpy);

            bool b0=(buttons[0]!=0.0);
            bool b1=(buttons[1]!=0.0);

            reachingHandler(b0,pos,rpy);
            handHandler(b1);
        }
        else
            yError("No robot connected!");

        return true;
    }
};


/**********************************************************/
int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not found!");
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    TeleOp teleop;
    return teleop.runModule(rf);
}

