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
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/FrameTransform.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IJoypadController.h>
#include "ParamParser.h"
#include "handThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

typedef vector<IVelocityControl*> IVelVector;
typedef vector<IControlMode*>     ICtrlModVec;
typedef vector<IEncoders*>        IEncVec;

class TeleOp: public RFModule
{
protected:
    enum FrameSource
    {
        VR_DEVICE,
        HAPTIC_DEVICE
    };

    HandThread         left{HandThread::left_hand};
    HandThread         right{HandThread::right_hand};
    HandThread*        hands[HandThread::hand_count];
    ParamParser        param_parser{"TeleOp Module"};
    string             mode;
    double             wrist_heave;
    double             gain;
    FrameSource        frameSource;
    string             rightHandFrame;
    string             leftHandFrame;
    string             rootHandFrame;
    Matrix             T;
    Node*              rosNode;
    PolyDriver         drvControlSource;
    PolyDriver         drvJoypad;
    IHapticDevice*     igeo{YARP_NULLPTR};
    IFrameTransform*   iTf;
    IJoypadController* iJoypad;
    map<int,string>    stateStr;
    Matrix             Tsim;
    int                ctrlMode;
    bool               absoluteRotation{false};

    /**********************************************************/

public:
    TeleOp(){}

    bool configureVrDevice(ResourceFinder& rf)
    {
        vector<pair<string, ParamParser::paramType> > param_v;
        param_v.push_back(std::make_pair("tfDevice",            ParamParser::TYPE_STRING));
        param_v.push_back(std::make_pair("tfLocal",             ParamParser::TYPE_STRING));
        param_v.push_back(std::make_pair("tfRemote",            ParamParser::TYPE_STRING));
        param_v.push_back(std::make_pair("tf_left_hand_frame",  ParamParser::TYPE_STRING));
        param_v.push_back(std::make_pair("tf_right_hand_frame", ParamParser::TYPE_STRING));
        param_v.push_back(std::make_pair("left_radius",         ParamParser::TYPE_DOUBLE));
        param_v.push_back(std::make_pair("right_radius",        ParamParser::TYPE_DOUBLE));
        param_v.push_back(std::make_pair("tf_root_frame",       ParamParser::TYPE_STRING));
        param_v.push_back(std::make_pair("joyDevice",           ParamParser::TYPE_STRING));
        param_v.push_back(std::make_pair("joyLocal",            ParamParser::TYPE_STRING));
        param_v.push_back(std::make_pair("joyRemote",           ParamParser::TYPE_STRING));
        param_v.push_back(std::make_pair("control_mode",        ParamParser::TYPE_STRING));

        if(!param_parser.parse(rf, param_v))
        {
            return false;
        }

        if(rf.find("control_mode").asString() == "velocity")
        {
            ctrlMode = VOCAB_CM_VELOCITY;
        }
        else if(rf.find("control_mode").asString() == "position")
        {
            ctrlMode = VOCAB_CM_POSITION_DIRECT;
        }
        else
        {
            yError() << "teleop: control mode not supported.. possible value are 'position' and 'velocity'";
            return false;
        }
        absoluteRotation = rf.check("absolute_rotation");

        Property tf_cfg, joy_cfg;
        tf_cfg.put("device",  rf.find("tfDevice").asString());
        tf_cfg.put("local",   rf.find("tfLocal").asString());
        tf_cfg.put("remote",  rf.find("tfRemote").asString());
        joy_cfg.put("device", rf.find("joyDevice").asString());
        joy_cfg.put("local",  rf.find("joyLocal").asString());
        joy_cfg.put("remote", rf.find("joyRemote").asString());

        leftHandFrame  = rf.find("tf_left_hand_frame").asString();
        rightHandFrame = rf.find("tf_right_hand_frame").asString();
        rootHandFrame  = rf.find("tf_root_frame").asString();

        if(!drvControlSource.open(tf_cfg))
        {
            yError() << "Teleoperation Module: unable to open the tf device";
            return false;
        }

        if (!drvControlSource.view(iTf))
        {
            yError() << "Teleoperation Module: dynamic_cast to IFrameTransform interface failed";
            return false;
        }

        if(!drvJoypad.open(joy_cfg))
        {
            yError() << "Teleoperation Module: unable to open the Joypad Device";
            return false;
        }

        if (!drvJoypad.view(iJoypad))
        {
            yError() << "Teleoperation Module: dynamic_cast to IJoypadController interface failed";
            return false;
        }

        unsigned int buttCount;
        if(iJoypad->getButtonCount(buttCount) && buttCount < 4)
        {
            yError() << "teleoperation via tf requires at least 4 button";
            return false;
        }

        unsigned int axisCount;
        if(iJoypad->getAxisCount(axisCount) && axisCount < 4)
        {
            yError() << "teleoperation via tf requires at least 4 axis";
            return false;
        }


        return true;
    }

    bool configureHaptic(ResourceFinder& rf)
    {
        ctrlMode = VOCAB_CM_VELOCITY;
        string device   = rf.check("device",   Value("geomagic")).asString();

        Property optGeo("(device hapticdeviceclient)");
        optGeo.put("remote", ("/" + device).c_str());
        optGeo.put("local", "/cer_teleop/geomagic");

        if (!drvControlSource.open(optGeo))
        {
            delete rosNode;
            return false;
        }

        if(!drvControlSource.view(igeo))
        {
            yError() << "dynamic_cast to IHapticDevice failed.";
        }

        igeo->setTransformation(SE3inv(T));

        return true;
    }

    bool configure(ResourceFinder& rf)
    {
        hands[HandThread::left_hand] = YARP_NULLPTR;
        hands[HandThread::right_hand] = YARP_NULLPTR;
        if(rf.check("frame_source") && rf.find("frame_source").isString())
        {
            string inputFS;

            inputFS = rf.find("frame_source").asString();
            if(inputFS == "VrDevice")
            {
                frameSource = VR_DEVICE;
            }
            else if(inputFS == "Haptic_Device")
            {
                frameSource = HAPTIC_DEVICE;
            }
            else
            {
                yError() << "frame_source parameter value not understood. valid value are transforms or Haptic_Device at the moment";
                return false;
            }
        }
        else
        {
            yError() << "frame_source parameter missing in configuration file";
            return false;
        }

        rosNode = new yarp::os::Node("/cer_teleop");

        //T =
        //  0-1 0 0
        //  0 0 1 0
        // -1 0 0 0
        //  0 0 0 1
        T      = zeros(4,4);
        T(0,1) = -1.0;
        T(1,2) = +1.0;
        T(2,0) = -1.0;
        T(3,3) = +1.0;

        switch(frameSource)
        {
        case VR_DEVICE:
            if(!configureVrDevice(rf))
            {
                return false;
            }
            hands[HandThread::left_hand]  = &left;
            hands[HandThread::right_hand] = &right;

            left.targetRadius = rf.find("left_radius").asFloat64();
            right.targetRadius = rf.find("right_radius").asFloat64();

            if(!left.openControlBoards(rf)  ||
               !right.openControlBoards(rf))
            {
                yError() << "teleop: something goes wrong...";
                return false;
            }

            if(!left.start() || !right.start())
            {
                yError() << "teleop: thread start fail";
                return false;
            }
            break;

        case HAPTIC_DEVICE:
            if(!configureHaptic(rf))
            {
                if(rf.check("arm-type"))
                {
                    if(rf.find("arm-type").asString() == "left")
                    {
                        hands[HandThread::left_hand]  = &left;
                        left.openControlBoards(rf);
                    }
                    else if(rf.find("arm-type").asString() == "right")
                    {
                        hands[HandThread::right_hand] = &right;
                        right.openControlBoards(rf);
                    }
                    else
                    {
                        hands[HandThread::right_hand] = &right;
                        right.openControlBoards(rf);
                        yError() << "arm-type parameter not understood.. possible values are 'left' or 'right'.. using right arm";
                    }
                }
                return false;
            }
            break;
        default:
            return false;
        }
        return true;
    }

    bool close()
    {
        if(igeo != YARP_NULLPTR)
        {
            igeo->setTransformation(eye(4, 4));
        }

        drvControlSource.close();

        left.stop();
        right.stop();

        left.threadRelease();
        right.threadRelease();

        delete rosNode;
        return true;
    }

    double getPeriod()
    {
        return 0.01;
    }

    bool updateModule()
    {
        //disclaimer: those static are read only constant value.. (so it's safe for them to be static)
        Matrix                  m(4, 4), m_gripper(4, 4);
        float                   button0, button1, button2;
        HandThread::CommandData data;
        static const string     enum2frameName[HandThread::hand_count] = {leftHandFrame, rightHandFrame};
        static HandThread*      enum2hands[HandThread::hand_count]     = {&left, &right};
        if (frameSource == HAPTIC_DEVICE)
        {
            Vector buttons, pos, rpy;
            igeo->getButtons(buttons);
            igeo->getPosition(pos);
            igeo->getOrientation(rpy);
            m = rpy2dcm(rpy);
            m.setSubcol(pos, 0, 3);

            data.pose             = m;
            data.button0          = buttons[0];
            data.button1          = buttons[1];
            data.controlMode      = VOCAB_CM_VELOCITY;
            data.singleButton     = true;
            data.targetDistance   = 0;
            data.simultMovRot     = false;
            data.absoluteRotation = false;
            (*enum2hands[0]).setData(data);
        }
        else if (frameSource == VR_DEVICE)
        {
            for(int i = 0; i < HandThread::hand_count; ++i)
            {
                string gripper = i ? "r_gripper" : "l_gripper";
                double axis0, axis1;
                bool reset = false;

                if(!iJoypad->getButton(!i * 4 + 0, button0))
                {
                    yWarning() << "unable to get buttons state";
                    reset = true;
                }

                if (!iJoypad->getButton(!i * 4 + 1, button2))
                {
                    yWarning() << "unable to get buttons state";
                    reset = true;
                }

                if(!iJoypad->getAxis(i, axis0) || !iJoypad->getAxis(i + 2, axis1))
                {
                    yWarning() << "unable to get buttons state";
                    reset = true;
                }

                if(!iTf->getTransform(enum2frameName[i], rootHandFrame, m))
                {
                    yWarning() << "teleop: unable to get transform between" << enum2frameName[i] << rootHandFrame;
                    reset = true;
                }

                if((*enum2hands[i]).targetRadius > 0 && !iTf->getTransform(enum2frameName[i], gripper, m_gripper))
                {
                    yWarning() << "teleop: unable to get transform between" << enum2frameName[i] << gripper;
                    reset = true;
                }

                if(reset)
                {
                    (*enum2hands[i]).setData(data);
                    return true;
                }

                data.pose               = m;
                data.button0            = button0 > 0.3;
                data.button1            = ctrlMode == VOCAB_CM_POSITION_DIRECT ? (axis0 + axis1)/2 : axis0 - axis1;
                data.button2            = button2;
                data.controlMode        = ctrlMode;
                data.singleButton       = false;
                data.simultMovRot       = true;
                data.absoluteRotation   = absoluteRotation;
                data.targetDistance     = sqrt(m_gripper[0][3] * m_gripper[0][3] +
                                               m_gripper[1][3] * m_gripper[1][3] +
                                               m_gripper[2][3] * m_gripper[2][3]);
                (*enum2hands[i]).setData(data);

                iTf->setTransform(enum2frameName[i]+"_target", "mobile_base_body_link",  (*enum2hands[i]).getMatrix());

                if(data.targetDistance < (*enum2hands[i]).targetRadius)
                {
                    //give a feedback to the user here
                }
            }
        }

        return true;
    }
};


/**********************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not found!");
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc, argv);

    TeleOp teleop;
    return teleop.runModule(rf);
}

