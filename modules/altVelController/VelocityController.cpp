/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Alessandro Scalzo
 * email:  alessandro.scalzo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#define ONLINE
//#define JOYSTICK

#ifdef JOYSTICK
#include <SDL.h>
#endif

#include <time.h>
#include <string>

#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <cmath>

#include <R1Controller.h>

using namespace cer::kinematics_alt::r1;

class R1Driver
{
public:
    enum R1Part { TORSO, TORSO_TRIPOD, HEAD, LEFT_ARM, LEFT_TRIPOD, RIGHT_ARM, RIGHT_TRIPOD, LEFT_HAND, RIGHT_HAND, NUM_R1_PARTS };

    R1Driver(std::string robotName);

    ~R1Driver() { close(); }

    bool open();
    void close();

    void getPos(cer::robot_model::Matrix &q);

    void setPos(cer::robot_model::Matrix &q);
    void setVel(cer::robot_model::Matrix &qdot);

    void setVelHands(double vlf, double vlt, double vrf, double vrt)
    {
        double vel[8];

        vel[0] = vlf; vel[1] = vlt;

        pVelCtrl[ LEFT_HAND]->velocityMove(vel);
        
        vel[0] = vrf; vel[1] = vrt;
        
        pVelCtrl[RIGHT_HAND]->velocityMove(vel);
    }

    int getNumOfJoints(int part)
    {
        return mNumJoints[part];
    }

    void modePosition(int part)
    {
        for (int j = 0; j<mNumJoints[part]; ++j) pCmdCtrlMode[part]->setControlMode(j, VOCAB_CM_POSITION);
    }

    void modeDirect(int part)
    {
        for (int j = 0; j < mNumJoints[part]; ++j) pCmdCtrlMode[part]->setControlMode(j, VOCAB_CM_POSITION_DIRECT);
    }

    void modeVelocity(int part)
    {
        for (int j = 0; j<mNumJoints[part]; ++j) pCmdCtrlMode[part]->setControlMode(j, VOCAB_CM_VELOCITY);
    }

    static const char *R1PartName[NUM_R1_PARTS];

protected:
    yarp::dev::PolyDriver* openDriver(std::string part);

    yarp::dev::PolyDriver* mDriver[NUM_R1_PARTS];

    int mNumJoints[NUM_R1_PARTS];

    yarp::dev::IEncoders          *pEncFbk[NUM_R1_PARTS];

    yarp::dev::IPositionControl   *pPosCtrl[NUM_R1_PARTS];
    yarp::dev::IVelocityControl   *pVelCtrl[NUM_R1_PARTS];
    yarp::dev::IPositionDirect    *pDirCtrl[NUM_R1_PARTS];

    yarp::dev::IControlMode       *pCmdCtrlMode[NUM_R1_PARTS];

    std::string mRobotName;
};

const char* R1Driver::R1PartName[NUM_R1_PARTS] = { "torso","torso_tripod","head","left_arm","left_wrist_tripod","right_arm","right_wrist_tripod","left_hand","right_hand" };

R1Driver::R1Driver(std::string robotName) : mRobotName(robotName)
{
    for (int part = TORSO; part < NUM_R1_PARTS; ++part)
    {
        pEncFbk[part] = NULL;
        
        pPosCtrl[part] = NULL;
        pVelCtrl[part] = NULL;
        pDirCtrl[part] = NULL;

        pCmdCtrlMode[part] = NULL;
    }
}

bool R1Driver::open()
{
    double ref_vel[NUM_R1_PARTS][8];
    double ref_acc[NUM_R1_PARTS][8];

    for (int part=TORSO; part<NUM_R1_PARTS; ++part)
    {
        for (int j=0; j<8; ++j)
        {
            ref_vel[part][j] = 15.0;
            ref_acc[part][j] = 100.0;
        }
    }

    ref_vel[TORSO][0] = 0.01;
    ref_acc[TORSO][0] = 0.05;

    ref_vel[TORSO_TRIPOD][0] = ref_vel[TORSO_TRIPOD][1] = ref_vel[TORSO_TRIPOD][2] = 0.01;
    ref_acc[TORSO_TRIPOD][0] = ref_acc[TORSO_TRIPOD][1] = ref_acc[TORSO_TRIPOD][2] = 0.05;

    ref_vel[LEFT_TRIPOD][0] = ref_vel[LEFT_TRIPOD][1] = ref_vel[LEFT_TRIPOD][2] = 0.005;
    ref_acc[LEFT_TRIPOD][0] = ref_acc[LEFT_TRIPOD][1] = ref_acc[LEFT_TRIPOD][2] = 0.025;

    ref_vel[RIGHT_TRIPOD][0] = ref_vel[RIGHT_TRIPOD][1] = ref_vel[RIGHT_TRIPOD][2] = 0.005;
    ref_acc[RIGHT_TRIPOD][0] = ref_acc[RIGHT_TRIPOD][1] = ref_acc[RIGHT_TRIPOD][2] = 0.025;

    for (int part = TORSO; part<NUM_R1_PARTS; ++part)
    {
        mNumJoints[part] = 0;

        mDriver[part] = openDriver(R1PartName[part]);

        if (mDriver[part])
        {
            mDriver[part]->view(pEncFbk[part]);

            if (!pEncFbk[part]) return false;

            mDriver[part]->view(pPosCtrl[part]);
            mDriver[part]->view(pVelCtrl[part]);
            mDriver[part]->view(pDirCtrl[part]);

            mDriver[part]->view(pCmdCtrlMode[part]);

            if (pEncFbk[part]) pEncFbk[part]->getAxes(&mNumJoints[part]);

            pPosCtrl[part]->setRefSpeeds(ref_vel[part]);
            pPosCtrl[part]->setRefAccelerations(ref_acc[part]);
            pVelCtrl[part]->setRefAccelerations(ref_acc[part]);

            modeVelocity(part);
        }
        else
        {
            return false;
        }
    }

    return true;
}

void R1Driver::close()
{
    for (int part = TORSO; part<NUM_R1_PARTS; ++part)
    {
        if (pCmdCtrlMode[part]) modePosition(part);
    }
}

yarp::dev::PolyDriver* R1Driver::openDriver(std::string part)
{
    yarp::dev::PolyDriver *pDriver = NULL;

    yarp::os::Property options;
    options.put("robot", mRobotName.c_str());
    options.put("device", "remote_controlboard");
    options.put("local", (std::string("/R1ctrl/") + part).c_str());
    options.put("remote", (mRobotName + "/" + part).c_str());

    pDriver = new yarp::dev::PolyDriver(options);

    if (!pDriver) return NULL;

    if (!pDriver->isValid())
    {
        pDriver->close();
        delete pDriver;
        pDriver = NULL;
    }

    return pDriver;
}

void R1Driver::getPos(cer::robot_model::Matrix &q)
{
    double enc[8];

    pEncFbk[TORSO_TRIPOD]->getEncoders(enc);

    q(0) = enc[0]; q(1) = enc[1]; q(2) = enc[2];

    pEncFbk[TORSO]->getEncoders(enc);

    q(3) = enc[3];

    pEncFbk[LEFT_ARM]->getEncoders(enc);

    q(4) = enc[0]; q(5) = enc[1]; q(6) = enc[2]; q(7) = enc[3]; q(8) = enc[4];

    pEncFbk[LEFT_TRIPOD]->getEncoders(enc);

    q(9) = enc[0]; q(10) = enc[1]; q(11) = enc[2];

    pEncFbk[RIGHT_ARM]->getEncoders(enc);

    q(12) = enc[0]; q(13) = enc[1]; q(14) = enc[2]; q(15) = enc[3]; q(16) = enc[4]; 

    pEncFbk[RIGHT_TRIPOD]->getEncoders(enc);

    q(17) = enc[0]; q(18) = enc[1]; q(19) = enc[2];

    pEncFbk[HEAD]->getEncoders(enc);

    q(20) = enc[0]; q(21) = enc[1];
}

void R1Driver::setPos(cer::robot_model::Matrix &q)
{
    double pos[8];

    pos[0] = q(0); pos[1] = q(1); pos[2] = q(2);

    pPosCtrl[TORSO_TRIPOD]->positionMove(pos);

    pos[0] = q(3);

    static const int torso_yaw = 3;

    pPosCtrl[TORSO]->positionMove(1,&torso_yaw,pos);

    static const int left_upper[] = { 0,1,2,3,4 };

    pos[0] = q(4); pos[1] = q(5); pos[2] = q(6); pos[3] = q(7); pos[4] = q(8);

    pPosCtrl[LEFT_ARM]->positionMove(5,left_upper,pos);

    pos[0] = q(9); pos[1] = q(10); pos[2] = q(11);

    pPosCtrl[LEFT_TRIPOD]->positionMove(pos);

    static const int right_upper[] = { 0,1,2,3,4 };

    pos[0] = q(12); pos[1] = q(13); pos[2] = q(14); pos[3] = q(15); pos[4] = q(16);

    pPosCtrl[RIGHT_ARM]->positionMove(5,right_upper,pos);

    pos[0] = q(17); pos[1] = q(18); pos[2] = q(19);

    pPosCtrl[RIGHT_TRIPOD]->positionMove(pos);

    pos[0] = q(20); pos[1] = q(21);

    pPosCtrl[HEAD]->positionMove(pos);
}

void R1Driver::setVel(cer::robot_model::Matrix &qdot)
{
    double vel[8];

    vel[0] = qdot(0); vel[1] = qdot(1); vel[2] = qdot(2);

    pVelCtrl[TORSO_TRIPOD]->velocityMove(vel);

    vel[0] = qdot(3);

    static const int torso_yaw = 3;

    pVelCtrl[TORSO]->velocityMove(1, &torso_yaw, vel);

    static const int left_upper[] = { 0,1,2,3,4 };

    vel[0] = qdot(4); vel[1] = qdot(5); vel[2] = qdot(6); vel[3] = qdot(7); vel[4] = qdot(8);

    pVelCtrl[LEFT_ARM]->velocityMove(5, left_upper, vel);

    vel[0] = qdot(9); vel[1] = qdot(10); vel[2] = qdot(11);

    pVelCtrl[LEFT_TRIPOD]->velocityMove(vel);

    static const int right_upper[] = { 0,1,2,3,4 };

    vel[0] = qdot(12); vel[1] = qdot(13); vel[2] = qdot(14); vel[3] = qdot(15); vel[4] = qdot(16);

    pVelCtrl[RIGHT_ARM]->velocityMove(5, right_upper, vel);

    vel[0] = qdot(17); vel[1] = qdot(18); vel[2] = qdot(19);

    pVelCtrl[RIGHT_TRIPOD]->velocityMove(vel);

    vel[0] = qdot(20); vel[1] = qdot(21);

    pVelCtrl[HEAD]->velocityMove(vel);
}

class R1ControlModule : public yarp::os::RateThread
{
public:
    R1ControlModule();
    ~R1ControlModule()
    {
        if (r1Ctrl) delete r1Ctrl;
        if (r1Model) delete r1Ctrl;
    }

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();
    virtual void onStop();

protected:
    void sendConfig(cer::robot_model::Matrix &q);
    void sendCOM(const std::string& name,int R,int G,int B,cer::robot_model::Vec3& P,double size,double alpha);
    void sendTarget(const char* name,int R,int G,int B,double x,double y,double z,double size,double alpha,double rx=0.0,double ry=0.0,double rz=0.0);
    void sendCover(const std::string& name, double x, double y, double z, double size);

    R1Model *r1Model;
    R1Controller *r1Ctrl;

#ifdef JOYSTICK
    SDL_Joystick *mStick;
    int mNumJoyAxis;
    int mNumJoyButt;
    int mNumJoyHats;
#endif

    Transform TargetL;
    Transform TargetR;

    Transform HandL;
    Transform HandR;

    R1Driver mDriver;

    FILE *dumpin;

#ifndef ONLINE
    double qdot_del[22][256];
    double qfbk_del[22][256];
#endif

    cer::robot_model::Matrix qfbk;

    yarp::os::BufferedPort<yarp::sig::Vector> portEncBase;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncTorso;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncHead;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncLeftArm;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncRightArm;

    yarp::os::BufferedPort<yarp::os::Bottle> portObjects;
};


R1ControlModule::R1ControlModule() : RateThread(int(PERIOD*1000.0)), qfbk(22) ,mDriver("/cer")
{
    r1Model = new R1Model();
    r1Ctrl = new R1Controller(r1Model);
}

bool R1ControlModule::threadInit()
{
#ifdef ONLINE
    if (!mDriver.open()) return false;
#endif

#ifdef JOYSTICK
    //SDL_INIT_TIMER SDL_INIT_HAPTIC SDL_INIT_GAMECONTROLLER 

    // SDL
    if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_NOPARACHUTE) == -1) return false;

    if (SDL_NumJoysticks()<1) return false;

    mStick = SDL_JoystickOpen(0);

    if (!mStick) return false;

    SDL_JoystickEventState(SDL_IGNORE);

    mNumJoyAxis = SDL_JoystickNumAxes(mStick);
    mNumJoyButt = SDL_JoystickNumButtons(mStick);
    mNumJoyHats = SDL_JoystickNumHats(mStick);

    // end SDL
#endif

    portEncBase.open("/CERControl/base:o");
    portEncTorso.open("/CERControl/torso:o");
    portEncHead.open("/CERControl/head:o");
    portEncLeftArm.open("/CERControl/left_arm:o");
    portEncRightArm.open("/CERControl/right_arm:o");
    portObjects.open("/CERControl/objects:o");

    yarp::os::Network::connect("/CERControl/base:o","/CERGui/base:i");
    yarp::os::Network::connect("/CERControl/torso:o","/CERGui/torso:i");
    yarp::os::Network::connect("/CERControl/head:o","/CERGui/head:i");
    yarp::os::Network::connect("/CERControl/left_arm:o","/CERGui/left_arm:i");
    yarp::os::Network::connect("/CERControl/right_arm:o","/CERGui/right_arm:i");
    yarp::os::Network::connect("/CERControl/objects:o","/CERGui/objects:i");

    yarp::os::Bottle& bot=portObjects.prepare();
    bot.clear();
    bot.addString("reset");
    portObjects.write();

    srand((unsigned)time(NULL));

#ifdef ONLINE
    mDriver.getPos(qfbk);
    r1Model->calcConfig(qfbk);
#else
    qfbk = r1Ctrl->getZeroConfig();

    for (int j = 0; j < 22; ++j)
    {
        for (int t = 0; t < 256; ++t)
        {
            qdot_del[j][t] = 0.0;
            qfbk_del[j][t] = qfbk(j);
        }
    }
#endif

    //r1Model->handL(HandL.Pj().x, HandL.Pj().y, HandL.Pj().z, ArpyL.x, ArpyL.y, ArpyL.z);
    //r1Model->handR(HandR.Pj().x, HandR.Pj().y, HandR.Pj().z, ArpyR.x, ArpyR.y, ArpyR.z);

    HandL = r1Model->getHandTransformL();
    HandR = r1Model->getHandTransformR();

    TargetL = HandL;
    TargetR = HandR;

    return true;
}

void R1ControlModule::onStop()
{
}

void R1ControlModule::threadRelease()
{

#ifdef JOYSTICK
    SDL_JoystickClose(mStick);
    SDL_Quit();
#endif

    portEncBase.interrupt();
    portEncTorso.interrupt();
    portEncHead.interrupt();
    portEncLeftArm.interrupt();
    portEncRightArm.interrupt();
    portObjects.interrupt();

    portEncBase.close();
    portEncTorso.close();
    portEncHead.close();
    portEncLeftArm.close();
    portEncRightArm.close();
    portObjects.close();
}

void R1ControlModule::run()
{
    static bool L_active = true;
    static bool R_active = true;

    static double El = 0.03;
    static double Er = 0.03;
    static double Et = 0.05;

    Vec3 VjoyL, VjoyR;
    Vec3 WjoyL, WjoyR;

#ifdef JOYSTICK

    SDL_JoystickUpdate();
    
    //////////////////////////////////////////////////////////////
    // enable left/right side
    unsigned char L_enable_new = SDL_JoystickGetButton(mStick, 8);
    unsigned char R_enable_new = SDL_JoystickGetButton(mStick, 9);

    static unsigned char L_enable_old = L_enable_new;
    static unsigned char R_enable_old = R_enable_new;

    bool enable_change = false;

    if (L_enable_new && !L_enable_old)
    {
        L_active = !L_active;
        enable_change = true;
    }

    if (R_enable_new && !R_enable_old)
    {
        R_active = !R_active;
        enable_change = true;
    }

    if (enable_change)
    {
        printf("L hand = %s * R hand = %s\n", L_active ? "ENABLED" : "DISABLED", R_active ? "ENABLED" : "DISABLED");
    }

    L_enable_old = L_enable_new;
    R_enable_old = R_enable_new;
    ///////////////////////////////////////////////////////////

    int sticklx = SDL_JoystickGetAxis(mStick, 1);
    int stickly = SDL_JoystickGetAxis(mStick, 0);
    int hatl = SDL_JoystickGetHat(mStick, 0);

    // trigger not pressed
    if (SDL_JoystickGetAxis(mStick, 5) > -16)
    {
        if (abs(sticklx) > 16) VjoyL.x = -sticklx * 0.1 / 32768.0;
        if (abs(stickly) > 16) VjoyL.y = -stickly * 0.1 / 32768.0;

        if (SDL_JoystickGetButton(mStick, 4)) VjoyL.z = 0.05;
        if (SDL_JoystickGetButton(mStick, 6)) VjoyL.z = -0.05;

        if (hatl & 15)
        {
            if (hatl & 4) El -= 0.01*PERIOD;
            if (hatl & 1) El += 0.01*PERIOD;
            if (hatl & 8) Et -= 0.01*PERIOD;
            if (hatl & 2) Et += 0.01*PERIOD;

            if (El < 0.02) El = 0.02;
            if (El > 0.12) El = 0.12;
            if (Et < 0.04) Et = 0.04;
            if (Et > 0.16) Et = 0.16;

            r1Ctrl->setExtensions(El, Er, Et);
        }
    }
    else // trigger pressed
    {
        if (abs(sticklx) > 16) WjoyL.y = -sticklx * 45.0 / 32768.0;
        if (abs(stickly) > 16) WjoyL.x = stickly * 45.0 / 32768.0;

        if (hatl & 8) WjoyL.z = 30.0;
        if (hatl & 2) WjoyL.z = -30.0;
    }

    //bool Vlmove = VjoyL.mod2() != 0.0;
    //bool Wlmove = WjoyL.mod2() != 0.0;

    /////////////////////////////////////////

    int stickrx = SDL_JoystickGetAxis(mStick, 3);
    int stickry = SDL_JoystickGetAxis(mStick, 2);

    if (SDL_JoystickGetAxis(mStick, 4) > -16)
    {
        if (abs(stickrx) > 16) VjoyR.x = -stickrx * 0.1 / 32768.0;
        if (abs(stickry) > 16) VjoyR.y = -stickry * 0.1 / 32768.0;

        if (SDL_JoystickGetButton(mStick, 5)) VjoyR.z = 0.05;
        if (SDL_JoystickGetButton(mStick, 7)) VjoyR.z = -0.05;

        if (SDL_JoystickGetButton(mStick, 0))
        {
            Et -= 0.01*PERIOD;
            if (Et < 0.04) Et = 0.04;
            r1Ctrl->setExtensions(El, Er, Et);
        }

        if (SDL_JoystickGetButton(mStick, 2))
        {
            Et += 0.01*PERIOD;
            if (Et > 0.16) Et = 0.16;
            r1Ctrl->setExtensions(El, Er, Et);
        }

        if (SDL_JoystickGetButton(mStick, 1))
        {
            Er -= 0.01*PERIOD;
            if (Er < 0.02) Er = 0.02;
            r1Ctrl->setExtensions(El, Er, Et);
        }

        if (SDL_JoystickGetButton(mStick, 3))
        {
            Er += 0.01*PERIOD;
            if (Er > 0.12) Er = 0.12;
            r1Ctrl->setExtensions(El, Er, Et);
        }
    }
    else
    {
        if (abs(stickrx) > 16) WjoyR.y =  -stickrx * 45.0 / 32768.0;
        if (abs(stickry) > 16) WjoyR.x = stickry * 45.0 / 32768.0;

        if (SDL_JoystickGetButton(mStick, 0)) WjoyR.z = -30.0;
        if (SDL_JoystickGetButton(mStick, 2)) WjoyR.z = 30.0;
    }

    bool Vrmove = VjoyR.mod2() != 0.0;
    bool Wrmove = WjoyR.mod2() != 0.0;

#endif

#ifdef ONLINE
    mDriver.getPos(qfbk);
#endif

    static cer::robot_model::Matrix qphantom = qfbk;

    static cer::robot_model::Matrix qdot(22);

    Vec3 VstarL = 5.0*(TargetL.Pj() - HandL.Pj());
    Vec3 VstarR = 5.0*(TargetR.Pj() - HandR.Pj());

    Vec3 WstarL = 5.0*(TargetL.Rj().quaternion() * HandL.Rj().quaternion().conj()).V;
    Vec3 WstarR = 5.0*(TargetR.Rj().quaternion() * HandR.Rj().quaternion().conj()).V;

    //VjoyL.print();
    //VjoyR.print();
    //WjoyL.print();
    //WjoyR.print();

    //VjoyL.clear();
    //VjoyR.clear();
    //WjoyL.clear();
    //WjoyR.clear();

    double vl3[] = { VjoyL.x, VjoyL.y, VjoyL.z };
    double vr3[] = { VjoyR.x, VjoyR.y, VjoyR.z };
    double wl3[] = { WjoyL.x, WjoyL.y, WjoyL.z };
    double wr3[] = { WjoyR.x, WjoyR.y, WjoyR.z };

    Vec3 ArpyL = TargetL.Rj().rpy();
    Vec3 ArpyR = TargetR.Rj().rpy();

    if (L_active && R_active)
    {
        r1Ctrl->velControl(qphantom, qdot, vl3, wl3, vr3, wr3);

        sendTarget("targetL", 255, 64, 0, TargetL.Pj().x, TargetL.Pj().y, TargetL.Pj().z, 16.0, 0.666, ArpyL.x, ArpyL.y, ArpyL.z);
        sendTarget("targetR", 0, 64, 255, TargetR.Pj().x, TargetR.Pj().y, TargetR.Pj().z, 16.0, 0.666, ArpyR.x, ArpyR.y, ArpyR.z);
    }
    else if (L_active)
    {
        r1Ctrl->velControl(qphantom, qdot, vl3, wl3, NULL, NULL);

        TargetR = HandR;

        sendTarget("targetL", 0, 64, 255, TargetL.Pj().x, TargetL.Pj().y, TargetL.Pj().z, 16.0, 0.666, ArpyL.x, ArpyL.y, ArpyL.z);
        sendTarget("targetR", 192, 192, 192, TargetR.Pj().x, TargetR.Pj().y, TargetR.Pj().z, 16.0, 0.5, ArpyR.x, ArpyR.y, ArpyR.z);
    }
    else if (R_active)
    {
        r1Ctrl->velControl(qphantom, qdot, NULL, NULL, vr3, wr3);

        TargetL = HandL;

        sendTarget("targetL", 192, 192, 192, TargetL.Pj().x, TargetL.Pj().y, TargetL.Pj().z, 16.0, 0.5, ArpyL.x, ArpyL.y, ArpyL.z);
        sendTarget("targetR", 0, 64, 255, TargetR.Pj().x, TargetR.Pj().y, TargetR.Pj().z, 16.0, 0.666, ArpyR.x, ArpyR.y, ArpyR.z);
    }
    else
    {
        r1Ctrl->velControl(qphantom, qdot, NULL, NULL, NULL, NULL);
    
        TargetL = HandL;
        TargetR = HandR;

        sendTarget("targetL", 192, 192, 192, TargetL.Pj().x, TargetL.Pj().y, TargetL.Pj().z, 16.0, 0.5, ArpyL.x, ArpyL.y, ArpyL.z);
        sendTarget("targetR", 192, 192, 192, TargetR.Pj().x, TargetR.Pj().y, TargetR.Pj().z, 16.0, 0.5, ArpyR.x, ArpyR.y, ArpyR.z);
    }

    static int TAU = 1;
    static double BETA = 0.1 / double(TAU);
    static double ALFA = 1.0 - BETA;

#ifdef ONLINE

    qdot(20) = qdot(21) = 0.0;

    mDriver.setVel(qdot);

    //mDriver.setVelHands(0.7*VhL,VhL,0.7*VhR,VhR);

    for (int j = 0; j < 22; ++j)
    {
        qphantom(j) += qdot(j)*PERIOD;
        qphantom(j) = ALFA*qphantom(j) + BETA*qfbk(j);
    }

    sendConfig(qfbk);

#else

    static int index = 0;

    for (int j = 0; j < 22; ++j)
    {
        qphantom(j) += qdot(j)*PERIOD;
        qphantom(j) = ALFA*qphantom(j) + BETA*qfbk_del[j][index];
    }

    for (int j = 0; j < 22; ++j)
    {
        qdot_del[j][index] = qdot(j);   
        qfbk_del[j][index] = qfbk(j);
    }

    index = (index + 1) % TAU;

    for (int j = 0; j < 22; ++j)
    {
        qfbk(j) += qdot_del[j][index] * PERIOD;
    }

    sendConfig(qphantom);

#endif

    ///////////////////////////////////////////////
    cer::robot_model::Vec3 COM = r1Model->getCOM();
    COM.z = -0.160;
    std::string G = "G";
    sendCOM(G, 192, 192, 0, COM, 16.0, 1.0);

    {
        std::string name;
        double x, y, z, size;

        int N = r1Model->getNSpheres();

        for (int n = 0; n < N; ++n)
        {
            r1Model->getSphere(n, x, y, z, size, name);
            sendCover(name, x, y, z, size);
        }
    }
}

/////////////////////////////////////////
/*
Quaternion QtargetL=Rl.quaternion();
Quaternion QtargetR=Rr.quaternion();

static const double KV = 20.0;
static const double KW = 10.0;

Vec3 VstarL = KV*(mPl - mHand[L]->Toj.Pj());
Vec3 VstarR = KV*(mPr - mHand[R]->Toj.Pj());

Vec3 WstarL = KW*(QtargetL*mHand[L]->Toj.Rj().quaternion().conj()).V;
Vec3 WstarR = KW*(QtargetR*mHand[R]->Toj.Rj().quaternion().conj()).V;
*/
/////////////////////////////////////////


void R1ControlModule::sendConfig(cer::robot_model::Matrix &q)
{
    if (portEncBase.getOutputCount()>0)
    {
        yarp::sig::Vector& enc=portEncBase.prepare();
        enc.clear();

        //Vec3 P=1000.*mRobot.getT().Pj();
        //Vec3 R=mRobot.getT().Rj().rpy();

        //enc.push_back(R.x);
        //enc.push_back(R.y);
        //enc.push_back(R.z);

        //enc.push_back(P.x);
        //enc.push_back(P.y);
        //enc.push_back(P.z);

        enc.push_back(0.0);
        enc.push_back(0.0);
        enc.push_back(0.0);
        enc.push_back(0.0);
        enc.push_back(0.0);
        enc.push_back(0.0);

        portEncBase.write();
    }

    if (portEncTorso.getOutputCount()>0)
    {
        yarp::sig::Vector& enc=portEncTorso.prepare();
        enc.clear();
        enc.push_back(360.0+1000.0*q(0));
        enc.push_back(360.0+1000.0*q(1));
        enc.push_back(360.0+1000.0*q(2));
        enc.push_back(q(3));
        portEncTorso.write();
    }

    if (portEncLeftArm.getOutputCount()>0)
    {
        yarp::sig::Vector& enc=portEncLeftArm.prepare();
        enc.clear();
        for (int i=4; i<9; ++i)  enc.push_back(q(i));
        for (int i=9; i<12; ++i) enc.push_back(220.0+1000.0*q(i));
        portEncLeftArm.write();
    }

    if (portEncRightArm.getOutputCount()>0)
    {
        yarp::sig::Vector& enc=portEncRightArm.prepare();
        enc.clear();
        for (int i=12; i<17; ++i) enc.push_back(q(i));
        for (int i=17; i<20; ++i) enc.push_back(220.0+1000.0*q(i));
        portEncRightArm.write();
    }

    if (portEncHead.getOutputCount()>0)
    {
        yarp::sig::Vector& enc=portEncHead.prepare();
        enc.clear();
        for (int i=20; i<22; ++i) enc.push_back(q(i));
        portEncHead.write();
    }
}

void R1ControlModule::sendCOM(const std::string& name, int R, int G, int B, cer::robot_model::Vec3& P, double size, double alpha)
{
    yarp::os::Bottle& botR=portObjects.prepare();
    botR.clear();
    botR.addString("object_with_label");
    botR.addString(name); botR.addString(name);
    botR.addDouble(size);
    botR.addDouble(0.0);
    botR.addDouble(0.0);
    botR.addDouble(P.x*1000.0);
    botR.addDouble(P.y*1000.0);
    botR.addDouble(P.z*1000.0);

    botR.addDouble(0.0); botR.addDouble(0.0); botR.addDouble(0.0);
    
    botR.addInt(R); botR.addInt(G); botR.addInt(B);
    botR.addDouble(alpha);
    //botR.addString("WORLD");
    portObjects.writeStrict();
}

void R1ControlModule::sendTarget(const char* name, int R, int G, int B, double x, double y, double z, double size, double alpha, double rx, double ry, double rz)
{
    yarp::os::Bottle& botR=portObjects.prepare();
    botR.clear();
    botR.addString("object"); botR.addString(name);
    botR.addDouble(size);
    botR.addDouble(0.0);
    botR.addDouble(0.0);
    botR.addDouble(x*1000.0);
    botR.addDouble(y*1000.0);
    botR.addDouble(z*1000.0);

    botR.addDouble(rx); 
    botR.addDouble(ry); 
    botR.addDouble(rz);

    botR.addInt(R); botR.addInt(G); botR.addInt(B);
    botR.addDouble(alpha);
    //botR.addString("WORLD");
    portObjects.writeStrict();
}

void R1ControlModule::sendCover(const std::string& name, double x, double y, double z, double size)
{
    yarp::os::Bottle& botR = portObjects.prepare();
    botR.clear();
    botR.addString("object"); botR.addString(name);
    botR.addDouble(size*1000.0);
    botR.addDouble(0.0);
    botR.addDouble(0.0);
    botR.addDouble(x*1000.0);
    botR.addDouble(y*1000.0);
    botR.addDouble(z*1000.0);

    botR.addDouble(360.0);
    botR.addDouble(360.0);
    botR.addDouble(360.0);

    botR.addInt(255); botR.addInt(255); botR.addInt(255);
    botR.addDouble(1.0);
    //botR.addString("WORLD");
    portObjects.writeStrict();
}

class CerTestModule : public yarp::os::RFModule
{
protected:
    R1ControlModule *mRobotThread;
    yarp::os::Port mHandlerPort;

public:
    CerTestModule()
    {
        mRobotThread=NULL;
    }

    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        mRobotThread = new R1ControlModule();

        if (!mRobotThread->start())
        {
            delete mRobotThread;
            return false;
        }

        mHandlerPort.open("/icubwalk");
        attach(mHandlerPort);

        return true;
    }

    virtual bool interruptModule()
    {
        mHandlerPort.interrupt();

        return true;
    }

    virtual bool close()
    {
        mHandlerPort.interrupt();
        mHandlerPort.close();

        mRobotThread->stop();
        delete mRobotThread;
        mRobotThread=NULL;

        return true;
    }

    virtual double getPeriod()
    {
        return 1.0;
    }

    virtual bool updateModule()
    {
        if (isStopping())
        {
            mRobotThread->stop();
            return false;
        }

        return true;
    }

    virtual bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply)
    {
        if (command.get(0).asString()=="quit") return false;

        return true;
    }
};

////////////////////////////////

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr,"ERROR: check Yarp network.\n");
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("iCubWalk.ini");
    rf.setDefaultContext("iCubWalk");
    rf.configure(argc,argv);

    CerTestModule CER;

    return CER.runModule(rf);
}

//sendCOM(std::string("LUarm"),192,192,0,mRobot.G[3],30.0,0.5);
//sendCOM(std::string("LLarm"),192,192,0,mRobot.G[4],30.0,0.5);
//sendCOM(std::string("Lhand"),192,192,0,mRobot.G[5],30.0,0.5);

//sendCOM(std::string("base"),192,192,0,mRobot.G[0],30.0,0.5);
//sendCOM(std::string("body"),192,192,0,mRobot.G[1],30.0,0.5);
//sendCOM(std::string("head"),192,192,0,mRobot.G[2],30.0,0.5);

//sendCOM(std::string("RUarm"),192,192,0,mRobot.G[6],30.0,0.5);
//sendCOM(std::string("RLarm"),192,192,0,mRobot.G[7],30.0,0.5);
//sendCOM(std::string("Rhand"),192,192,0,mRobot.G[8],30.0,0.5);

// reaching tests
#if 0
static double alfa = -135.0;
static double radius = 0.3;
static double dalfa = 5.0;
static double dradius = 0.05;

static bool newtarget = false;

static double timeout = 15.0;

if (newtarget)
{
    timeout = 1.5;

    //alfa += dalfa*PERIOD;

    alfa += dalfa;

    if (alfa > 45.0)
    {
        alfa = 45.0;
        dalfa = -dalfa;
        radius += dradius;
    }

    if (alfa < -135.0)
    {
        alfa = -135.0;
        dalfa = -dalfa;
        radius += dradius;
    }
}

//static FILE *dump = fopen("70_cm_H_hand.txt", "w");

if (radius > 1.0)
{
    //fclose(dump);
    askToStop();
}

TargetL.Pj().x = 0.044 + radius*cos(DEG2RAD*alfa);
TargetL.Pj().y = -radius*sin(DEG2RAD*alfa);
TargetL.Pj().z = 0.74;

TargetR.Pj().x = 0.044 + radius*cos(DEG2RAD*alfa);
TargetR.Pj().y = radius*sin(DEG2RAD*alfa);
TargetR.Pj().z = 0.74;

Vec3 HrpyL;
Vec3 HrpyR;

r1Model->handL(HandL.Pj().x, HandL.Pj().y, HandL.Pj().z, HrpyL.x, HrpyL.y, HrpyL.z);
r1Model->handR(HandR.Pj().x, HandR.Pj().y, HandR.Pj().z, HrpyR.x, HrpyR.y, HrpyR.z);

HandL.Rj() = Rotation(HrpyL);
HandR.Rj() = Rotation(HrpyR);

static int ntarget = 0;

timeout -= PERIOD;

static double heading_error = 0.0;

if ((HandL.Pj() - TargetL.Pj()).mod() < 0.01)
{
    //fprintf(dump, "%f, %f, %f, 1, %f\n", TargetL.Pj().x, TargetL.Pj().y, 0.16 + TargetL.Pj().z, heading_error);
    printf("%f, %f, %f, 1, %f\n", TargetL.Pj().x, TargetL.Pj().y, 0.16 + TargetL.Pj().z, heading_error);

    static char buff[32];
    sprintf_s(buff, "T%d", ntarget++);
    sendTarget(std::string(buff), 0, 192, 0, TargetL.Pj().x, TargetL.Pj().y, TargetL.Pj().z, 8.0, 0.666, 360.0, 360.0, 360.0);

    newtarget = true;
}
else if (timeout < 0.0)
{
    //fprintf(dump, "%f, %f, %f, 0, %f\n", TargetL.Pj().x, TargetL.Pj().y, 0.16 + TargetL.Pj().z, heading_error);
    printf("%f, %f, %f, 0, %f\n", TargetL.Pj().x, TargetL.Pj().y, 0.16 + TargetL.Pj().z, heading_error);

    static char buff[32];
    sprintf_s(buff, "T%d", ntarget++);
    sendTarget(std::string(buff), 192, 0, 32, TargetL.Pj().x, TargetL.Pj().y, TargetL.Pj().z, 8.0, 0.666, 360.0, 360.0, 360.0);

    newtarget = true;
}
else
{
    newtarget = false;
}

static Vec3 Tl = TargetL.Pj();
static Vec3 Tr = TargetR.Pj();

Vl = (TargetL.Pj() - Tl) / PERIOD;
Vr = (TargetR.Pj() - Tr) / PERIOD;

Tl = TargetL.Pj();
Tr = TargetR.Pj();

//TargetL.Pj() += PERIOD*Vl;
//TargetR.Pj() += PERIOD*Vr;

//if (TargetL.Pj().z < 0.5) { TargetL.Pj().z = 0.5; Vl.z = 0.0; }
//if (TargetR.Pj().z < 0.5) { TargetR.Pj().z = 0.5; Vr.z = 0.0; }
//if (TargetL.Pj().z > 1.0) { TargetL.Pj().z = 1.0; Vl.z = 0.0; }
//if (TargetR.Pj().z > 1.0) { TargetR.Pj().z = 1.0; Vr.z = 0.0; }

// Wl and Wr are angular velocities

//ArpyL += PERIOD*Wl;
//ArpyR += PERIOD*Wr;

//TargetL.Rj() = Rotation(ArpyL);
//TargetR.Rj() = Rotation(ArpyR);





//Wl = (HandL.Zj() % Vec3(0.0, 0.0, 1.0));
//Wl = (HandL.Rj().Ey() % Vec3(0.0, 0.0, -1.0));
//Wl = (HandL.Rj().Ex() % Vec3(0.0, 0.0, -1.0));
//Wl *= 0.1;
#endif
