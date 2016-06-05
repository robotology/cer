/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
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

//#include <stdio.h>
//#include <time.h>
//#include <iostream>

#ifndef SOLVER_IMPL___
#define SOLVER_IMPL___

#include <yarp/os/Time.h>

#include <cer_kinematics_alt/private/Matrix.h>
#include <cer_kinematics_alt/private/Joints.h>

#include <cer_kinematics_alt/Solver.h>

#define TORSO_MAX_TILT          30.0  // [deg]
#define MIN_TORSO_EXTENSION    -0.05  // [m]
#define MAX_TORSO_EXTENSION     0.15  // [m]

#define MIN_ARM_EXTENSION       0.0   // [m]
#define MAX_ARM_EXTENSION       0.14  // [m]
#define WRIST_MAX_TILT          35.0  // [deg]

namespace cer {
namespace kinematics_alt {

#define NJOINTS 22

#define ROOT NULL

enum { R = 0, L = 1 };

class KinR1Impl
{
public:

    KinR1Impl();

    virtual ~KinR1Impl(void)
    {
        if (mRoot) delete mRoot;
    }

    Vec3 getCOM(){ return COMbalanced; }

    void getConfig(yarp::sig::Vector &qi)
    {
        for (int j=0; j<NJOINTS; ++j) qi(j)=q(j);
    }

    void setPositionThreshold(double thr=DEFAULT_POS_THRESHOLD)
    {
        mPosThreshold=thr;
    }

    bool fkin(const yarp::sig::Vector &qin, yarp::sig::Matrix &H,int frame=-1)
    {
        if (frame<0) frame=L_WRIST_TRIPOD_3;

        if (frame>NJOINTS) return false;

        Matrix qconf(NJOINTS);

        int N=qin.length();

        if (N>NJOINTS) N=NJOINTS;

        for (int j=0; j<N; ++j) qconf(j)=qin(j);

        mRoot->calcPosture(qconf,T_ROOT);

        Rotation& R=mPart[frame]->Toj.Rj();
        Vec3& P=mPart[frame]->Toj.Pj();

        H.resize(4,4);
        H(0,0)=R(0,0); H(0,1)=R(0,1); H(0,2)=R(0,2); H(0,3)=P.x;
        H(1,0)=R(1,0); H(1,1)=R(1,1); H(1,2)=R(1,2); H(1,3)=P.y;
        H(2,0)=R(2,0); H(2,1)=R(2,1); H(2,2)=R(2,2); H(2,3)=P.z;
        H(3,0)=0.0;    H(3,1)=0.0;    H(3,2)=0.0;    H(3,3)=1.0;

        return true;
    }

    void adjElongation(double armElongL,double armElongR,double torsoElong)
    {
        if (torsoElong!=mTorsoExt)
        {
            double qa=torsoElong-mTorsoExc;
            double qb=torsoElong+mTorsoExc;

            if (qa<MIN_TORSO_EXTENSION){ qa=MIN_TORSO_EXTENSION; qb+=MIN_TORSO_EXTENSION-qa; }
            if (qb>MAX_TORSO_EXTENSION){ qa+=MAX_TORSO_EXTENSION-qb; qb=MAX_TORSO_EXTENSION; }

            mTorsoExt=0.5*(qa+qb);

            qzero(0)=qzero(1)=qzero(2)=mTorsoExt;
            qmin[0]=qmin[1]=qmin[2]=qa;
            qmax[0]=qmax[1]=qmax[2]=qb;
        }

        if (armElongL!=mArmExt[L])
        {
            double qa=armElongL-mArmExc[L];
            double qb=armElongL+mArmExc[L];

            if (qa<MIN_ARM_EXTENSION){ qa=MIN_ARM_EXTENSION; qb+=MIN_ARM_EXTENSION-qa; }
            if (qb>MAX_ARM_EXTENSION){ qa+=MAX_ARM_EXTENSION-qb; qb=MAX_ARM_EXTENSION; }

            mArmExt[L]=0.5*(qa+qb);

            qzero(9)=qzero(10)=qzero(11)=mArmExt[L];
            qmin[9]=qmin[10]=qmin[11]=qa;
            qmax[9]=qmax[10]=qmax[11]=qb;
        }

        if (armElongR!=mArmExt[R])
        {
            double qa=armElongR-mArmExc[R];
            double qb=armElongR+mArmExc[R];

            if (qa<MIN_ARM_EXTENSION){ qa=MIN_ARM_EXTENSION; qb+=MIN_ARM_EXTENSION-qa; }
            if (qb>MAX_ARM_EXTENSION){ qa+=MAX_ARM_EXTENSION-qb; qb=MAX_ARM_EXTENSION; }

            mArmExt[R]=0.5*(qa+qb);

            qzero(17)=qzero(18)=qzero(19)=mArmExt[L];
            qmin[17]=qmin[18]=qmin[19]=qa;
            qmax[17]=qmax[18]=qmax[19]=qb;
        }
    }

    bool ikin_left_solver(const yarp::sig::Matrix &Hd,
              yarp::sig::Vector &qout,
              double armElongL=DEFAULT_ARM_EXTENSION,
              double armElongR=DEFAULT_ARM_EXTENSION,
              double torsoElong=DEFAULT_TORSO_EXTENSION,
              double timeoutSec=SOLVER_TIMEOUT)
    {
        adjElongation(armElongL,armElongR,torsoElong);

        QtargetL=Rotation(Vec3(Hd(0,0),Hd(1,0),Hd(2,0)),Vec3(Hd(0,1),Hd(1,1),Hd(2,1)),Vec3(Hd(0,2),Hd(1,2),Hd(2,2))).quaternion();

        XtargetL=Vec3(Hd(0,3),Hd(1,3),Hd(2,3));

        q=qzero;

        int N=qout.length();

        if (N>NJOINTS) N=NJOINTS;

        double time_end=yarp::os::Time::now()+timeoutSec;

        while (yarp::os::Time::now()<=time_end)
        {
            leftHand2Target(XtargetL,QtargetL,true);
        }

        calcPostureFromRoot(T_ROOT);
        checkG();
        
        for (int j=0; j<N; ++j) qout(j)=qbalanced(j);

        return true;
    }

    bool ikin_left_ctrl(const yarp::sig::Matrix &Hd,
              yarp::sig::Vector &qin,
              yarp::sig::Vector &qdotout,
              double armElongL=DEFAULT_ARM_EXTENSION,
              double armElongR=DEFAULT_ARM_EXTENSION,
              double torsoElong=DEFAULT_TORSO_EXTENSION)
    {
        adjElongation(armElongL,armElongR,torsoElong);

        QtargetL=Rotation(Vec3(Hd(0,0),Hd(1,0),Hd(2,0)),Vec3(Hd(0,1),Hd(1,1),Hd(2,1)),Vec3(Hd(0,2),Hd(1,2),Hd(2,2))).quaternion();

        XtargetL=Vec3(Hd(0,3),Hd(1,3),Hd(2,3));

        int N=qin.length();

        if (N>NJOINTS) N=NJOINTS;

        for (int j=0; j<N; ++j) q(j)=qin(j);

        leftHand2Target(XtargetL,QtargetL,false);
        
        for (int j=0; j<N; ++j) qdotout(j)=qdot(j);

        return true;
    }

    bool ikin_right_solver(const yarp::sig::Matrix &Hd,
              yarp::sig::Vector &qout,
              double armElongL=DEFAULT_ARM_EXTENSION,
              double armElongR=DEFAULT_ARM_EXTENSION,
              double torsoElong=DEFAULT_TORSO_EXTENSION,
              double timeoutSec=SOLVER_TIMEOUT)
    {
        adjElongation(armElongL,armElongR,torsoElong);

        QtargetL=Rotation(Vec3(Hd(0,0),Hd(1,0),Hd(2,0)),Vec3(Hd(0,1),Hd(1,1),Hd(2,1)),Vec3(Hd(0,2),Hd(1,2),Hd(2,2))).quaternion();

        XtargetL=Vec3(Hd(0,3),Hd(1,3),Hd(2,3));

        q=qzero;

        int N=qout.length();

        if (N>NJOINTS) N=NJOINTS;

        double time_end=yarp::os::Time::now()+timeoutSec;

        while (yarp::os::Time::now()<=time_end)
        {
            rightHand2Target(XtargetL,QtargetL,true);
        }

        calcPostureFromRoot(T_ROOT);
        checkG();
        
        for (int j=0; j<N; ++j) qout(j)=qbalanced(j);

        return true;
    }

    bool ikin_right_ctrl(const yarp::sig::Matrix &Hd,
              yarp::sig::Vector &qin,
              yarp::sig::Vector &qdotout,
              double armElongL=DEFAULT_ARM_EXTENSION,
              double armElongR=DEFAULT_ARM_EXTENSION,
              double torsoElong=DEFAULT_TORSO_EXTENSION)
    {
        adjElongation(armElongL,armElongR,torsoElong);

        QtargetL=Rotation(Vec3(Hd(0,0),Hd(1,0),Hd(2,0)),Vec3(Hd(0,1),Hd(1,1),Hd(2,1)),Vec3(Hd(0,2),Hd(1,2),Hd(2,2))).quaternion();

        XtargetL=Vec3(Hd(0,3),Hd(1,3),Hd(2,3));

        int N=qin.length();

        if (N>NJOINTS) N=NJOINTS;

        for (int j=0; j<N; ++j) q(j)=qin(j);

        rightHand2Target(XtargetL,QtargetL,false);
        
        for (int j=0; j<N; ++j) qdotout(j)=qdot(j);

        return true;
    }


    bool ikin_2hand_solver(const yarp::sig::Matrix &HdL,
                           const yarp::sig::Matrix &HdR,
                           yarp::sig::Vector &qout,
                           double armElongL=DEFAULT_ARM_EXTENSION,
                           double armElongR=DEFAULT_ARM_EXTENSION,
                           double torsoElong=DEFAULT_TORSO_EXTENSION,
                           double timeoutSec=SOLVER_TIMEOUT)
    {
        adjElongation(armElongL,armElongR,torsoElong);

        QtargetL=Rotation(Vec3(HdL(0,0),HdL(1,0),HdL(2,0)),Vec3(HdL(0,1),HdL(1,1),HdL(2,1)),Vec3(HdL(0,2),HdL(1,2),HdL(2,2))).quaternion();
        QtargetR=Rotation(Vec3(HdR(0,0),HdR(1,0),HdR(2,0)),Vec3(HdR(0,1),HdR(1,1),HdR(2,1)),Vec3(HdR(0,2),HdR(1,2),HdR(2,2))).quaternion();

        XtargetL=Vec3(HdL(0,3),HdL(1,3),HdL(2,3));
        XtargetR=Vec3(HdR(0,3),HdR(1,3),HdR(2,3));

        q=qzero;

        int N=qout.length();

        if (N>NJOINTS) N=NJOINTS;

        double time_end=yarp::os::Time::now()+timeoutSec;

        while (yarp::os::Time::now()<=time_end)
        {
            bothHands2Targets(XtargetL,QtargetL,XtargetR,QtargetR,true);
        }
        
        calcPostureFromRoot(T_ROOT);
        checkG();

        for (int j=0; j<N; ++j) qout(j)=qbalanced(j);

        return true;
    }

    bool ikin_2hand_ctrl(const yarp::sig::Matrix &HdL,
                         const yarp::sig::Matrix &HdR,
                         yarp::sig::Vector &qin,
                         yarp::sig::Vector &qdotout,
                         double armElongL=DEFAULT_ARM_EXTENSION,
                         double armElongR=DEFAULT_ARM_EXTENSION,
                         double torsoElong=DEFAULT_TORSO_EXTENSION)
    {
        adjElongation(armElongL,armElongR,torsoElong);

        QtargetL=Rotation(Vec3(HdL(0,0),HdL(1,0),HdL(2,0)),Vec3(HdL(0,1),HdL(1,1),HdL(2,1)),Vec3(HdL(0,2),HdL(1,2),HdL(2,2))).quaternion();
        QtargetR=Rotation(Vec3(HdR(0,0),HdR(1,0),HdR(2,0)),Vec3(HdR(0,1),HdR(1,1),HdR(2,1)),Vec3(HdR(0,2),HdR(1,2),HdR(2,2))).quaternion();

        XtargetL=Vec3(HdL(0,3),HdL(1,3),HdL(2,3));
        XtargetR=Vec3(HdR(0,3),HdR(1,3),HdR(2,3));

        int N=qin.length();

        if (N>NJOINTS) N=NJOINTS;

        for (int j=0; j<N; ++j) q(j)=qin(j);

        bothHands2Targets(XtargetL,QtargetL,XtargetR,QtargetR,false);
        
        for (int j=0; j<N; ++j) qdotout(j)=qdot(j);

        return true;
    }

protected:

    void calcPostureFrom(Component* pComp,Transform& Tref)
    {
        pComp->calcPosture(q,Tref,NULL);
    }

    void calcPostureFromRoot(Transform& Tref)
    {
        calcPostureFrom(mRoot,Tref);
    }

    void limitJointAngles(Matrix& ql)
    {
        for (int j=0; j<NJOINTS; ++j)
        {
            if (ql(j)<=qmin[j])
            {
                ql(j)=qmin[j];
            }
            else if (ql(j)>=qmax[j])
            {
                ql(j)=qmax[j];
            }
        }
    }

    void limitJointSpeeds(Matrix& ql,Matrix& qd)
    {
        for (int j=0; j<NJOINTS; ++j)
        {
            if (qd(j)<-Q[j]) qd(j)=-Q[j]; else if (qd(j)>Q[j]) qd(j)=Q[j];
             
            if (ql(j)<=qmin[j])
            {
                if (qd(j)<0.0) qd(j)=qmin[j]-ql(j);
            }
            else if (ql(j)>=qmax[j])
            {
                if (qd(j)>0.0) qd(j)=qmax[j]-ql(j);
            }
        }
    }

    void checkG();

    void leftHand2Target(Vec3& Xstar,Quaternion& Qstar,bool oneShot);
    void rightHand2Target(Vec3& Xstar,Quaternion& Qstar,bool oneShot);
    void bothHands2Targets(Vec3& XstarL,Quaternion& QstarL,Vec3& XstarR,Quaternion& QstarR,bool oneShot);

    int leftHandController(Vec3& Xstar,Quaternion& Qstar);

    Matrix q;
    Matrix qdot;
    Matrix qzero;
    Vec3 COM;
    Vec3 COMbalanced;

    Matrix qbalanced;

    Matrix Jg;

    Vec3 Gforce;

    double W2[NJOINTS];
    double B2[NJOINTS];

    double Lvi[6];
    double Lwi[6];
    double Lgi[2];

    Vec3 XtargetL,XtargetR;
    Quaternion QtargetL,QtargetR;

    int startNeck;
    int startLArm;
    int startRArm;
    int startLForeArm;
    int startRForeArm;

    double mTorsoExt,mTorsoExc;
    double mArmExt[2],mArmExc[2];

    int nj;

    Vec3 G[9];

    double qmin[NJOINTS],qmax[NJOINTS];
    double Kz[NJOINTS];
    double Ks[NJOINTS];
    double Kc[NJOINTS];

    double Q[NJOINTS];

    double mPosThreshold;

    Component *mRoot;
    //Component *mBase;
    Component *mTorsoYaw;
    Component *mLShoulder;
    Component *mRShoulder;
    Component *mHand[2];

    Component *mUArm[2];
    Component *mFArm[2];
    Component *mHead;
    Component *mNeck;

    Component *mLshoulder0;
    Component *mRshoulder0;

    Component *mLshoulder1;
    Component *mRshoulder1;

    Component *mPart[NJOINTS];

    Joint *mJoint[NJOINTS];
    Trifid *mTrifidTorso;
    Trifid *mTrifidHand[2];

    Transform T_ROOT;

    enum { IMPOSSIBLE=-2,FAILURE=-1,IN_PROGRESS=0,ON_TARGET=1,OUT_OF_LIMITS=2,UNBALANCED=3,FALLEN=4,BALANCED=5,FRONTIER=6 };
};

}
}

#endif