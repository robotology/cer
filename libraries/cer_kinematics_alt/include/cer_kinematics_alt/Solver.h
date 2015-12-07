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

#ifndef __CER_SOLVER_H__
#define __CER_SOLVER_H__

//#include <Windows.h>

#include <stdio.h>
#include <time.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Time.h>

#include <cer_kinematics_alt/private/Matrix.h>
#include <cer_kinematics_alt/private/Joints.h>

namespace cer {
namespace kinematics2 {

#define NJOINTS 22
#define PERIOD 0.01 // seconds

#define DEFAULT_ARM_EXTENSION   0.23
#define DEFAULT_TORSO_EXTENSION 0.0
#define DEFAULT_POS_THRESHOLD   0.005

#define SOLVER_TIMEOUT 40 

#define WRIST_MAX_TILT 35.0
#define TORSO_MAX_TILT 30.0

#define ROOT NULL

enum { R = 0, L = 1 };

// pseudoinverse as Lagrange multipliers problem
// minimize 1/2*qd.t()*W*qd under the constraint v = J*qd
// G(qd,L) = 1/2*qd.t()*W*qd+L.t()*(v-J*qd)
// dG/dqd = qd.t()*W-L.t()*J = 0  ==> qd.t()*W = L.t()*J  ==>  W*qd = J.t()*L  ==>  qd = W.inv()*J.t()*L
// v = J*qd = J*W.inv()*J.t()*L  ==>  L = (J*W.inv()*J.t()).inv()*v
// qd = W.inv()*J.t()*L = W.inv()*J.t()*(J*W.inv()*J.t()).inv()*v


/**
 * Class to handle direct and inverse kinematics of the robot arm. 
 * 
 * @author Alessandro Scalzo
 */
class LeftSideSolver
{
public:

    LeftSideSolver(); 

    virtual ~LeftSideSolver(void)
    {
        if (mRoot) delete mRoot;
    }

    Vec3 getCOM(){ return COM_valid; }

    /**
     * Set the required precision for position reaching.
     *
     * @param thr the threshold for the position reaching ([m]).
     */
    void setPositionThreshold(double thr=DEFAULT_POS_THRESHOLD)
    {
        mPosThreshold=thr;
    }

    /**
     * Forward Kinematics Law.
     * 
     * @param qin      the DOFs values ([m]-[deg]-[m]).
     * @param H        the 4-by-4 homogeneous matrix of the specified 
     *                 frame ([m]).
     * @param frame    specify the DOF number whose frame is returned. 
     *                 Thus, frame is in [0...nDOF-1]; negative
     *                 numbers account for the end-effector frame.
     * @return true/false on success/failure.
     */
    bool fkin(const yarp::sig::Vector &qin, yarp::sig::Matrix &H,int frame=LEFT_HAND)
    {
        if (frame<0) frame=LEFT_HAND;
            
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

    /**
     * Inverse Kinematics Law.
     * 
     * @param Hd         the desired 4-by-4 homogeneous matrix 
     *                   representing the end-effector frame ([m]).
     * @param qout       the solved DOFs ([m]-[deg]-[m]).
     * @param armElong   the desired elongation of the left forearm tripod ([m]). 
     * @param torsoElong the desired elongation of the torso tripod ([m]).
     * @return true/false on success/failure.
     */
    bool ikin(const yarp::sig::Matrix &Hd, yarp::sig::Vector &qout,double armElong=DEFAULT_ARM_EXTENSION,double torsoElong=DEFAULT_TORSO_EXTENSION)
    {
        if (torsoElong!=mTorsoExt)
        {
            qzero(0)=qzero(1)=qzero(2)=mTorsoExt=torsoElong;
            qmin[0]=qmin[1]=qmin[2]=mTorsoExt-0.5*mTorsoExc;
            qmax[0]=qmax[1]=qmax[2]=mTorsoExt+0.5*mTorsoExc;
        }

        if (armElong!=mArmExt[L])
        {
            qzero(9)=qzero(10)=qzero(11)=mArmExt[L]=armElong;
            qmin[9]=qmin[10]=qmin[11]=mArmExt[L]-0.5*mArmExc[L];
            qmax[9]=qmax[10]=qmax[11]=mArmExt[L]+0.5*mArmExc[L];
        }

        QtargetL=Rotation(Vec3(Hd(0,0),Hd(1,0),Hd(2,0)),Vec3(Hd(0,1),Hd(1,1),Hd(2,1)),Vec3(Hd(0,2),Hd(1,2),Hd(2,2))).quaternion();
        
        XtargetL=Vec3(Hd(0,3),Hd(1,3),Hd(2,3)); 

        q=q_valid=qzero;

        int ret_code=IN_PROGRESS;

        int steps=0;

        int N=qout.length();
        
        if (N>NJOINTS) N=NJOINTS; 

        //leftHand2Target(XtargetL,QtargetL);
        //for (int j=0; j<N; ++j) qout(j)=q(j);

        while (ret_code==IN_PROGRESS)
        {
            ret_code=leftHand2Target(XtargetL,QtargetL);

            if (++steps>=SOLVER_TIMEOUT)
            {
                ret_code=OUT_OF_REACH;
            }
        }

        if (ret_code==OUT_OF_REACH)
        {
            for (int j=0; j<N; ++j) qout(j)=q(j);

            return false;
        }

        if (ret_code==ON_TARGET)
        {
            for (int j=0; j<N; ++j) qout(j)=q(j);

            return true;
        }
        
        return false;
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
            if (ql(j)<=qmin[j]) 
            {
                if (qd(j)<0.0)
                {
                    qd(j)=0.0;
                }
            }
            else if (ql(j)>=qmax[j])
            {   
                if (qd(j)>0.0)
                {
                    qd(j)=0.0;
                }
            }
        }
    }

    inline int checkG();

    int leftHand2Target(Vec3& Xstar,Quaternion& Qstar);

    Matrix q;
    Matrix qzero;
    Matrix q_valid;
    Vec3 COM;
    Vec3 COM_valid;

    Matrix Jg;
    Vec3 Gforce;
    Matrix S;

    Matrix W,W2;

    Matrix Jhand;
    Matrix J;
    Matrix Jv; 
    Matrix Jw; 
    Matrix AJvt; 
    Matrix lva,Rva; 
    Matrix Lva;
    Matrix qv; 
    Matrix Z;
    Matrix OJwt;
    Matrix Lwa;
    Matrix A;
    Matrix qz;
    Matrix SJt;

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
    double qp_old[NJOINTS];
    double Kz[NJOINTS];
    double Ks[NJOINTS];

    double Q[NJOINTS];
    double Q2[NJOINTS];

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

    enum { IMPOSSIBLE=-2,FAILURE=-1,IN_PROGRESS=0,ON_TARGET=1,OUT_OF_LIMITS=2,OUT_OF_REACH=3,FALLEN=4,BALANCED=5,FRONTIER=6 };
    enum { LEFT_HAND=11,RIGHT_HAND=19,HEAD=21 };
};

inline int LeftSideSolver::checkG()
{
    Matrix Jl(2,NJOINTS);

    G[0]=mRoot->getJg(Jl);
    COM=25.0*G[0];
    Jg=25.0*Jl;

    G[1]=mNeck->getJg(Jl);
    COM+=7.5*G[1];
    Jg+=7.5*Jl;

    G[2]=mHead->getJg(Jl);
    COM+=2.5*G[2];
    Jg+=2.5*Jl;


    G[3]=mUArm[L]->getJg(Jl);
    COM+=1.5*G[3];
    Jg+=1.5*Jl;

    G[4]=mTrifidHand[L]->getJg(Jl);
    COM+=1.0*G[4];
    Jg+=1.0*Jl;

    G[5]=mHand[L]->getJg(Jl);
    COM+=2.6*G[5];
    Jg+=2.6*Jl;


    G[6]=mUArm[R]->getJg(Jl);
    COM+=1.5*G[6];
    Jg+=1.5*Jl;

    G[7]=mTrifidHand[R]->getJg(Jl);
    COM+=1.0*G[7];
    Jg+=1.0*Jl;

    G[8]=mHand[R]->getJg(Jl);
    COM+=0.6*G[8];
    Jg+=0.6*Jl;


    COM/=43.2;
    Jg/=43.2;

    for (int j=0; j<4; ++j) for (int k=0; k<2; ++k)
    {
        Jg(k,startLForeArm+j)=0.0;
        Jg(k,startRForeArm+j)=0.0;
    }

    COM.z=0.0;

    Vec3 C1( 0.055, 0.110, 0.0);
    Vec3 W1(-0.074, 0.170, 0.0);
    Vec3 C2(-0.244, 0.000, 0.0);
    Vec3 W2(-0.074,-0.170, 0.0);
    Vec3 C3( 0.055,-0.110, 0.0);

    //Vec3 G0=(C1+C2+C3+W1+W2)/5.0;
    Vec3 G0=(W1+W2)/2.0;

    Vec3 L0=(W1-C1).norm();
    Vec3 L1=(C2-W1).norm();
    Vec3 L2=(W2-C2).norm();
    Vec3 L3=(C3-W2).norm();
    Vec3 L4=(C1-C3).norm();

    Vec3 D0=COM-C1,D1=COM-W1,D2=COM-C2,D3=COM-W2,D4=COM-C3;

    double d0=(D0%L0).z,d1=(D1%L1).z,d2=(D2%L2).z,d3=(D3%L3).z,d4=(D4%L4).z;

    Gforce.clear();

    if (d0>=0.0 || d1>=0.0 || d2>=0.0 || d3>=0.0 || d4>=0.0)
    {
        Gforce=10.0*(COM_valid-COM);
        //return FRONTIER;
        return OUT_OF_REACH;
    }

    D0=D0-(D0*L0)*L0;
    D1=D1-(D1*L1)*L1;
    D2=D2-(D2*L2)*L2;
    D3=D3-(D3*L3)*L3;
    D4=D4-(D4*L4)*L4;

    d0=D0.mod(); d1=D1.mod(); d2=D2.mod(); d3=D3.mod(); d4=D4.mod();

    if (d0>0.04 && d1>0.04 && d2>0.04 && d3>0.04 && d4>0.04)
    {
        q_valid=q;
        COM_valid=COM;
        Gforce=G0-COM;
        return BALANCED;
    }

    if (d0>0.02 && d1>0.02 && d2>0.02 && d3>0.02 && d4>0.02)
    {
        Gforce=10.0*(COM_valid-COM);
        return FRONTIER;
    }

    return OUT_OF_REACH;
}

}
}

#endif


