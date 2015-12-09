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

#include <stdio.h>
#include <time.h>

#include <yarp/os/Time.h>

#include <cer_kinematics_alt/private/Matrix.h>
#include <cer_kinematics_alt/private/Joints.h>

#include <cer_kinematics_alt/Solver.h>

// pseudoinverse as Lagrange multipliers problem
// minimize 1/2*qd.t()*W*qd under the constraint v = J*qd
// G(qd,L) = 1/2*qd.t()*W*qd+L.t()*(v-J*qd)
// dG/dqd = qd.t()*W-L.t()*J = 0  ==> qd.t()*W = L.t()*J  ==>  W*qd = J.t()*L  ==>  qd = W.inv()*J.t()*L
// v = J*qd = J*W.inv()*J.t()*L  ==>  L = (J*W.inv()*J.t()).inv()*v
// qd = W.inv()*J.t()*L = W.inv()*J.t()*(J*W.inv()*J.t()).inv()*v

//Vec3 Werr=(mHand[L]->Toj.Rj().Ex()%Uxstar)+(mHand[L]->Toj.Rj().Ey()%Uystar)+(mHand[L]->Toj.Rj().Ez()%Uzstar);

namespace cer {
namespace kinematics_alt {

#define NJOINTS 22

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
class LeftSideSolverImpl
{
public:

    LeftSideSolverImpl();

    virtual ~LeftSideSolverImpl(void)
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

    int checkG();

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
};

}
}

using namespace cer::kinematics_alt;

int LeftSideSolverImpl::checkG()
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

LeftSideSolverImpl::LeftSideSolverImpl() :
    q(NJOINTS),
    qzero(NJOINTS),
    q_valid(NJOINTS),
    T_ROOT(0,0,0,0,0,0),
    COM_valid(0.0,0.0,-0.16),
    Jg(2,NJOINTS),
    S(12,12),
    Jhand(6,NJOINTS),
    J(6,12),
    Jv(3,12),
    Jw(3,12),
    AJvt(12,3),
    SJt(12,6),
    lva(3),
    Rva(3,3),
    Lva(3,3),
    qv(12),
    Z(12,12),
    OJwt(12,3),
    Lwa(3,3),
    A(12,12),
    qz(12),
    W(12,12),W2(12,12)
{
    nj=0;

    int id=0;

    mRoot = new Component();

    //Component* base = new Component(Transform(0, 0, 180, 0, 0, -410),mRoot);
    Component* base = new Component(Transform(0, 0, 180, 0, 0, 0),mRoot);

    //mTrifidTorso = new Trifid(id, 90, 360, 560, qmin, qmax, base);

    mTorsoExt=0.0;
    mTorsoExc=1.5*0.090*tan(DEG2RAD*TORSO_MAX_TILT);
    //mTrifidTorso = new Trifid(id, 90, 410.0-0.5*hmax, 410.0+0.5*hmax, qmin, qmax, base);
    mTrifidTorso = new Trifid(id, 90, 1000.0*(mTorsoExt-0.5*mTorsoExc), 1000.0*(mTorsoExt+0.5*mTorsoExc), qmin, qmax, base);

    mPart[0] = mPart[1] = mPart[2] = mTrifidTorso;

    mTorsoYaw = new Joint(id,-90,90,qmin,qmax,mTrifidTorso); mJoint[nj++] = (Joint*)mTorsoYaw;

    mPart[3] = mTorsoYaw;

    // left arm
    {
        mLShoulder = new Component(Transform(-84,312.317,100,180),mTorsoYaw);

        startLArm = id;
        mLshoulder0 = new Joint(id,-20,100,qmin,qmax,mLShoulder); mJoint[nj++] = (Joint*)mLshoulder0;
        mLshoulder0 = new Component(Transform(0,-159.422,90,90),mLshoulder0);
        mPart[4] = mLshoulder0;

        mLshoulder1 = new Joint(id,-10,100,qmin,qmax,mLshoulder0); mJoint[nj++] = (Joint*)mLshoulder1;
        mLshoulder1 = new Component(Transform(34,0,-90,-100),mLshoulder1);
        mPart[5] = mLshoulder1;

        Component* Lshoulder2 = new Joint(id,-90,90,qmin,qmax,mLshoulder1); mJoint[nj++] = (Joint*)Lshoulder2;
        Lshoulder2 = new Component(Transform(0,-251,90,-90),Lshoulder2);
        mUArm[L] = Lshoulder2;
        mPart[6] = Lshoulder2;

        Component* Lelbow = new Joint(id,0,140,qmin,qmax,Lshoulder2); mJoint[nj++] = (Joint*)Lelbow;
        Lelbow = new Component(Transform(0,0,-90,0),Lelbow);
        mPart[7] = Lelbow;

        startLForeArm = id;
        Component* LfarmRot = new Joint(id,-90,90,qmin,qmax,Lelbow); mJoint[nj++] = (Joint*)LfarmRot;
        LfarmRot = new Component(Transform(0,-71,180,-90),LfarmRot);
        mPart[8] = LfarmRot;

        mFArm[L] = LfarmRot;

        mArmExt[L]=0.230;
        mArmExc[L]=1.5*0.018*tan(DEG2RAD*WRIST_MAX_TILT);
        //mTrifidHand[L] = new Trifid(id, 18, 220, 360, qmin, qmax, LfarmRot);
        mTrifidHand[L] = new Trifid(id, 18, 1000.0*(mArmExt[L]-0.5*mArmExc[L]), 1000.0*(mArmExt[L]+0.5*mArmExc[L]), qmin, qmax, LfarmRot);

        Component* frameAdj=new Component(Transform(0,20,0,0,0,0),mTrifidHand[L]);

        mHand[L] = new Component(Transform(0,-90,0,0,0,70),frameAdj);

        mPart[9] = mPart[10] = mPart[11] = mHand[L];
    }

    // right arm
    {
        mRShoulder = new Component(Transform(-84,312.317,80,-180),mTorsoYaw);

        startRArm = id;
        mRshoulder0 = new Joint(id,-20,100,qmin,qmax,mRShoulder); mJoint[nj++] = (Joint*)mRshoulder0;
        mRshoulder0 = new Component(Transform(0,159.422,90,-90),mRshoulder0);

        mPart[12] = mRshoulder0;

        mRshoulder1 = new Joint(id,-10,100,qmin,qmax,mRshoulder0); mJoint[nj++] = (Joint*)mRshoulder1;
        mRshoulder1 = new Component(Transform(-34,0,-90,-100),mRshoulder1);

        mPart[13] = mRshoulder1;

        Component* Rshoulder2 = new Joint(id,-90,90,qmin,qmax,mRshoulder1); mJoint[nj++] = (Joint*)Rshoulder2;
        Rshoulder2 = new Component(Transform(0,251,-90,90),Rshoulder2);
        mUArm[R] = Rshoulder2;

        mPart[14] = Rshoulder2;

        Component* Relbow = new Joint(id,0,140,qmin,qmax,Rshoulder2); mJoint[nj++] = (Joint*)Relbow;
        Relbow = new Component(Transform(0,0,90,0),Relbow);

        mPart[15] = Relbow;

        startRForeArm = id;
        Component* RfarmRot = new Joint(id,-90,90,qmin,qmax,Relbow); mJoint[nj++] = (Joint*)RfarmRot;
        RfarmRot = new Component(Transform(0,71,0,-90),RfarmRot);
        mFArm[R] = RfarmRot;

        mPart[16] = RfarmRot;

        mArmExt[R]=0.230;
        mArmExc[R]=1.5*0.018*tan(DEG2RAD*WRIST_MAX_TILT);
        //mTrifidHand[R] = new Trifid(id, 18, 220, 360, qmin, qmax, RfarmRot);
        mTrifidHand[R] = new Trifid(id, 18, 1000.0*(mArmExt[R]-0.5*mArmExc[R]), 1000.0*(mArmExt[R]+0.5*mArmExc[R]), qmin, qmax, RfarmRot);

        Component* frameAdj=new Component(Transform(0,20,0,0,0,0),mTrifidHand[R]);

        mHand[R] = new Component(Transform(0,-90,180,0,0,70),frameAdj);

        mPart[17] = mPart[18] = mPart[19] = mHand[R];
    }

    // head
    {
        mNeck = new Component(Transform(-84,339,90,180),mTorsoYaw);

        startNeck = id;
        Component* neckPitch = new Joint(id,-90,30,qmin,qmax,mNeck); mJoint[nj++] = (Joint*)neckPitch;
        neckPitch = new Component(Transform(0,0,-90,0),neckPitch);

        mPart[20] = neckPitch;

        Component* neckYaw = new Joint(id,-80,80,qmin,qmax,neckPitch); mJoint[nj++] = (Joint*)neckYaw;
        neckYaw = new Component(Transform(0,200,0,0),neckYaw);

        mHead = neckYaw;

        mPart[21] = mHead;
    }

    ///////////////////////////////////////////////////////////////////////////////////

    for (int j=0; j<NJOINTS; ++j)
    {
        Q[j]=DEG2RAD*30.0;
        Ks[j]=0.25*RAD2DEG;
        Kz[j]=0.01*DEG2RAD;
    }

    for (int j=0; j<=2; ++j)
    {
        int k=startLForeArm+1+j;
        int h=startRForeArm+1+j;

        Q[j]=0.020;
        Q[h]=Q[k]=0.014;
        Ks[j]=Ks[k]=Ks[h]=0.25;
        Kz[j]=Kz[h]=Kz[k]=0.01;
    }

    for (int j=0; j<12; ++j)
    {
        S(j,j)=Q[j]*Q[j];
        W(j,j)=Q[j];
        W2(j,j)=Q[j]*Q[j];
    }

    q.clear();

    // masses

    q(0)=q(1)=q(2)=0.410;
    q(startLForeArm+1)=q(startLForeArm+2)=q(startLForeArm+3)=0.22;
    q(startRForeArm+1)=q(startRForeArm+2)=q(startRForeArm+3)=0.22;

    calcPostureFromRoot(T_ROOT);

    mHand[R]->setCOMworldRef(0.6,-84,-167,-268);
    mHand[L]->setCOMworldRef(2.6,-84, 167,-268);

    mUArm[R]->setCOMworldRef(1.5,-84,-191,215);
    mUArm[L]->setCOMworldRef(1.5,-84, 191,215);

    mTrifidHand[R]->setCOMworldRef(1.0,-84,-191,-89);
    mTrifidHand[L]->setCOMworldRef(1.0,-84, 191,-89);

    mNeck->setCOMworldRef( 7.5,-14,0, 180);
    mHead->setCOMworldRef( 2.5,-28,0, 521);
    mRoot->setCOMworldRef(25.0,-48,0,-415);

    /////////

    JOINTS(j) q(j)=0.5*(qmin[j]+qmax[j]);

    q(20)=q(21)=0.0;

    q_valid=qzero=q;

    mPosThreshold=DEFAULT_POS_THRESHOLD;

    calcPostureFromRoot(T_ROOT);
}

int LeftSideSolverImpl::leftHand2Target(Vec3& Xstar,Quaternion& Qstar)
{
    calcPostureFromRoot(T_ROOT);

    //////////////////////////////////////////

    Vec3 Vstar=Xstar-mHand[L]->Toj.Pj();

    if (Vstar.mod()<mPosThreshold) return ON_TARGET;

    Vec3 Wstar=(Qstar*mHand[L]->Toj.Rj().quaternion().conj()).V;

    Matrix Vref=     Vstar;
    Matrix Wref=M_PI*Wstar;

    /////////////////////////////////////////

    mHand[L]->getJ(Jhand);

    J=Jhand.sub(0,6,0,12);

    Jv=J.sub(0,3,0,12);
    Jw=J.sub(3,3,0,12);

    //////////////////////

    for (int j=0; j<12; ++j)
    {
        double s=(q(j)-qzero(j))/((q(j)>qzero(j))?(qmax[j]-qzero(j)):(qmin[j]-qzero(j)));

        if (s>1.0) s=1.0;

        A(j,j)=sqrt(1.0-s*s)*S(j,j);

        qz(j)=Kz[j]*(qzero(j)-q(j));
    }

    SJt=fast_mul_diag_full(S,J.t());
    qz-=SJt*((J*SJt).inv()*(J*qz));

    //qz-=J.inv(S)*(J*qz);

    ///////////////////////////////////////////////

    //AJvt=A*Jv.t();
    AJvt=fast_mul_diag_full(A,Jv.t());

    (Jv*AJvt).base(lva,Rva);

    //Lva(0,0)=lva(0)/(lva(0)*lva(0)+0.000004);
    //Lva(1,1)=lva(1)/(lva(1)*lva(1)+0.000004);
    //Lva(2,2)=lva(2)/(lva(2)*lva(2)+0.000004);

    Lva(0,0)=1.0/lva(0);
    Lva(1,1)=1.0/lva(1);
    Lva(2,2)=1.0/lva(2);

    Matrix Vrot=Lva*Rva.t()*Vref;

    static const double VLIM=5.0;

    if (fabs(Vrot(0))>VLIM) Vrot(0)=(Vrot(0)>0.0?VLIM:-VLIM);
    if (fabs(Vrot(1))>VLIM) Vrot(1)=(Vrot(1)>0.0?VLIM:-VLIM);
    if (fabs(Vrot(2))>VLIM) Vrot(2)=(Vrot(2)>0.0?VLIM:-VLIM);

    qv=AJvt*(Rva*Vrot);

    ///////////////////////////////////////////////

    static Matrix I=Matrix::id(12);

    Z=I-Jv.inv(S)*Jv;

    OJwt=Z*fast_mul_diag_full(A,(Jw*Z).t());

    static Matrix lwa(3),Rwa(3,3);

    (Jw*OJwt).base(lwa,Rwa);

    Lwa(0,0)=1.0/lwa(0);
    Lwa(1,1)=1.0/lwa(1);
    Lwa(2,2)=1.0/lwa(2);

    Matrix Wrot=Lwa*Rwa.t()*(Wref-Jw*qv);

    static const double WLIM=1.0;

    if (fabs(Wrot(0))>WLIM) Wrot(0)=(Wrot(0)>0.0?WLIM:-WLIM);
    if (fabs(Wrot(1))>WLIM) Wrot(1)=(Wrot(1)>0.0?WLIM:-WLIM);
    if (fabs(Wrot(2))>WLIM) Wrot(2)=(Wrot(2)>0.0?WLIM:-WLIM);

    qv+=OJwt*(Rwa*Wrot);

    ////////////////////////////////////////

    for (int j=0; j<12; ++j) q(j)+=Ks[j]*(qv(j)+qz(j));

    limitJointAngles(q);

    q_valid=q;

    return IN_PROGRESS;
}

///////////////////////////////////////////


LeftSideSolver::LeftSideSolver()
{
    solverImpl=new LeftSideSolverImpl();
}

LeftSideSolver::~LeftSideSolver()
{
    delete solverImpl;
}

yarp::sig::Vector LeftSideSolver::getCOM()
{
    yarp::sig::Vector com(3);
    
    Vec3 G=solverImpl->getCOM();
    com(0)=G.x;
    com(1)=G.y;
    com(2)=G.z;

    return com;
}

void LeftSideSolver::setPositionThreshold(double thr)
{ 
    solverImpl->setPositionThreshold(thr); 
}

bool LeftSideSolver::fkin(const yarp::sig::Vector &qin, yarp::sig::Matrix &H,int frame)
{
    return solverImpl->fkin(qin,H,frame);
}

bool LeftSideSolver::ikin(const yarp::sig::Matrix &Hd, yarp::sig::Vector &qout,double armElong,double torsoElong)
{
    return solverImpl->ikin(Hd,qout,armElong,torsoElong);
}

