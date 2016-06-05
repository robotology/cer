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

#include <cer_kinematics_alt/private/SolverImpl.h>

// pseudoinverse as Lagrange multipliers problem
// minimize 1/2*qd.t()*W*qd under the constraint v = J*qd
// G(qd,L) = 1/2*qd.t()*W*qd+L.t()*(v-J*qd)
// dG/dqd = qd.t()*W-L.t()*J = 0  ==> qd.t()*W = L.t()*J  ==>  W*qd = J.t()*L  ==>  qd = W.inv()*J.t()*L
// v = J*qd = J*W.inv()*J.t()*L  ==>  L = (J*W.inv()*J.t()).inv()*v
// qd = W.inv()*J.t()*L = W.inv()*J.t()*(J*W.inv()*J.t()).inv()*v

//Vec3 Werr=(mHand[L]->Toj.Rj().Ex()%Uxstar)+(mHand[L]->Toj.Rj().Ey()%Uystar)+(mHand[L]->Toj.Rj().Ez()%Uzstar);

using namespace cer::kinematics_alt;

KinR1Impl::KinR1Impl() :
    q(NJOINTS),
    qdot(NJOINTS),
    qzero(NJOINTS),
    qbalanced(NJOINTS),
    T_ROOT(0,0,0,0,0,0),
    Jg(2,20)
{
    nj=0;

    int id=0;

    mRoot = new Component();

    //Component* base = new Component(Transform(0, 0, 180, 0, 0, -410),mRoot);
    //Component* base = new Component(Transform(0, 0, 180, 0, 0, 0),mRoot);

    Component* base = new Component(Transform(0, 0, 180, 44, 0, 470),mRoot);

    //mTrifidTorso = new Trifid(id, 90, 360, 560, qmin, qmax, base);

    mTorsoExt=0.0;
    mTorsoExc=0.75*0.090*tan(DEG2RAD*TORSO_MAX_TILT);
    //mTrifidTorso = new Trifid(id, 90, 410.0-0.5*hmax, 410.0+0.5*hmax, qmin, qmax, base);
    mTrifidTorso = new Trifid(id, 90, 1000.0*(mTorsoExt-mTorsoExc), 1000.0*(mTorsoExt+mTorsoExc), qmin, qmax, base);

    mPart[0] = mPart[1] = mPart[2] = mTrifidTorso;

    mTorsoYaw = new Joint(id,-60,60,qmin,qmax,mTrifidTorso); mJoint[nj++] = (Joint*)mTorsoYaw;

    mPart[3] = mTorsoYaw;

    // left arm
    {
        //(A -0.084) (D 0.325869) (alpha 104.000002) (offset 180.0)
        mLShoulder = new Component(Transform(-84,325.869,104.000002,180),mTorsoYaw);

        startLArm = id;
        mLshoulder0 = new Joint(id,-30,90,qmin,qmax,mLShoulder); mJoint[nj++] = (Joint*)mLshoulder0;

        //(A -0.0) (D -0.182419) (alpha 90.0) (offset 90.0)
        mLshoulder0 = new Component(Transform(0,-182.419,90,90),mLshoulder0);
        mPart[4] = mLshoulder0;

        mLshoulder1 = new Joint(id,-10,75,qmin,qmax,mLshoulder0); mJoint[nj++] = (Joint*)mLshoulder1;
        
        //(A 0.034) (D 0.0) (alpha -90.0) (offset -104.000002)
        mLshoulder1 = new Component(Transform(34,0,-90,-104.000002),mLshoulder1);
        mPart[5] = mLshoulder1;

        Component* Lshoulder2 = new Joint(id,-90,90,qmin,qmax,mLshoulder1); mJoint[nj++] = (Joint*)Lshoulder2;

        //(A -0.0) (D -0.251) (alpha 90.0) (offset -90.0)
        Lshoulder2 = new Component(Transform(0,-251,90,-90),Lshoulder2);
        mUArm[L] = Lshoulder2;
        mPart[6] = Lshoulder2;

        Component* Lelbow = new Joint(id,0,100,qmin,qmax,Lshoulder2); mJoint[nj++] = (Joint*)Lelbow;
        //(A -0.0) (D 0.0) (alpha -90.0) (offset -0.0)
        Lelbow = new Component(Transform(0,0,-90,0),Lelbow);
        mPart[7] = Lelbow;

        startLForeArm = id;
        Component* LfarmRot = new Joint(id,-90,90,qmin,qmax,Lelbow); mJoint[nj++] = (Joint*)LfarmRot;
        
        //(A -0.0) (D -0.291) (alpha -180.0) (offset -90.0)
        LfarmRot = new Component(Transform(0,-291,180,-90),LfarmRot);
        mPart[8] = LfarmRot;

        mFArm[L] = LfarmRot;

        mArmExt[L]=DEFAULT_ARM_EXTENSION;
        mArmExc[L]=0.75*0.018*tan(DEG2RAD*WRIST_MAX_TILT);
        //mTrifidHand[L] = new Trifid(id, 18, 220, 360, qmin, qmax, LfarmRot);
        mTrifidHand[L] = new Trifid(id, 18, 1000.0*(mArmExt[L]-mArmExc[L]), 1000.0*(mArmExt[L]+mArmExc[L]), qmin, qmax, LfarmRot);

        Component* frameAdj=new Component(Transform(0,15,0,0,0,0),mTrifidHand[L]);
        mHand[L] = new Component(Transform(0,-90,0,0,0,104),frameAdj);

        mPart[9] = mPart[10] = mPart[11] = mHand[L];
    }

    // right arm
    {
        //(A -0.084) (D 0.325869) (alpha 75.999998) (offset 180.0) (min -60.0) (max 60.0)
        mRShoulder = new Component(Transform(-84,325.869,75.999998,180),mTorsoYaw);

        startRArm = id;
        mRshoulder0 = new Joint(id,-30,90,qmin,qmax,mRShoulder); mJoint[nj++] = (Joint*)mRshoulder0;
        //(A -0.0) (D 0.182419) (alpha 90.0) (offset -90.0) (min -30.0) (max 90.0)
        mRshoulder0 = new Component(Transform(0,182.419,90,-90),mRshoulder0);

        mPart[12] = mRshoulder0;

        mRshoulder1 = new Joint(id,-10,75,qmin,qmax,mRshoulder0); mJoint[nj++] = (Joint*)mRshoulder1;
        //(A -0.034) (D 0.0) (alpha -90.0) (offset -104.000002) (min -10.0) (max 75.0)
        mRshoulder1 = new Component(Transform(-34,0,-90,-104.000002),mRshoulder1);

        mPart[13] = mRshoulder1;

        Component* Rshoulder2 = new Joint(id,-90,90,qmin,qmax,mRshoulder1); mJoint[nj++] = (Joint*)Rshoulder2;
        //(A -0.0) (D 0.251) (alpha -90.0) (offset 90.0) (min -90.0) (max 90.0)
        Rshoulder2 = new Component(Transform(0,251,-90,90),Rshoulder2);
        mUArm[R] = Rshoulder2;

        mPart[14] = Rshoulder2;

        Component* Relbow = new Joint(id,0,101,qmin,qmax,Rshoulder2); mJoint[nj++] = (Joint*)Relbow;
        //(A -0.0) (D 0.0) (alpha 90.0) (offset -0.0) (min 0.0) (max 101.0)
        Relbow = new Component(Transform(0,0,90,0),Relbow);

        mPart[15] = Relbow;

        startRForeArm = id;
        Component* RfarmRot = new Joint(id,-90,90,qmin,qmax,Relbow); mJoint[nj++] = (Joint*)RfarmRot;
        //(A -0.0) (D 0.291) (alpha 0.0) (offset -90.0) (min -90.0) (max 90.0)
        RfarmRot = new Component(Transform(0,291,0,-90),RfarmRot);
        mFArm[R] = RfarmRot;

        mPart[16] = RfarmRot;

        mArmExt[R]=DEFAULT_ARM_EXTENSION;
        mArmExc[R]=0.75*0.018*tan(DEG2RAD*WRIST_MAX_TILT);
        //mTrifidHand[R] = new Trifid(id, 18, 220, 360, qmin, qmax, RfarmRot);
        mTrifidHand[R] = new Trifid(id, 18, 1000.0*(mArmExt[R]-mArmExc[R]), 1000.0*(mArmExt[R]+mArmExc[R]), qmin, qmax, RfarmRot);

        Component* frameAdj=new Component(Transform(0,15,0,0,0,0),mTrifidHand[R]);
        mHand[R] = new Component(Transform(0,-90,180,0,0,104),frameAdj);

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
        Ks[j]=0.05*RAD2DEG;
        Kz[j]=0.25*DEG2RAD;
    
        Kc[j]=RAD2DEG;
    }

    for (int j=0; j<=2; ++j)
    {
        int k=startLForeArm+1+j;
        int h=startRForeArm+1+j;

        Q[j]=0.02;
        Q[k]=Q[h]=0.01;
        Ks[j]=Ks[k]=Ks[h]=0.05;
        Kz[j]=Kz[k]=Kz[h]=0.25;

        Kc[j]=Kc[k]=Kc[h]=1.0;
    }

    for (int j=0; j<NJOINTS; ++j)
    {
        W2[j]=Q[j]*Q[j];
    }

    q.clear();

    // masses

    q(0)=q(1)=q(2)=0.0;

    q(startLForeArm+1)=q(startRForeArm+1)=0.01+0.005;

    q(startLForeArm+2)=q(startRForeArm+2)=q(startLForeArm+3)=q(startRForeArm+3)=0.01-0.005;

    calcPostureFromRoot(T_ROOT);

    mHand[R]->setCOMworldRef(0.667,-37,-195,222);
    mHand[L]->setCOMworldRef(1.667,-37, 195,222);

    mTrifidHand[R]->setCOMworldRef(1.13,-40,-210,438);
    mTrifidHand[L]->setCOMworldRef(1.13,-40, 210,438);

    mUArm[R]->setCOMworldRef(1.43,-41,-212,-89);
    mUArm[L]->setCOMworldRef(1.43,-41, 212,-89);

    mNeck->setCOMworldRef(12.8,7,0,715);
    mHead->setCOMworldRef( 2.88,12,0,997);
    mRoot->setCOMworldRef(31.0,19,0,81);

    /////////

    JOINTS(j) q(j)=0.5*(qmin[j]+qmax[j]);

    q(20)=q(21)=0.0;

    qzero=q;

    qzero(startLForeArm+1)=qzero(startRForeArm+1)=0.01+0.005;

    qzero(startLForeArm+2)=qzero(startRForeArm+2)=qzero(startLForeArm+3)=qzero(startRForeArm+3)=0.01-0.005;

    mPosThreshold=DEFAULT_POS_THRESHOLD;

    calcPostureFromRoot(T_ROOT);
}

void KinR1Impl::leftHand2Target(Vec3& Xstar,Quaternion& Qstar,bool oneShot)
{
    static const int NUSED=12;
    static const int NCTRL=20;

    static Matrix Jhand(6,NCTRL);

    static Matrix Jv(3,NUSED);
    static Matrix Jw(3,NUSED);

    static Matrix Vref(3);
    static Matrix Wref(3);

    static Matrix Vrot(3);
    static Matrix Wrot(3);
    static Matrix Grot(2);
    
    static Matrix Vg(2);

    static Matrix lv(3),Rv(3,3),B2Jvt(NUSED,3);
    static Matrix lw(3),Rw(3,3),B2Kwt(NUSED,3);

    static Matrix lg(2),Rg(2,2),B2Jgt(NCTRL,2);

    static const Matrix I=Matrix::id(NUSED);

    static Matrix Nv(NUSED,NUSED);
    static Matrix Kw(3,NUSED);

    static const double GLIM=100.0;
    static const double VLIM=100.0;
    static const double WLIM=20.0;

    static Matrix qz(NCTRL);
    static Matrix qv(NUSED);

    //////////////////////////////////////////

    calcPostureFromRoot(T_ROOT);
    checkG();

    //////////////////////////////////////////

    Vec3 Vstar=Xstar-mHand[L]->Toj.Pj();

    //if (Vstar.mod()<mPosThreshold) ret_code=ON_TARGET;

    Vec3 Wstar=0.5*(Qstar*mHand[L]->Toj.Rj().quaternion().conj()).V;

    Vref=10.0*Vstar;
    Wref=10.0*Wstar;

    /////////////////////////////////////////

    mHand[L]->getJ(Jhand);

    Jv=Jhand.sub(0,3,0,NUSED);
    Jw=Jhand.sub(3,3,0,NUSED);

    ///////////////////////////////////////////////////////
    // CENTER OF MASS
    Gforce.z=0.0;

    double Kg=250.0*Gforce.mod();

    Vg(0)=Kg*Gforce.x; 
    Vg(1)=Kg*Gforce.y;

    B2Jgt=fast_mul_diag_full(B2,Jg.t());
    
    (Jg*B2Jgt).base2(lg,Rg);

    Lgi[0]=1.0/lg(0);
    Lgi[1]=1.0/lg(1);

    Grot=fast_mul_diag_full(Lgi,Rg.t()*Vg);

    if (fabs(Grot(0))>GLIM) Grot(0)=(Grot(0)>0.0?GLIM:-GLIM);
    if (fabs(Grot(1))>GLIM) Grot(1)=(Grot(1)>0.0?GLIM:-GLIM);
    
    qz=B2Jgt*(Rg*Grot);
    //
    ///////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    // REST POSITION
    for (int j=0; j<NCTRL; ++j)
    {
        double s=(q(j)-qzero(j))/((q(j)>qzero(j))?(qmax[j]-qzero(j)):(qmin[j]-qzero(j)));

        if (s>1.0) s=1.0;

        qz(j)+=Kz[j]*(qzero(j)-q(j));

        s=1.0-s*s;

        if (s<0.05) s=0.05;

        B2[j]=s*W2[j];
    }
    //
    ///////////////////////////////////////////////////////

    qz-=Jhand.inv(B2)*(Jhand*qz);

    ///////////////////////////////////////////////////////
    // POSITION PLANNING 
        B2Jvt=fast_mul_diag_full(B2,Jv.t());
        (Jv*B2Jvt).base(lv,Rv);

        for (int k=0; k<3; ++k) Lvi[k]=1.0/lv(k);
        
        Vrot=fast_mul_diag_full(Lvi,Rv.t()*Vref);

        for (int k=0; k<3; ++k) if (fabs(Vrot(k))>VLIM) Vrot(k)=(Vrot(k)>0.0?VLIM:-VLIM);
    
        qv=B2Jvt*(Rv*Vrot);
    //
    ///////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////
    // ROTATION PLANNING
        Wref-=Jw*qv;

        Nv=I-B2Jvt*(Jv*B2Jvt).inv()*Jv;

        Kw=Jw*Nv;

        B2Kwt=fast_mul_diag_full(B2,Kw.t());
        (Jw*B2Kwt).base(lw,Rw);

        for (int k=0; k<3; ++k) Lwi[k]=1.0/lw(k);

        Wrot=fast_mul_diag_full(Lwi,Rw.t()*Wref);

        for (int k=0; k<3; ++k) if (fabs(Wrot(k))>WLIM) Wrot(k)=(Wrot(k)>0.0?WLIM:-WLIM);

        qv+=Nv*(B2Kwt*(Rw*Wrot));
	//
    ///////////////////////////////////////////////////////

    if (oneShot)
    {
        for (int j=0; j<NUSED; ++j) q(j)+=Ks[j]*qv(j);
        for (int j=0; j<NCTRL; ++j) q(j)+=Ks[j]*qz(j);

        limitJointAngles(q);
    }
    else
    {
        qdot.clear();

        for (int j=0; j<NUSED; ++j) qdot(j)+=qv(j);
        for (int j=0; j<NCTRL; ++j) qdot(j)+=qz(j);

        limitJointSpeeds(q,qdot);

        for (int j=0; j<NJOINTS; ++j) qdot(j)*=Kc[j];
    }
}


void KinR1Impl::rightHand2Target(Vec3& Xstar,Quaternion& Qstar,bool oneShot)
{
    static const int NUSED=12;
    static const int NCTRL=20;

    static Matrix Jhand(6,NCTRL);

    static Matrix Jv(3,NUSED);
    static Matrix Jw(3,NUSED);

    static Matrix Vref(3);
    static Matrix Wref(3);

    static Matrix Vrot(3);
    static Matrix Wrot(3);
    static Matrix Grot(2);
    
    static Matrix Vg(2);

    static Matrix lv(3),Rv(3,3),B2Jvt(NUSED,3);
    static Matrix lw(3),Rw(3,3),B2Kwt(NUSED,3);

    static Matrix lg(2),Rg(2,2),B2Jgt(NCTRL,2);

    static const Matrix I=Matrix::id(NUSED);

    static Matrix Nv(NUSED,NUSED);
    static Matrix Kw(3,NUSED);

    static const double GLIM=100.0;
    static const double VLIM=100.0;
    static const double WLIM=20.0;

    static Matrix qz(NCTRL);
    static Matrix qv(NUSED);

    //////////////////////////////////////////

    calcPostureFromRoot(T_ROOT);
    checkG();

    //////////////////////////////////////////

    Vec3 Vstar=Xstar-mHand[R]->Toj.Pj();

    //if (Vstar.mod()<mPosThreshold) ret_code=ON_TARGET;

    Vec3 Wstar=0.5*(Qstar*mHand[R]->Toj.Rj().quaternion().conj()).V;

    Vref=10.0*Vstar;
    Wref=10.0*Wstar;

    /////////////////////////////////////////

    mHand[R]->getJ(Jhand);

    Jv=Jhand.sub(0,3,0,NUSED);
    Jw=Jhand.sub(3,3,0,NUSED);

    ///////////////////////////////////////////////////////
    // CENTER OF MASS
    Gforce.z=0.0;

    double Kg=250.0*Gforce.mod();

    Vg(0)=Kg*Gforce.x; 
    Vg(1)=Kg*Gforce.y;

    B2Jgt=fast_mul_diag_full(B2,Jg.t());
    
    (Jg*B2Jgt).base2(lg,Rg);

    Lgi[0]=1.0/lg(0);
    Lgi[1]=1.0/lg(1);

    Grot=fast_mul_diag_full(Lgi,Rg.t()*Vg);

    if (fabs(Grot(0))>GLIM) Grot(0)=(Grot(0)>0.0?GLIM:-GLIM);
    if (fabs(Grot(1))>GLIM) Grot(1)=(Grot(1)>0.0?GLIM:-GLIM);
    
    qz=B2Jgt*(Rg*Grot);
    //
    ///////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    // REST POSITION
    for (int j=0; j<NCTRL; ++j)
    {
        double s=(q(j)-qzero(j))/((q(j)>qzero(j))?(qmax[j]-qzero(j)):(qmin[j]-qzero(j)));

        if (s>1.0) s=1.0;

        qz(j)+=Kz[j]*(qzero(j)-q(j));

        s=1.0-s*s;

        if (s<0.05) s=0.05;

        B2[j]=s*W2[j];
    }
    //
    ///////////////////////////////////////////////////////

    qz-=Jhand.inv(B2)*(Jhand*qz);

    ///////////////////////////////////////////////////////
    // POSITION PLANNING 
        B2Jvt=fast_mul_diag_full(B2,Jv.t());
        (Jv*B2Jvt).base(lv,Rv);

        for (int k=0; k<3; ++k) Lvi[k]=1.0/lv(k);
        
        Vrot=fast_mul_diag_full(Lvi,Rv.t()*Vref);

        for (int k=0; k<3; ++k) if (fabs(Vrot(k))>VLIM) Vrot(k)=(Vrot(k)>0.0?VLIM:-VLIM);
    
        qv=B2Jvt*(Rv*Vrot);
    //
    ///////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////
    // ROTATION PLANNING
        Wref-=Jw*qv;

        Nv=I-B2Jvt*(Jv*B2Jvt).inv()*Jv;

        Kw=Jw*Nv;

        B2Kwt=fast_mul_diag_full(B2,Kw.t());
        (Jw*B2Kwt).base(lw,Rw);

        for (int k=0; k<3; ++k) Lwi[k]=1.0/lw(k);

        Wrot=fast_mul_diag_full(Lwi,Rw.t()*Wref);

        for (int k=0; k<3; ++k) if (fabs(Wrot(k))>WLIM) Wrot(k)=(Wrot(k)>0.0?WLIM:-WLIM);

        qv+=Nv*(B2Kwt*(Rw*Wrot));
	//
    ///////////////////////////////////////////////////////

    if (oneShot)
    {
        for (int j=0; j<NUSED; ++j) q(j)+=Ks[j]*qv(j);
        for (int j=0; j<NCTRL; ++j) q(j)+=Ks[j]*qz(j);

        limitJointAngles(q);
    }
    else
    {
        qdot.clear();

        for (int j=0; j<NUSED; ++j) qdot(j)+=qv(j);
        for (int j=0; j<NCTRL; ++j) qdot(j)+=qz(j);

        limitJointSpeeds(q,qdot);

        for (int j=0; j<NJOINTS; ++j) qdot(j)*=Kc[j];
    }
}


void KinR1Impl::bothHands2Targets(Vec3& XstarL,Quaternion& QstarL,Vec3& XstarR,Quaternion& QstarR,bool oneShot)
{
    static const int NUSED=20;
    static const int NCTRL=20;

    static Matrix JhandL(6,NUSED);
    static Matrix JhandR(6,NUSED);

    static Matrix Jtot(12,NUSED);

    static Matrix Jv(6,NUSED);
    static Matrix Jw(6,NUSED);

    static Matrix Vref(6);
    static Matrix Wref(6);

    static Matrix Vrot(6);
    static Matrix Wrot(6);
    static Matrix Grot(2);
    
    static Matrix Vg(2);

    static Matrix Lv(6,6),Rv(6,6),B2Jvt(NUSED,6);
    static Matrix Lw(6,6),Rw(6,6),B2Kwt(NUSED,6);

    static Matrix lg(2),Rg(2,2),B2Jgt(NCTRL,2);

    static const Matrix I=Matrix::id(NUSED);

    static Matrix Nv(NUSED,NUSED);
    static Matrix Kw(6,NUSED);

    static const double GLIM=100.0;
    static const double VLIM=100.0;
    static const double WLIM=20.0;

    static Matrix qz(NCTRL);
    static Matrix qv(NUSED);

    //////////////////////////////////////////

    calcPostureFromRoot(T_ROOT);
    checkG();

    //////////////////////////////////////////

    Vec3 VstarL=XstarL-mHand[L]->Toj.Pj();
    Vec3 VstarR=XstarR-mHand[R]->Toj.Pj();

    //if (Vstar.mod()<mPosThreshold) ret_code=ON_TARGET;

    Vec3 WstarL=0.25*(QstarL*mHand[L]->Toj.Rj().quaternion().conj()).V;
    Vec3 WstarR=0.25*(QstarR*mHand[R]->Toj.Rj().quaternion().conj()).V;

    Vref(0)=10.0*VstarL.x; Vref(1)=10.0*VstarL.y; Vref(2)=10.0*VstarL.z;
    Vref(3)=10.0*VstarR.x; Vref(4)=10.0*VstarR.y; Vref(5)=10.0*VstarR.z;

    Wref(0)=10.0*WstarL.x; Wref(1)=10.0*WstarL.y; Wref(2)=10.0*WstarL.z;
    Wref(3)=10.0*WstarR.x; Wref(4)=10.0*WstarR.y; Wref(5)=10.0*WstarR.z;    

    /////////////////////////////////////////

    mHand[L]->getJ(JhandL);
    mHand[R]->getJ(JhandR);

    for (int i=0; i<3; ++i)
    {
        for (int j=0; j<NUSED; ++j)
        {
            Jv(i  ,j)=JhandL(i,j);
            Jv(i+3,j)=JhandR(i,j);

            Jw(i  ,j)=JhandL(i+3,j);
            Jw(i+3,j)=JhandR(i+3,j);
        }
    }

    for (int i=0; i<6; ++i)
    {
        for (int j=0; j<NUSED; ++j)
        {
            Jtot(i  ,j)=Jv(i,j);
            Jtot(i+6,j)=Jw(i,j);
        }
    }

    ///////////////////////////////////////////////////////
    // CENTER OF MASS
    Gforce.z=0.0;
    double Kg=250.0*Gforce.mod();
    Vg(0)=Kg*Gforce.x; 
    Vg(1)=Kg*Gforce.y;
    
    B2Jgt=fast_mul_diag_full(B2,Jg.t());
    
    (Jg*B2Jgt).base2(lg,Rg);

    Lgi[0]=1.0/lg(0);
    Lgi[1]=1.0/lg(1);

    Grot=fast_mul_diag_full(Lgi,Rg.t()*Vg);

    if (fabs(Grot(0))>GLIM) Grot(0)=(Grot(0)>0.0?GLIM:-GLIM);
    if (fabs(Grot(1))>GLIM) Grot(1)=(Grot(1)>0.0?GLIM:-GLIM);
    
    qz=B2Jgt*(Rg*Grot);
    //
    ///////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    // REST POSITION
    for (int j=0; j<NCTRL; ++j)
    {
        double s=(q(j)-qzero(j))/((q(j)>qzero(j))?(qmax[j]-qzero(j)):(qmin[j]-qzero(j)));

        if (s>1.0) s=1.0;

        qz(j)+=Kz[j]*(qzero(j)-q(j));

        s=1.0-s*s;

        if (s<0.05) s=0.05;

        B2[j]=s*W2[j];
    }
    //
    ///////////////////////////////////////////////////////

    qz-=Jtot.inv(B2)*(Jtot*qz);

    ///////////////////////////////////////////////////////
    // POSITION PLANNING 
        B2Jvt=fast_mul_diag_full(B2,Jv.t());
 
        (Jv*B2Jvt).Jacobi(Lv,Rv);

        for (int k=0; k<6; ++k) Lvi[k]=1.0/Lv(k,k);
        
        Vrot=fast_mul_diag_full(Lvi,Rv.t()*Vref);

        for (int k=0; k<6; ++k) if (fabs(Vrot(k))>VLIM) Vrot(k)=(Vrot(k)>0.0?VLIM:-VLIM);
    
        qv=B2Jvt*(Rv*Vrot);
    //
    ///////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////
    // ROTATION PLANNING
        Wref-=Jw*qv;

        Nv=I-B2Jvt*(Jv*B2Jvt).inv()*Jv;

        Kw=Jw*Nv;

        B2Kwt=fast_mul_diag_full(B2,Kw.t());

        (Jw*B2Kwt).Jacobi(Lw,Rw);

        for (int k=0; k<6; ++k) Lwi[k]=1.0/Lw(k,k);

        Wrot=fast_mul_diag_full(Lwi,Rw.t()*Wref);

        for (int k=0; k<6; ++k) if (fabs(Wrot(k))>WLIM) Wrot(k)=(Wrot(k)>0.0?WLIM:-WLIM);

        qv+=Nv*(B2Kwt*(Rw*Wrot));
	//
    ///////////////////////////////////////////////////////

    if (oneShot)
    {
        for (int j=0; j<NUSED; ++j) q(j)+=Ks[j]*(qv(j)+qz(j));

        limitJointAngles(q);
    }
    else
    {
        for (int j=0; j<NUSED; ++j) qdot(j)=qv(j)+qz(j);

        limitJointSpeeds(q,qdot);

        for (int j=0; j<NUSED; ++j) qdot(j)*=Kc[j];
    }
}

void KinR1Impl::checkG()
{
    Matrix Jl(2,20);

    double mass=0.0;

    G[0]=mRoot->getJg(Jl);
    COM=mRoot->Mj*G[0];
    Jg=mRoot->Mj*Jl;
    mass+=mRoot->Mj;

    G[1]=mNeck->getJg(Jl);
    COM+=mNeck->Mj*G[1];
    Jg+=mNeck->Mj*Jl;
    mass+=mNeck->Mj;

    G[2]=mHead->getJg(Jl);
    COM+=mHead->Mj*G[2];
    Jg+=mHead->Mj*Jl;
    mass+=mHead->Mj;

    G[3]=mUArm[L]->getJg(Jl);
    COM+=mUArm[L]->Mj*G[3];
    Jg+=mUArm[L]->Mj*Jl;
    mass+=mUArm[L]->Mj;

    G[4]=mTrifidHand[L]->getJg(Jl);
    COM+=mTrifidHand[L]->Mj*G[4];
    Jg+=mTrifidHand[L]->Mj*Jl;
    mass+=mTrifidHand[L]->Mj;

    G[5]=mHand[L]->getJg(Jl);
    COM+=mHand[L]->Mj*G[5];
    Jg+=mHand[L]->Mj*Jl;
    mass+=mHand[L]->Mj;

    G[6]=mUArm[R]->getJg(Jl);
    COM+=mUArm[R]->Mj*G[6];
    Jg+=mUArm[R]->Mj*Jl;
    mass+=mUArm[R]->Mj;

    G[7]=mTrifidHand[R]->getJg(Jl);
    COM+=mTrifidHand[R]->Mj*G[7];
    Jg+=mTrifidHand[R]->Mj*Jl;
    mass+=mTrifidHand[R]->Mj;

    G[8]=mHand[R]->getJg(Jl);
    COM+=mHand[R]->Mj*G[8];
    Jg+=mHand[R]->Mj*Jl;
    mass+=mHand[R]->Mj;

    COM/=mass;
    Jg/=mass;

    for (int j=0; j<4; ++j) for (int k=0; k<2; ++k)
    {
        Jg(k,startLForeArm+j)=0.0;
        Jg(k,startRForeArm+j)=0.0;
    }

    COM.z=0.0;

    Vec3 C1( 0.055+0.074, 0.110, 0.0);
    Vec3 W1(-0.074+0.074, 0.170, 0.0);
    Vec3 C2(-0.244+0.074, 0.000, 0.0);
    Vec3 W2(-0.074+0.074,-0.170, 0.0);
    Vec3 C3( 0.055+0.074,-0.110, 0.0);

    Vec3 G0=(W1+W2)/2.0;

    Vec3 L0=(W1-C1).norm();
    Vec3 L1=(C2-W1).norm();
    Vec3 L2=(W2-C2).norm();
    Vec3 L3=(C3-W2).norm();
    Vec3 L4=(C1-C3).norm();

    Vec3 D0=COM-C1,D1=COM-W1,D2=COM-C2,D3=COM-W2,D4=COM-C3;

    double d0=(D0%L0).z,d1=(D1%L1).z,d2=(D2%L2).z,d3=(D3%L3).z,d4=(D4%L4).z;

    Gforce=G0-COM;

    if (d0>=0.0 || d1>=0.0 || d2>=0.0 || d3>=0.0 || d4>=0.0) return;

    D0=D0-(D0*L0)*L0;
    D1=D1-(D1*L1)*L1;
    D2=D2-(D2*L2)*L2;
    D3=D3-(D3*L3)*L3;
    D4=D4-(D4*L4)*L4;

    d0=D0.mod(); d1=D1.mod(); d2=D2.mod(); d3=D3.mod(); d4=D4.mod();

    const double LIMIT=0.03;

    if (d0>LIMIT && d1>LIMIT && d2>LIMIT && d3>LIMIT && d4>LIMIT)
    {
        qbalanced=q;
        COMbalanced=COM;
    }
}

///////////////////////////////////////////



