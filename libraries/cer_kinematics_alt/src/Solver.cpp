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

#include <cer_kinematics_alt/Solver.h>

using namespace cer::kinematics2;

//Vec3 Werr=(mHand[L]->Toj.Rj().Ex()%Uxstar)+(mHand[L]->Toj.Rj().Ey()%Uystar)+(mHand[L]->Toj.Rj().Ez()%Uzstar);

LeftSideSolver::LeftSideSolver() : 
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

		//mHand[L] = new Component(Transform(0,20,0,0,0,0),mTrifidHand[L]);
		mHand[L] = new Component(Transform(0,0,0,0,0,70),mTrifidHand[L]);

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

		//mHand[R] = new Component(Transform(0,20,0,0,0,0),mTrifidHand[R]);
		mHand[R] = new Component(Transform(0,0,0,0,0,70),mTrifidHand[R]);

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




#if 0
int LeftSideSolver::leftHand2Target(Vec3& Xstar,Quaternion& Qstar)
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

	///////////////////////////////////////////////

	Matrix lv(3),Rv(3,3),Lv(3,3);
	(Jv*A*Jv.t()).base(lv,Rv);

	Lv(0,0)=1.0/lv(0);
	Lv(1,1)=1.0/lv(1);
	Lv(2,2)=1.0/lv(2);

	Matrix Vrot=Lv*Rv.t()*Vref;

	if (fabs(Vrot(0))>5.0) Vrot(0)=(Vrot(0)>0.0?5.0:-5.0);
	if (fabs(Vrot(1))>5.0) Vrot(1)=(Vrot(1)>0.0?5.0:-5.0);
	if (fabs(Vrot(2))>5.0) Vrot(2)=(Vrot(2)>0.0?5.0:-5.0);
	
	qv=A*Jv.t()*Rv*Vrot;

	////////////////////////////////////////

	for (int j=0; j<12; ++j) q(j)+=Ks[j]*(qv(j)+qz(j));

	limitJointAngles(q);

	q_valid=q;

	return IN_PROGRESS;
}
#endif


int LeftSideSolver::leftHand2Target(Vec3& Xstar,Quaternion& Qstar)
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
