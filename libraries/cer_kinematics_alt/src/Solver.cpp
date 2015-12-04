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
	qz(12)
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
		Ks[j]=0.5*RAD2DEG;
		Kz[j]=0.01*DEG2RAD;
	}

	for (int j=0; j<=2; ++j) 
	{
		int k=startLForeArm+1+j;
		int h=startRForeArm+1+j;

		Q[j]=0.020;
		Q[h]=Q[k]=0.014;
		Ks[j]=Ks[k]=Ks[h]=0.5;
		Kz[j]=Kz[h]=Kz[k]=0.01;
	}

	for (int j=0; j<12; ++j)
	{
		S(j,j)=Q[j]*Q[j];
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

	Lva(0,0)=lva(0)/(lva(0)*lva(0)+0.000004); 
	Lva(1,1)=lva(1)/(lva(1)*lva(1)+0.000004);
	Lva(2,2)=lva(2)/(lva(2)*lva(2)+0.000004);

	//T=AJvt*(Rva*Lva*Rva.t());
	//qv=T*Vref;

	qv=AJvt*(Rva*(Lva*(Rva.t()*Vref)));

	///////////////////////////////////////////////

	static Matrix I=Matrix::id(12);

	Z=I-Jv.inv(S)*Jv;

	//O=Z*A*Z.t();
	//O=Z*fast_mul_diag_full(A,Z.t());
	//OJwt=O*Jw.t();
	//OJwt=Z*(A*(Z.t()*Jw.t()));
	OJwt=Z*fast_mul_diag_full(A,(Jw*Z).t());

	//static Matrix Kw(3,12); Kw=Jw*Z;
	//static Matrix AKwt(12,3); AKwt=A*Kw.t();

	static Matrix lwa(3),Rwa(3,3);

	//(Kw*AKwt).base(lwa,Rwa);
	(Jw*OJwt).base(lwa,Rwa);

	Lwa(0,0)=lwa(0)/(lwa(0)*lwa(0)+0.1); 
	Lwa(1,1)=lwa(1)/(lwa(1)*lwa(1)+0.1);
	Lwa(2,2)=lwa(2)/(lwa(2)*lwa(2)+0.1);

	//qv+=Z*(AKwt*(Rwa*(Lwa*(Rwa.t()*(Wref-Jw*qv)))));

	qv+=OJwt*(Rwa*(Lwa*(Rwa.t()*(Wref-Jw*qv))));

	////////////////////////////////////////

	for (int j=0; j<12; ++j) q(j)+=Ks[j]*(qv(j)+qz(j));

	limitJointAngles(q);

	q_valid=q;

	return IN_PROGRESS;
}

#if 0
int Robot::leftHand2Target(Vec3& Xstar,Quaternion& Qstar,bool bExtendTorso,bool bExtendHand,bool verbose)
{
	calcPostureFromRoot(T_ROOT);

	//////////////////////////////////////////

	Vec3 Vstar=Xstar-mHand[L]->Toj.Pj();

	if (Vstar.mod()<0.005) return ON_TARGET;

	Vec3 Wstar=(Qstar*mHand[L]->Toj.Rj().quaternion().conj()).V;
	
	static Matrix Vref(3); Vref=25.0*Vstar;
	static Matrix Wref(3); Wref=50.0*Wstar;

	/////////////////////////////////////////

	static Matrix Jhand(6,NJOINTS);

	mHand[L]->getJ(Jhand);

	static Matrix J(6,12);

	J=Jhand.sub(0,6,0,12);

	static Matrix Jv(3,12); Jv=J.sub(0,3,0,12);
	static Matrix Jw(3,12); Jw=J.sub(3,3,0,12);

	//////////////////////

	static Matrix A(12,12);

	static Matrix qz(12);

	for (int j=0; j<12; ++j)
	{
		double s=(q(j)-qzero(j))/((q(j)>qzero(j))?(qmax[j]-qzero(j)):(qmin[j]-qzero(j)));

		if (s>1.0) s=1.0;

		A(j,j)=sqrt(1.0-s*s)*S(j,j);

		qz(j)=Kz[j]*(qzero(j)-q(j));
	}

	qz-=J.inv(S)*(J*qz);
	//qz-=S*(J.t()*((J*S*J.t()).inv()*(J*qz)));

	///////////////////////////////////////////////

	static Matrix AJvt(12,3); AJvt=A*Jv.t(); 

	static Matrix lva(3),Rva(3,3);

	(Jv*AJvt).base(lva,Rva);

	static Matrix Lva(3,3); 

	Lva(0,0)=lva(0)/(lva(0)*lva(0)+0.000004); 
	Lva(1,1)=lva(1)/(lva(1)*lva(1)+0.000004);
	Lva(2,2)=lva(2)/(lva(2)*lva(2)+0.000004);

	static Matrix Avi(12,3); Avi=AJvt*(Rva*Lva*Rva.t());

	///////////////////////////////////////////////

	static Matrix I=Matrix::id(12);

	static Matrix Ns(12,12); Ns=I-Jv.inv(S)*Jv;

	static Matrix Kw(3,12); Kw=Jw*Ns;

	static Matrix Yref(3); Yref=Wref-Jw*(Avi*Vref);

	////////////////////////////////////////
	
	static Matrix AKwt(12,3); AKwt=A*Kw.t();

	static Matrix lwa(3),Rwa(3,3);

	(Kw*AKwt).base(lwa,Rwa);

	static Matrix Lwa(3,3);
	Lwa(0,0)=lwa(0)/(lwa(0)*lwa(0)+0.01); 
	Lwa(1,1)=lwa(1)/(lwa(1)*lwa(1)+0.01);
	Lwa(2,2)=lwa(2)/(lwa(2)*lwa(2)+0.01);

	//static Matrix Awi(12,3); Awi=AKwt*(Rwa*Lwa*Rwa.t());

	////////////////////////////////////////

	//static Matrix qv(12); qv=Avi*Vref+Ns*(Awi*Yref);

	static Matrix qv(12); qv=Avi*Vref+Ns*(AKwt*(Rwa*(Lwa*(Rwa.t()*Yref))));

	for (int j=0; j<12; ++j) q(j)+=Ks[j]*(qv(j)+qz(j));

	limitJointAngles(q);

	q_valid=q;

	return IN_PROGRESS;
}

#endif

#if 0

int Robot::leftHand2Target(Vec3& Xstar,Quaternion& Qstar,bool bExtendTorso,bool bExtendHand,bool verbose)
{
	calcPostureFromRoot(T_ROOT);

	//////////////////////////////////////////

	Vec3 Vstar=Xstar-mHand[L]->Toj.Pj();

	//if (Vstar.mod()<0.005) return ON_TARGET;

	Vec3 Wstar=(Qstar*mHand[L]->Toj.Rj().quaternion().conj()).V;
	
	/////////////////////////////////////////

	static Matrix Jhand(6,NJOINTS);

	mHand[L]->getJ(Jhand);

	static Matrix J(6,12);

	J=Jhand.sub(0,6,0,12);

	static Matrix Jv(3,12); Jv=J.sub(0,3,0,12);
	static Matrix Jw(3,12); Jw=J.sub(3,3,0,12);

	//////////////////////

	static Matrix A(12,12);

	static Matrix qz(12);

	for (int j=0; j<12; ++j)
	{
		double s=(q(j)-qzero(j))/((q(j)>qzero(j))?(qmax[j]-qzero(j)):(qmin[j]-qzero(j)));

		if (s>1.0) s=1.0;

		A(j,j)=(1.0-s*s)*S(j,j);

		qz(j)=Uq[j]*(qzero(j)-q(j));
	}

	Vec3 V[3];

	//////////////////////

	static Matrix qv(12);

	static Matrix AJvt(12,3); AJvt=A*Jv.t(); 
	static Matrix JvAJvt(3,3); JvAJvt=Jv*AJvt;

	Matrix lv=JvAJvt.eigen();

	Matrix Lv(3,3); 
	Lv(0,0)=lv(0)/(lv(0)*lv(0)+0.000004); 
	Lv(1,1)=lv(1)/(lv(1)*lv(1)+0.000004);
	Lv(2,2)=lv(2)/(lv(2)*lv(2)+0.000004);

	for (int d=0; d<2; ++d)
	{
		double Kx=JvAJvt(1,2)*(lv(d)-JvAJvt(0,0))+JvAJvt(2,0)*JvAJvt(0,1);
		double Ky=JvAJvt(2,0)*(lv(d)-JvAJvt(1,1))+JvAJvt(0,1)*JvAJvt(1,2);
		double Kz=JvAJvt(0,1)*(lv(d)-JvAJvt(2,2))+JvAJvt(1,2)*JvAJvt(2,0);

		if (fabs(Kx)<=fabs(Ky) && fabs(Kx)<=fabs(Kz))
		{
			V[d].x=1.0; V[d].y=Kx/Ky; V[d].z=Kx/Kz; V[d].normalize();
		}
		else if (fabs(Ky)<=fabs(Kz) && fabs(Ky)<=fabs(Kx))
		{
			V[d].y=1.0; V[d].x=Ky/Kx; V[d].z=Ky/Kz; V[d].normalize();
		}
		else
		{
			V[d].z=1.0; V[d].y=Kz/Ky; V[d].x=Kz/Kx; V[d].normalize();
		}
	}

	V[2]=V[0]%V[1];

	Matrix Rv(3,3);

	Rv(0,0)=V[0].x; Rv(0,1)=V[1].x; Rv(0,2)=V[2].x;
	Rv(1,0)=V[0].y; Rv(1,1)=V[1].y; Rv(1,2)=V[2].y;
	Rv(2,0)=V[0].z; Rv(2,1)=V[1].z; Rv(2,2)=V[2].z;

	///////////////////////////////////////////////
	
	static Matrix qw(12);

	static Matrix AJwt(12,3); AJwt=A*Jw.t(); 
	static Matrix JwAJwt(3,3); JwAJwt=Jw*AJwt;

	Matrix lw=JwAJwt.eigen();

	Matrix Lw(3,3); 
	Lw(0,0)=lw(0)/(lw(0)*lw(0)+0.000004); 
	Lw(1,1)=lw(1)/(lw(1)*lw(1)+0.000004);
	Lw(2,2)=lw(2)/(lw(2)*lw(2)+0.000004);

	for (int d=0; d<2; ++d)
	{
		double Kx=JwAJwt(1,2)*(lw(d)-JwAJwt(0,0))+JwAJwt(2,0)*JwAJwt(0,1);
		double Ky=JwAJwt(2,0)*(lw(d)-JwAJwt(1,1))+JwAJwt(0,1)*JwAJwt(1,2);
		double Kz=JwAJwt(0,1)*(lw(d)-JwAJwt(2,2))+JwAJwt(1,2)*JwAJwt(2,0);

		if (fabs(Kx)<=fabs(Ky) && fabs(Kx)<=fabs(Kz))
		{
			V[d].x=1.0; V[d].y=Kx/Ky; V[d].z=Kx/Kz; V[d].normalize();
		}
		else if (fabs(Ky)<=fabs(Kz) && fabs(Ky)<=fabs(Kx))
		{
			V[d].y=1.0; V[d].x=Ky/Kx; V[d].z=Ky/Kz; V[d].normalize();
		}
		else
		{
			V[d].z=1.0; V[d].y=Kz/Ky; V[d].x=Kz/Kx; V[d].normalize();
		}
	}

	V[2]=V[0]%V[1];

	Matrix Rw(3,3);

	Rw(0,0)=V[0].x; Rw(0,1)=V[1].x; Rw(0,2)=V[2].x;
	Rw(1,0)=V[0].y; Rw(1,1)=V[1].y; Rw(1,2)=V[2].y;
	Rw(2,0)=V[0].z; Rw(2,1)=V[1].z; Rw(2,2)=V[2].z;

	///////////////////////////////////////////////

	//JWJt.dump();
	//(R*L*R.t()).dump();

	static Matrix I=Matrix::id(12);

	Matrix Jvi=AJvt*(Rv*Lv*Rv.t());
	Matrix Jwi=AJwt*(Rw*Lw*Rw.t());

	Matrix Kw=Jw*(I-Jvi*Jv);

	qz -= S*J.t()*(J*S*J.t()).inv()*J*qz;
	qv = qz + Jvi*(Matrix)(25.0*Vstar) + Kw.inv()*((Matrix)(75.0*Wstar) - Jw*Jvi*(Matrix)(25.0*Vstar));// + (I-Jvi*Jv)*(I-Kw.inv()*Kw)*qz;

	for (int j=0; j<12; ++j) q(j)+=Kq[j]*qv(j);

	limitJointAngles(q);

	q_valid=q;

	return IN_PROGRESS;
}

#endif

#if 0
int Robot::leftHand2Target(Vec3& Xstar,Quaternion& Qstar,bool bExtendTorso,bool bExtendHand)
{
	calcPostureFromRoot(T_ROOT);

	//////////////////////////////////////////

	Vec3 Xerr=Xstar-mHand[L]->Toj.Pj();

	double Xd=Xerr.mod();

	Vec3 Werr=(Qstar*mHand[L]->Toj.Rj().quaternion().conj()).V;

	double Wd=Werr.mod();

	if (Xd<0.005) return ON_TARGET;
	
	/////////////////////////////////////////

	static Matrix J(6,NJOINTS);

	mHand[L]->getJ(J);

	static double S[NJOINTS];

	JOINTS(j)
	{
		qp(j)=(qzero(j)-q(j));

		double s=qp(j)/((qp(j)>0.0)?(qzero(j)-qmin[j]):(qmax[j]-qzero(j)));

		S[j]=1.0-s*s;

		qp(j)*=Uq[j];
	}

	// forearm
	{
		double s=-mTrifidHand[L]->angle()/35.0;
		s=1.0-s*s;

		for (int j=startLForeArm+1; j<=startLForeArm+3; ++j) if (s<S[j]) S[j]=s;
	}

	//waist
	{
		double s=-mTrifidTorso->angle()/30.0;
		s=1.0-s*s;

		for (int j=0; j<=2; ++j) if (s<S[j]) S[j]=s;
	}

	JOINTS(j) if (S[j]<0.0) S[j]=0.0;

	//JOINTS(j) S[j]=1.0;
	
	static Matrix SW(NJOINTS,NJOINTS);
	JOINTS(j) SW(j,j)=S[j]*W[j];

	/////////////////////////////
	//static Matrix Gerr(2); 
	//Gerr(0)=Gforce.x; 
	//Gerr(1)=Gforce.y;
	//qp+=Jg.inv()*Gerr;
	//qp+=Jg.t()*((Jg*Jg.t()).inv()*Gerr);
	//qp-=Jhand.inv()*(Jhand*qp);
	/////////////////////////////

	static Matrix SWJt(NJOINTS,6); SWJt=SW*J.t();
	static Matrix JSWJt(6,6);      JSWJt=J*SWJt;

	double detJ=JSWJt.det();

	/*
	if (detJ>0.000001)
	{
		static Matrix Err(6);
		Err(0)=Xerr.x; Err(1)=Xerr.y; Err(2)=Xerr.z;
		Err(3)=Werr.x; Err(4)=Werr.y; Err(5)=Werr.z;

		qp+=SWJt*(JSWJt.inv()*(Err-J*qp));
	}
	else
	*/
	{
		static Matrix Jv(3,NJOINTS); Jv=J.sub(0,3,0,NJOINTS);
		static Matrix Jw(3,NJOINTS); Jw=J.sub(3,3,0,NJOINTS);
		/*
		static Matrix SWJvt(NJOINTS,3); SWJvt=SW*Jv.t();
		static Matrix SWJwt(NJOINTS,3); SWJwt=SW*Jw.t();

		static Matrix JvSWJvt(3,3); JvSWJvt=Jv*SWJvt;
		static Matrix JwSWJwt(3,3); JwSWJwt=Jw*SWJwt;

		double detJv=JvSWJvt.det();
		double detJw=JwSWJwt.det();

		if (detJv<0.000001) return OUT_OF_REACH;
		*/

		static Matrix w(NJOINTS,NJOINTS);
		JOINTS(j) w(j,j)=W[j];

		static Matrix WJvt(NJOINTS,3); WJvt=w*Jv.t();

		static Matrix JvWJvt(3,3); JvWJvt=Jv*WJvt;

		double detJv=JvWJvt.det();

		if (detJv<0.000001) return OUT_OF_REACH;

		//qp.clear();

		//qp+=SWJwt*(JwSWJwt.inv()*((Matrix)Werr-Jw*qp));
		//qp+=SWJvt*(JvSWJvt.inv()*((Matrix)Xerr-Jv*qp));

		//qp-=w*J.t()*((J*w*J.t()).inv()*(J*qp));
		qp.clear();
		qp+=w*Jv.t()*((Jv*w*Jv.t()).inv()*((Matrix)Xerr-Jv*qp));
		//printf("%f    %f\n",detJv,detJw);
	}

	/*
	static Matrix Jv(3,NJOINTS);
	static Matrix Jw(3,NJOINTS);

	Jv=J.sub(0,3,0,NJOINTS);
	Jw=J.sub(3,3,0,NJOINTS);

	if (false)
	{
		static Matrix M(3,3);
		M(0,0)=M(1,1)=M(2,2)=0.02;
		static Matrix I=Matrix::id(NJOINTS);
		Matrix Jvi=SW*Jv.t()*(Jv*SW*Jv.t()+M).inv();
		Matrix Tw=Jw*(I-Jvi*Jv);
		Matrix Twi=SW*Tw.t()*(Tw*SW*Tw.t()+M).inv();
		qp = Jvi*(Matrix)Xerr + Twi*((Matrix)Werr-Jw*Jvi*(Matrix)Xerr) + (I-Jvi*Jv)*(I-Twi*Tw)*qp;
	}
	else
	{
		static Matrix SWJvt(NJOINTS,3);
		static Matrix SWJwt(NJOINTS,3);
	
		SWJvt=SW*Jv.t();
		SWJwt=SW*Jw.t();

		static Matrix JSWvi(NJOINTS,3);
		static Matrix JSWwi(NJOINTS,3);

		static Matrix JvSWJvt(3,3);
		static Matrix JwSWJwt(3,3);

		JvSWJvt=Jv*SWJvt;
		JwSWJwt=Jw*SWJwt;

		double dv=JvSWJvt.det();
		double dw=JwSWJwt.det();

		printf("%f    %f\n",dv,dw);

		JSWvi=SWJvt*(Jv*SWJvt).inv();
		JSWwi=SWJwt*(Jw*SWJwt).inv();

		qp+=JSWwi*((Matrix)(10.0*Werr)-Jw*qp);
		qp+=JSWvi*((Matrix)Xerr-Jv*qp);
	}
	*/

	// HEAD attitude
	{
		Vec3 DirH=mHead->Toj.Rj().Ex();
		Vec3 DirT=(Xstar-mHead->Toj.Pj()).norm();

		Vec3 Werr=0.666*(DirH%DirT);

		qp(startNeck  )+=Werr*mJoint[nj-2]->Toj.Rj().Ez();
		qp(startNeck+1)+=Werr*mJoint[nj-1]->Toj.Rj().Ez();
	}

	mTrifidTorso->checkAngle(30.0,q,qp);
	mTrifidHand[L]->checkAngle(35.0,q,qp);
	
	limitJointSpeeds(q,qp);

	JOINTS(j) q(j)+=Kq[j]*qp(j);

	mTrifidTorso->setExtension(q,qzero(0));
	mTrifidHand[L]->setExtension(q,qzero(startLForeArm+1));
	
	limitJointAngles(q);

	//if (((Vec3)(Jw*qp)).mod()<0.1*Xerr.mod()) return OUT_OF_REACH;

	//if (detJ<0.000001) return OUT_OF_REACH;

	q_valid=q;

	if (Xd<0.005) return ON_TARGET;

	//if (Xd<0.005 && (Wd<0.05 || ((Vec3)(Jw*qp)).mod()<0.1*Werr.mod())) return ON_TARGET;

	return IN_PROGRESS;
}
#endif