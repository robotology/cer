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

#ifndef __ROBOT_CONTROL_H__
#define __ROBOT_CONTROL_H__

#include <Geometry.h>
#include <Matrix.h>
#include <RobotModel.h>

#define MAX_NJOINTS 32

#define VMAX 0.6
#define WMAX 1.0

#define STOP_DISTANCE 0.05

/*
#define TORSO_RADIUS 0.090 // [m]
#define ARM_RADIUS   0.018 // [m]

#define TORSO_MAX_TILT          30.0  // [deg]
#define MIN_TORSO_EXTENSION    -0.05  // [m]
#define MAX_TORSO_EXTENSION     0.15  // [m]

#define DEFAULT_ARM_EXTENSION   0.02  // [m]
#define DEFAULT_TORSO_EXTENSION 0.0   // [m]

#define MIN_ARM_EXTENSION       0.0   // [m]
#define MAX_ARM_EXTENSION       0.14  // [m]
#define WRIST_MAX_TILT          35.0  // [deg]
#define WRIST_TILT_ZERO         0.0 //15.0  // [deg]

#define PERIOD 0.01
*/

using namespace cer::robot_model;

namespace cer {
namespace kinematics_alt {

class RobotController
{
public:
	RobotController(RobotModel* rm)
	{
		robotModel = rm;
	}

	virtual ~RobotController()
	{
	}

	virtual void velControl(Matrix &qin, Matrix &qdotout, double *Vl, double *Wl, double *Vr, double *Wr)
	{
		int N = NJOINTS;

		int Mv = 0;
		int Mw = 0;

		if (Vl) Mv += 3;
		if (Vr) Mv += 3;
		if (Wl) Mw += 3;
		if (Wr) Mw += 3;

		Matrix V(Mv);
		Matrix W(Mw);

		Matrix Jv(Mv, N);
		Matrix Jw(Mw, N);

		robotModel->calcConfig(qin);

		const Matrix& JhandL = robotModel->calcHandJacobian(RobotModel::L);
		const Matrix& JhandR = robotModel->calcHandJacobian(RobotModel::R);

		{
			int m = 0;

			if (Vl)
			{
				for (int i = 0; i < 3; ++i)
				{
					for (int j = 0; j < N; ++j)
					{
						Jv(i, j) = JhandL(i, j);
					}
				}

				Vec3 v(Vl);

				if (v.mod() > VMAX) v.normalize(VMAX);

				V(m++) = v.x; V(m++) = v.y; V(m++) = v.z;
			}

			if (Vr)
			{
				for (int i = 0; i < 3; ++i)
				{
					for (int j = 0; j < N; ++j)
					{
						Jv(m + i, j) = JhandR(i, j);
					}
				}

				Vec3 v(Vr);

				if (v.mod() > VMAX) v.normalize(VMAX);

				V(m++) = v.x; V(m++) = v.y; V(m++) = v.z;
			}
		}
		{
			int m = 0;

			if (Wl)
			{
				for (int i = 0; i < 3; ++i)
				{
					for (int j = 0; j < N; ++j)
					{
						Jw(i, j) = JhandL(i + 3, j);
					}
				}

				Vec3 w(Wl);

				if (w.mod() > WMAX) w.normalize(WMAX);

				W(m++) = w.x; W(m++) = w.y; W(m++) = w.z;
			}

			if (Wr)
			{
				for (int i = 0; i < 3; ++i)
				{
					for (int j = 0; j < N; ++j)
					{
						Jw(m + i, j) = JhandR(i + 3, j);
					}
				}

				Vec3 w(Wr);

				if (w.mod() > WMAX) w.normalize(WMAX);

				W(m++) = w.x; W(m++) = w.y; W(m++) = w.z;
			}
		}

		static Matrix qz(N);

		////////////////////////////////
		// zero
		{
			for (int j = 0; j<N; ++j)
			{
				double delta = qin(j) - qzero(j);

				double s = delta / ((delta > 0.0) ? (qmax(j) - qzero(j)) : (qmin(j) - qzero(j)));

				qz(j) = -s*Kz[j] * delta;

				s = 1.0 - s*s;

				if (s<0.05) s = 0.05;

				B2[j] = s*W2[j];
				//B2[j] = W2[j];
			}
		}
		//
		////////////////////////////////

		static Matrix distance;

		Matrix qv(N);

		////////////////////////////////
		// probe position move
		if (Vl || Vr)
		{
			static const double VLIM = 10.0;

			solveV(qv, Jv, V, VLIM);
		}

		const Matrix &Jn = robotModel->calcInterference(distance);

		Matrix Vov = Jn*qv;

		int ncriticals = 0;

		static int rcrit[MAX_NJOINTS];
		static double vcrit[MAX_NJOINTS];

		for (int r = 0; r < distance.R; ++r)
		{
			double delta = STOP_DISTANCE - distance(r);

			//double delta = STOP_DISTANCE - distance(r);

			if (delta > 0.0 && Vov(r) < 0.0)
			{
				rcrit[ncriticals] = r;
				vcrit[ncriticals] = 10.0*delta;
				++ncriticals;
			}
		}

		Matrix P = Matrix::id(N);

		Matrix q(N);

		////////////////////////////////
		// obstacle avoidance
		if (ncriticals)
		{
			static const double OLIM = 10.0;

			Matrix J(ncriticals, N);
			Matrix V(ncriticals);

			for (int r = 0; r < ncriticals; ++r)
			{
				V(r) = vcrit[r];

				for (int j = 0; j < N; ++j)
				{
					J(r, j) = Jn(rcrit[r], j);
				}
			}

			solveO(q, J, V, OLIM, P);
		}

		const Matrix& Jg = robotModel->calcGravity(G);

		static Vec3 Gforce;

		double margin = robotModel->getBalancing(Gforce);

		bool balanced = margin > 0.05;

		if (!balanced)
		{
			//printf("UNBALANCED %f\n", 100.0*margin);

			Matrix Vgv = Jg*qv;

			if (Vgv(0)*Gforce.x + Vgv(1)*Gforce.y > 0.0)
			{
				balanced = true;
			}
		}

		////////////////////////////////
		// balance
		if (!balanced && P.det() != 0.0)
		{
			static const double KG = 100.0;
			static const double GLIM = 10.0;

			static Matrix V(2);

			double Kg = KG*Gforce.mod();
			V(0) = Kg*Gforce.x;
			V(1) = Kg*Gforce.y;

			V -= Jg*q;

			Matrix H = Jg*P;

			solveG(q, H, V, GLIM, P);
		}
		////////////////////////////////

		// target control
		ikin_VW(q, Jv, Jw, V, W, P, (Vl || Vr), (Wl || Wr));

		////////////////////////////////
		if (false && balanced)
		{
			static const double KG = 1.0;
			static const double GLIM = 10.0;

			if (margin < 0.08)
			{
				double Kg = (0.08-margin)*KG*Gforce.mod();

				static Matrix V(2);

				V(0) = Kg*Gforce.x;
				V(1) = Kg*Gforce.y;

				solveG(qz, Jg, V, GLIM);
			}
		}
		
		////////////////////////////////
		// balance
		if (balanced && P.det() != 0.0)
		{
			static const double KG = 10.0;
			static const double GLIM = 10.0;

			//if (margin < 0.08)
			{
				//printf("balancing %f\n", 100.0*margin);

				//double Kg = (0.08-margin)*KG*Gforce.mod();
				
				double Kg = KG*Gforce.mod();

				static Matrix V(2);

				V(0) = Kg*Gforce.x;
				V(1) = Kg*Gforce.y;

				V -= Jg*q;

				Matrix H = Jg*P;

				solveG(q, H, V, GLIM, P);
			}
		}
		////////////////////////////////

		if (!ncriticals)
		{
			static const double OLIM = 10.0;

			Matrix V(distance.R);

			for (int r = 0; r < distance.R; ++r)
			{
				V(r) = 0.05*0.05 / distance(r);

				if (fabs(V(r)) < 0.0005) V(r) = 0.0;
			}

			solveO(qz, Jn, V, OLIM);
		}

		q += P*qz;

		for (int j = 0; j<N; ++j) qdotout(j) = Kc[j] * q(j);

		joint_limits(qin, qdotout);
	}

	Vec3 getCOM(){ return G; }

	const Matrix& getZeroConfig(){ return qzero; }

protected:
	RobotModel *robotModel;
	int NJOINTS;

	Vec3 G;

	Matrix qzero;
	Matrix qmax;
	Matrix qmin;
	Matrix qdot_max;

	double W2[MAX_NJOINTS];
	double B2[MAX_NJOINTS];

	double Kc[MAX_NJOINTS];
	double Kz[MAX_NJOINTS];

	double acc_max[MAX_NJOINTS];
	double dec_max[MAX_NJOINTS];

	void solveV(Matrix &qdot, const Matrix &J, Matrix &V, double VLIM)
	{
		static Matrix L, R;
		L.resize(V.R, V.R, false);
		R.resize(V.R, V.R, false);

		static Matrix B2Jt;
		B2Jt.resize(NJOINTS, V.R, false);

		B2Jt = fast_mul_diag_full(B2, J.t());

		(J*B2Jt).Jacobi(L, R);

		static double Li[MAX_NJOINTS];

		for (int p = 0; p < V.R; ++p)
		{
			//if (L(p, p) < 0.001) Li[p] = 0.0; else Li[p] = 1.0 / L(p, p);

			Li[p] = L(p, p) / (L(p, p)*L(p, p) + 0.001*0.0001);
		}

		static Matrix Vrot;
		Vrot.resize(V.R, 1, false);
		Vrot = fast_mul_diag_full(Li, R.t()*V);

		for (int p = 0; p < V.R; ++p)
		{
			if (Vrot(p) < -VLIM) Vrot(p) = -VLIM; else if (Vrot(p) > VLIM) Vrot(p) = VLIM;
		}

		qdot += B2Jt*(R*Vrot);
	}

	void solveG(Matrix &qdot, const Matrix &J, Matrix &V, double VLIM)
	{
		static Matrix L, R;
		L.resize(V.R, V.R, false);
		R.resize(V.R, V.R, false);

		static Matrix B2Jt;
		B2Jt.resize(NJOINTS, V.R, false);

		B2Jt = fast_mul_diag_full(B2, J.t());

		(J*B2Jt).Jacobi(L, R);

		static double Li[MAX_NJOINTS];

		for (int p = 0; p < V.R; ++p)
		{
			//if (L(p, p) < 0.001) Li[p] = 0.0; else Li[p] = 1.0 / L(p, p);

			Li[p] = L(p, p) / (L(p, p)*L(p, p) + 0.001*0.0001);
		}

		static Matrix Vrot;
		Vrot.resize(V.R, 1, false);
		Vrot = fast_mul_diag_full(Li, R.t()*V);

		for (int p = 0; p < V.R; ++p)
		{
			if (Vrot(p) < -VLIM) Vrot(p) = -VLIM; else if (Vrot(p) > VLIM) Vrot(p) = VLIM;
		}

		qdot += B2Jt*(R*Vrot);
	}

	void solveO(Matrix &qdot, const Matrix &J, Matrix &V, double VLIM)
	{
		static Matrix L, R;
		L.resize(V.R, V.R, false);
		R.resize(V.R, V.R, false);

		static Matrix B2Jt;
		B2Jt.resize(NJOINTS, V.R, false);

		B2Jt = fast_mul_diag_full(B2, J.t());

		(J*B2Jt).Jacobi(L, R);

		static double Li[MAX_NJOINTS];

		for (int p = 0; p < V.R; ++p)
		{
			//if (L(p, p) < 0.001) Li[p] = 0.0; else Li[p] = 1.0 / L(p, p);

			Li[p] = L(p, p) / (L(p, p)*L(p, p) + 0.001*0.0001);
		}

		static Matrix Vrot;
		Vrot.resize(V.R, 1, false);
		Vrot = fast_mul_diag_full(Li, R.t()*V);

		for (int p = 0; p < V.R; ++p)
		{
			if (Vrot(p) < -VLIM) Vrot(p) = -VLIM; else if (Vrot(p) > VLIM) Vrot(p) = VLIM;
		}

		qdot += B2Jt*(R*Vrot);
	}

	void solveV(Matrix &qdot, const Matrix &J, Matrix &V, double VLIM, Matrix& P)
	{
		static Matrix L, R;
		L.resize(V.R, V.R, false);
		R.resize(V.R, V.R, false);

		static Matrix B2Jt;
		B2Jt.resize(NJOINTS, V.R, false);

		B2Jt = fast_mul_diag_full(B2, J.t());

		(J*B2Jt).Jacobi(L, R);

		static double Li[MAX_NJOINTS];

		for (int p = 0; p < V.R; ++p)
		{
			Li[p] = L(p, p) / (L(p, p)*L(p, p) + 0.001*0.0001);
		}

		static Matrix LiRt;
		LiRt.resize(V.R, V.R, false);
		LiRt = fast_mul_diag_full(Li, R.t());

		static Matrix Vrot;
		Vrot.resize(V.R, 1, false);
		Vrot = LiRt*V;

		for (int p = 0; p < V.R; ++p)
		{
			if (Vrot(p) < -VLIM) Vrot(p) = -VLIM; else if (Vrot(p) > VLIM) Vrot(p) = VLIM;
		}

		qdot += B2Jt*(R*Vrot);

		P -= B2Jt*(R*LiRt)*J;
	}

	void solveG(Matrix &qdot, const Matrix &J, Matrix &V, double VLIM, Matrix& P)
	{
		static Matrix L, R;
		L.resize(V.R, V.R, false);
		R.resize(V.R, V.R, false);

		static Matrix B2Jt;
		B2Jt.resize(NJOINTS, V.R, false);

		B2Jt = fast_mul_diag_full(B2, J.t());

		(J*B2Jt).Jacobi(L, R);

		static double Li[MAX_NJOINTS];

		for (int p = 0; p < V.R; ++p)
		{
			Li[p] = L(p, p) / (L(p, p)*L(p, p) + 0.001*0.0001);
		}

		static Matrix LiRt;
		LiRt.resize(V.R, V.R, false);
		LiRt = fast_mul_diag_full(Li, R.t());

		static Matrix Vrot;
		Vrot.resize(V.R, 1, false);
		Vrot = LiRt*V;

		for (int p = 0; p < V.R; ++p)
		{
			if (Vrot(p) < -VLIM) Vrot(p) = -VLIM; else if (Vrot(p) > VLIM) Vrot(p) = VLIM;
		}

		qdot += B2Jt*(R*Vrot);

		P -= B2Jt*(R*LiRt)*J;
	}

	void solveO(Matrix &qdot, const Matrix &J, Matrix &V, double VLIM, Matrix& P)
	{
		static Matrix L, R;
		L.resize(V.R, V.R, false);
		R.resize(V.R, V.R, false);

		static Matrix B2Jt;
		B2Jt.resize(NJOINTS, V.R, false);

		B2Jt = fast_mul_diag_full(B2, J.t());

		(J*B2Jt).Jacobi(L, R);

		static double Li[MAX_NJOINTS];

		for (int p = 0; p < V.R; ++p)
		{
			Li[p] = L(p, p) / (L(p, p)*L(p, p) + 0.001*0.0001);
		}

		static Matrix LiRt;
		LiRt.resize(V.R, V.R, false);
		LiRt = fast_mul_diag_full(Li, R.t());

		static Matrix Vrot;
		Vrot.resize(V.R, 1, false);
		Vrot = LiRt*V;

		for (int p = 0; p < V.R; ++p)
		{
			if (Vrot(p) < -VLIM) Vrot(p) = -VLIM; else if (Vrot(p) > VLIM) Vrot(p) = VLIM;
		}

		qdot += B2Jt*(R*Vrot);

		P -= B2Jt*(R*LiRt)*J;
	}

	void ikin_VW(Matrix &qdotout, const Matrix& Jv, const Matrix& Jw, Matrix V, Matrix W, Matrix &P, bool vcontrol, bool wcontrol)
	{
		static double Li[6];

		////////////////////////////////
		// target control
		if (vcontrol && P.det() != 0.0)
		{
			static const double VLIM = 10.0;

			V -= Jv*qdotout;

			static Matrix H;
			H.resize(Jv.R, Jv.C, false);
			H = Jv*P;

			Matrix B2Ht;
			B2Ht.resize(Jv.C, Jv.R, false);
			B2Ht = fast_mul_diag_full(B2, H.t());

			static Matrix L, R;
			L.resize(V.R, V.R, false);
			R.resize(V.R, V.R, false);

			(H*B2Ht).Jacobi(L, R);

			for (int p = 0; p < V.R; ++p)
			{
				if (L(p, p) < 0.000001)
				{
					Li[p] = 0.0;
				}
				else
				{
					Li[p] = 1.0 / L(p, p);
				}
			}

			static Matrix LiRt;
			LiRt.resize(V.R, V.R, false);
			LiRt = fast_mul_diag_full(Li, R.t());

			V = LiRt*V;

			for (int p = 0; p < V.R; ++p)
			{
				//Vrot(p) *= exp(-0.5*Vrot(p)*Vrot(p) / (VLIM*VLIM)) / 0.6;
				if (V(p) < -VLIM) V(p) = -VLIM; else if (V(p) > VLIM) V(p) = VLIM;
			}

			qdotout += B2Ht*(R*V);

			P -= B2Ht*(R*LiRt)*H;
		}

		////////////////////////////////
		// orientation
		if (wcontrol && P.det() != 0.0)
		{
			static const double WLIM = 5.0;

			//W -= Jw*qdotout;

			static Matrix H;
			H.resize(Jw.R, Jw.C, false);
			H = Jw*P;

			static Matrix B2Ht;
			B2Ht.resize(Jw.C, Jw.R, false);
			B2Ht = fast_mul_diag_full(B2, H.t());

			Matrix L, R;
			L.resize(W.R, W.R, false);
			R.resize(W.R, W.R, false);

			(H*B2Ht).Jacobi(L, R);

			for (int p = 0; p < W.R; ++p)
			{
				if (L(p, p) < 0.000001)
				{
					Li[p] = 0.0;
				}
				else
				{
					Li[p] = 1.0 / L(p, p);
				}
			}

			static Matrix LiRt;
			LiRt.resize(W.R, W.R, false);
			LiRt = fast_mul_diag_full(Li, R.t());

			W = LiRt*W;

			for (int p = 0; p < W.R; ++p)
			{
				W(p) *= exp(-0.5*W(p)*W(p) / (WLIM*WLIM)) / 0.6;
				//if (W(p) < -WLIM) W(p) = -WLIM; else if (W(p) > WLIM) W(p) = WLIM;
			}

			qdotout += B2Ht*(R*W);

			P -= B2Ht*(R*LiRt)*H;
		}
	}

	virtual void joint_limits(Matrix& q, Matrix& qdot)
	{
		static Matrix qdot_prec(NJOINTS);

		for (int j = 0; j < NJOINTS; ++j)
		{
			if (qdot_prec(j) < 0.0 && qdot(j) > qdot_prec(j) || qdot_prec(j) > 0.0 && qdot(j) < qdot_prec(j))
			{
				if (qdot(j) - qdot_prec(j) > dec_max[j])
				{
					qdot(j) = qdot_prec(j) + dec_max[j];
				}
				else if (qdot(j) - qdot_prec(j) < -dec_max[j])
				{
					qdot(j) = qdot_prec(j) - dec_max[j];
				}
			}
			else
			{
				if (qdot(j) - qdot_prec(j) > acc_max[j])
				{
					qdot(j) = qdot_prec(j) + acc_max[j];
				}
				else if (qdot(j) - qdot_prec(j) < -acc_max[j])
				{
					qdot(j) = qdot_prec(j) - acc_max[j];
				}
			}

			qdot_prec(j) = qdot(j);

			if (q(j) <= qmin(j))
			{
				if (qdot(j)<0.0) qdot(j) = qmin(j) - q(j);
			}
			else if (q(j) >= qmax(j))
			{
				if (qdot(j)>0.0) qdot(j) = qmax(j) - q(j);
			}

			if (qdot(j)<-qdot_max(j))
			{
				qdot(j) = -qdot_max(j);
			}
			else if (qdot(j)>qdot_max(j))
			{
				qdot(j) = qdot_max(j);
			}
		}
	}
};

}
}

#endif
