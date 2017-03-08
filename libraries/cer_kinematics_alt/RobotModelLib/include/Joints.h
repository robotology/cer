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

#ifndef __CER_JOINTS_H__
#define __CER_JOINTS_H__

#include <stdio.h>
#include "Matrix.h"
#include "Geometry.h"

#define JOINTS(j) for (int j=0; j<mRoot->NJ; ++j)

//#define SET(V,j,W) { V(0,j)=W(0); V(1,j)=W(1); V(2,j)=W(2); }

namespace cer
{
	namespace robot_model
	{

		class Component
		{
		protected:
			Component(Component* parent)
			{
				Nchilds = 0;
				mFrom = NULL;
				mParent = parent;

				if (parent)
				{
					mRoot = parent->mRoot;
					mRoot->NJ++;
				}
				else
				{
					mRoot = this;
					NJ = 1;
				}

				Poj = Zoj = Voj = NULL;

				if (mParent) mParent->addChild(this);
			}

		public:
			virtual ~Component()
			{
				for (int i = 0; i < Nchilds; ++i) delete mChilds[i];

				if (Poj) delete[] Poj;
				if (Zoj) delete[] Zoj;
				if (Voj) delete[] Voj;
			}

			virtual void calcPosture(Matrix& q, const Transform& Tprec, Component *from = NULL, Vec3* Poj_1 = NULL, Vec3* Zoj_1 = NULL, Vec3* Voj_1 = NULL) = 0;

			void addChild(Component *child){ mChilds[Nchilds++] = child; }

			void setGworld(double mass, double x, double y, double z)
			{
				Mj = mass;

				Grel = Tparent.inv()*Vec3(x, y, z);
			}

			void calcGravity(double & M, Vec3& G, Matrix& Jg)
			{
				Vec3 Gj = Tparent*Grel;

				G += Mj*Gj;

				M += Mj;

				for (int j = 0; j < Jg.C; ++j)
				{
					Vec3 ZjxP_Gj = Voj[j] + (Zoj[j] % (Gj - Poj[j]));

					Jg(0, j) += Mj*ZjxP_Gj.x;
					Jg(1, j) += Mj*ZjxP_Gj.y;
				}
			}

			void getJ(Matrix& J)
			{
				for (int j = 0; j < J.C; ++j)
				{
					Vec3 ZjxPPj = Voj[j] + (Zoj[j] % (Toj.Pj() - Poj[j]));

					J(0, j) = ZjxPPj.x; J(1, j) = ZjxPPj.y; J(2, j) = ZjxPPj.z;
					J(3, j) = Zoj[j].x; J(4, j) = Zoj[j].y; J(5, j) = Zoj[j].z;
				}
			}

			Transform Toj;
			Transform Tparent;

			int NJ;

			Vec3 *Poj;// [NJ];
			Vec3 *Zoj;// [NJ];
			Vec3 *Voj;// [NJ];

			Vec3 Grel;

			double Mj;

		protected:
			int Nchilds;

			Component *mRoot;
			Component *mFrom;
			Component *mParent;
			Component *mChilds[8];
		};

		class Link : public Component
		{
		public:
			Link(const Transform &T0, Component* parent) : Tdir(T0), Tinv(T0.inv()), Component(parent){}

			virtual void calcPosture(Matrix& q, const Transform& Tprec, Component *from = NULL, Vec3* Poj_1 = NULL, Vec3* Zoj_1 = NULL, Vec3* Voj_1 = NULL)
			{
				Tparent = Tprec;

				if (!Poj) Poj = new Vec3[mRoot->NJ];
				if (!Zoj) Zoj = new Vec3[mRoot->NJ];
				if (!Voj) Voj = new Vec3[mRoot->NJ];

				if (Zoj_1 && Poj_1 && Voj_1)
				{
					JOINTS(j) { Poj[j] = Poj_1[j]; Zoj[j] = Zoj_1[j]; Voj[j] = Voj_1[j]; }
				}
				else
				{
					JOINTS(j) { Poj[j].clear(); Zoj[j].clear(); Voj[j].clear(); }
				}

				mFrom = from;

				if (mFrom == mParent)
				{
					Toj = Tprec*Tdir;

					for (int i = 0; i < Nchilds; ++i) mChilds[i]->calcPosture(q, Toj, this, Poj, Zoj, Voj);
				}
				else
				{
					Toj = Tprec;

					Transform T = Tprec*Tinv;

					if (mParent) mParent->calcPosture(q, T, this, Poj, Zoj, Voj);

					for (int i = 0; i < Nchilds; ++i)
					{
						if (mFrom != mChilds[i]) mChilds[i]->calcPosture(q, T, this, Poj, Zoj, Voj);
					}
				}
			}

		protected:
			const Transform Tdir, Tinv;
		};

		class Joint : public Component
		{
		protected:
			Joint(int id, double q0, double q1, Component* parent) : Component(parent)
			{
				j0 = id;

				setLimits(q0, q1);
			}

		public:
			virtual void setLimits(double q0, double q1)
			{
				qmin = q0;
				qmax = q1;
			}

			virtual void getLimits(double& q0, double& q1)
			{
				q0 = qmin;
				q1 = qmax;
			}

		protected:
			int j0;
			double qmin;
			double qmax;
		};

		class RotJoint : public Joint
		{
		public:
			RotJoint(int id, double q0, double q1, Component* parent) : Joint(id, q0, q1, parent)
			{
			}

			virtual void calcPosture(Matrix& q, const Transform& Tprec, Component *from, Vec3* Poj_1 = NULL, Vec3* Zoj_1 = NULL, Vec3* Voj_1 = NULL)
			{
				Tparent = Tprec;

				if (!Poj) Poj = new Vec3[mRoot->NJ];
				if (!Zoj) Zoj = new Vec3[mRoot->NJ];
				if (!Voj) Voj = new Vec3[mRoot->NJ];

				if (Zoj_1 && Poj_1 && Voj_1)
				{
					JOINTS(j) { Poj[j] = Poj_1[j]; Zoj[j] = Zoj_1[j]; Voj[j] = Voj_1[j]; }
				}
				else
				{
					JOINTS(j) { Poj[j].clear(); Zoj[j].clear(); Voj[j].clear(); }
				}

				mFrom = from;

				if (mFrom == mParent)
				{
					Toj = Tprec*Transform(q(j0));

					Poj[j0] = Toj.Pj();
					Zoj[j0] = Toj.Zj();
					Voj[j0].clear();

					for (int i = 0; i < Nchilds; ++i) mChilds[i]->calcPosture(q, Toj, this, Poj, Zoj, Voj);
				}
				else
				{
					Toj = Tprec*Transform(-q(j0));

					Poj[j0] = Toj.Pj();
					Zoj[j0] = -Toj.Zj();
					Voj[j0].clear();

					if (mParent) mParent->calcPosture(q, Toj, this, Poj, Zoj, Voj);

					for (int i = 0; i < Nchilds; ++i)
					{
						if (mFrom != mChilds[i]) mChilds[i]->calcPosture(q, Toj, this, Poj, Zoj, Voj);
					}
				}
			}

		protected:
		};

		class Trifid : public Joint
		{
		public:
			Trifid(double size, int id, double q0, double q1, Component* parent) : Joint(id, q0, q1, parent)
			{
				L = size;

				j1 = id + 1;
				j2 = id + 2;

				invNz = 1.0 / (1.5*sqrt(3.0)*L*L);

				B0 = Vec3(L, 0.0, 0.0);
				B1 = Vec3(-0.5*L, 0.5*sqrt(3.0)*L, 0.0);
				B2 = Vec3(-0.5*L, -0.5*sqrt(3.0)*L, 0.0);
			}

			~Trifid(){}

			virtual void calcPosture(Matrix& q, const Transform& Tprec, Component *from, Vec3* Poj_1 = NULL, Vec3* Zoj_1 = NULL, Vec3* Voj_1 = NULL);

			void setExtension(Matrix&q, double qe)
			{
				double delta = qe - (q(j0) + q(j1) + q(j2)) / 3.0;

				q(j0) += delta; q(j1) += delta; q(j2) += delta;

				if (q(j0) < qmin){ double inc = qmin - q(j0); q(j0) = qmin; q(j1) += inc; q(j2) += inc; }
				if (q(j1) < qmin){ double inc = qmin - q(j1); q(j1) = qmin; q(j2) += inc; q(j0) += inc; }
				if (q(j2) < qmin){ double inc = qmin - q(j2); q(j2) = qmin; q(j0) += inc; q(j1) += inc; }

				if (q(j0) > qmax){ double inc = qmax - q(j0); q(j0) = qmin; q(j1) += inc; q(j2) += inc; }
				if (q(j1) > qmax){ double inc = qmax - q(j1); q(j1) = qmin; q(j2) += inc; q(j0) += inc; }
				if (q(j2) > qmax){ double inc = qmax - q(j2); q(j2) = qmin; q(j0) += inc; q(j1) += inc; }
			}

			double angle(){ return AA.mod()*RAD2DEG; }

			/*
			bool checkAngle(double angle,Matrix &J,Matrix &q,Matrix &qp)
			{
			if (AA.mod()*RAD2DEG>=angle)
			{
			Vec3 W0=Z[0]*qp(j0);
			Vec3 W1=Z[1]*qp(j1);
			Vec3 W2=Z[2]*qp(j2);

			if ((W0+W1+W2)*AA>0.0)
			{
			bool ret=false;

			if (W0*AA>0.0)
			{
			qp(j0)=0.0;
			if (J.clearCol(j0)) ret=true;
			}

			if (W1*AA>0.0)
			{
			qp(j1)=0.0;
			if (J.clearCol(j1)) ret=true;
			}

			if (W2*AA>0.0)
			{
			qp(j2)=0.0;
			if (J.clearCol(j2)) ret=true;
			}

			return ret;
			}
			}

			return false;
			}

			bool checkAngle(double angle,Matrix &q,Matrix &qp)
			{
			if (AA.mod()*RAD2DEG>=angle)
			{
			Vec3 W0=Z[0]*qp(j0);
			Vec3 W1=Z[1]*qp(j1);
			Vec3 W2=Z[2]*qp(j2);

			if ((W0+W1+W2)*AA>0.0)
			{
			if (W0*AA>0.0) qp(j0)=0.0;

			if (W1*AA>0.0) qp(j1)=0.0;

			if (W2*AA>0.0) qp(j2)=0.0;
			}

			return true;
			}

			return false;
			}
			*/

			/*
			bool limitAngle(double angle,Matrix &qp)
			{
			bool ret=false;

			if (AA.mod()*RAD2DEG>=angle)
			{
			Vec3 W0=Z[0]*qp(j0);
			Vec3 W1=Z[1]*qp(j1);
			Vec3 W2=Z[2]*qp(j2);

			if ((W0+W1+W2)*AA>0.0)
			{
			if (W0*AA>0.0) qp(j0)=0.0;

			if (W1*AA>0.0) qp(j1)=0.0;

			if (W2*AA>0.0) qp(j2)=0.0;

			return true;
			}
			}

			return false;
			}
			*/

			void extend(Vec3 Xp, Matrix& q, Matrix& qp, double gain, int *bound)
			{
				double v = gain*((V[0] + V[1] + V[2])*Xp);

				if (v < 0.0)
				{
					if (bound[j0] != -1 && bound[j1] != -1 && bound[j2] != -1)
					{
						if (q(j0) > qmin && q(j1) > qmin && q(j2) > qmin)
						{
							qp(j0) += v; qp(j1) += v; qp(j2) += v;
						}
					}
				}
				else
				{
					if (bound[j0] != 1 && bound[j1] != 1 && bound[j2] != 1)
					{
						if (q(j0) < qmax && q(j1) < qmax && q(j2) < qmax)
						{
							qp(j0) += v; qp(j1) += v; qp(j2) += v;
						}
					}
				}
			}

		protected:
			int j1, j2;

			double L;

			Vec3 B0, B1, B2;

			double invNz;

		public:
			Vec3 AA;
			Vec3 Z[3];
			Vec3 V[3];
		};

	}
}

#endif
