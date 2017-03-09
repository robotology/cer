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

#ifndef __ROBOT_COVERS_H__
#define __ROBOT_COVERS_H__

#include <string>
#include <vector>
#include "Geometry.h"

namespace cer
{
	namespace robot_model
	{
		class Sphere
		{
		protected:
			Vec3 Clocal;

		public:
			Vec3 Cworld;
			double radius;
			std::string name;

			Sphere(){}

			Sphere(double x, double y, double z, double r, const char *s) : Clocal(x, y, z), radius(r), name(s)
			{
			}

			~Sphere(){}

			void pose(Transform &T)
			{
				Cworld = T*Clocal;
			}
		};

		inline double distance(Sphere &Sa, Sphere &Sb, Vec3 &A, Vec3 &B, Vec3 &U)
		{
			A = Sa.Cworld;
			B = Sb.Cworld;

			U = A - B;

			double r = U.normalize() - (Sa.radius + Sb.radius);

			A -= Sa.radius*U;
			B += Sb.radius*U;

			return r;
		}

		class Cover
		{
		public:
			enum { MAXSPHERES = 32 };
			enum { FLOATING = -1 };

			Sphere sphere[MAXSPHERES];
			int nspheres;

			Cover(int id = FLOATING) : partID(id){ nspheres = 0; }
			~Cover(){}

			Sphere* addSphere(double x, double y, double z, double r, const char *name)
			{
				sphere[nspheres] = Sphere(x, y, z, r, name);

				return &sphere[nspheres++];
			}

			void pose(Transform& T)
			{
				for (int s = 0; s < nspheres; ++s) sphere[s].pose(T);
			}

			void getSphere(int s, double &x, double& y, double& z, double &r, std::string &name)
			{
				x = sphere[s].Cworld.x;
				y = sphere[s].Cworld.y;
				z = sphere[s].Cworld.z;
				r = sphere[s].radius;
				name = sphere[s].name;
			}

			int partID;
		};

		inline double repulsion(Cover *Ca, Cover *Cb, Vec3 &Xa, Vec3 &Xb, Vec3& Ud)
		{
			Vec3 A;
			Vec3 B;
			Vec3 U;

			Xa.clear();
			Xb.clear();
			Ud.clear();

			double r = 0.0;

			double D = 1E10;

			for (int a = 0; a < Ca->nspheres; ++a)
			{
				for (int b = 0; b < Cb->nspheres; ++b)
				{
					double d = distance(Ca->sphere[a], Cb->sphere[b], A, B, U);

					if (d < D) D = d;

					static const double S = -1.0 / (0.05*0.05);

					double s = exp(d*fabs(d)*S);

					Xa += s*A;
					Xb += s*B;
					Ud += s*U;

					r += s;
				}
			}

			Xa /= r;
			Xb /= r;
			Ud.normalize();

			return D;
		}

		class Interference
		{
		public:
			enum { MAXDEPS = 32 };

			Interference(Cover *pA, Cover *pB, int j0, int j1)
			{
				init(pA, pB);

				for (int j = j0; j <= j0; ++j) addJointDep(j);
			}

			Interference(Cover *pA, Cover *pB)
			{
				init(pA, pB);
			}

			~Interference(){}

			void addJointDep(int j)
			{
				jdep.push_back(j);
			}

			int getDep(int d)
			{
				return jdep[d];
			}

			Cover *coverA;
			Cover *coverB;

			std::vector<int> jdep;

		protected:
			void init(Cover *pA, Cover *pB)
			{
				coverA = pA;
				coverB = pB;
			}
		};

	}
}

#endif
