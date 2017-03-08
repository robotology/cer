/*
* Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
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

#ifndef __ROBOT_MODEL_H___
#define __ROBOT_MODEL_H___

#include "Geometry.h"
#include "Matrix.h"
#include "Joints.h"
#include "Covers.h"

namespace cer
{
	namespace robot_model
	{

		class RobotModel
		{
		protected:

			RobotModel() : T_ROOT()
			{
				mRoot = NULL;

				gravDirty = true;
				selfDirty = true;
			}

		public:

			virtual ~RobotModel()
			{
				if (mRoot) delete mRoot; // recursive delete

				for (unsigned int i = 0; i < interference.size(); ++i) delete interference[i];

				for (unsigned int c = 0; c < (int)cover_list.size(); ++c) delete cover_list[c];
			}

			virtual int getNDOF() = 0;

			void getJointLimits(Matrix &q0, Matrix &q1)
			{
				q0 = qmin;
				q1 = qmax;
			}

			virtual void calcConfig(Matrix &q)
			{
				mRoot->calcPosture(q, T_ROOT);

				gravDirty = selfDirty = true;
			}

			virtual const Matrix& calcGravity(Vec3& com)
			{
				if (gravDirty)
				{
					gravDirty = false;

					double M = 0.0;

					G.clear();
					Jgrav.clear();

					for (unsigned int g = 0; g < heavy_part.size(); ++g)
					{
						heavy_part[g]->calcGravity(M, G, Jgrav);
					}

					Jgrav /= M;

					G /= M;
				}

				com = G;

				return Jgrav;
			}

			virtual const Vec3& getCOM() { return G; }

			virtual const Matrix& calcInterference(Matrix &distance);

			//virtual const Matrix& calcInterference(Vec3* Xa, Vec3* Xb, Vec3* Ud, Matrix &distance);

			virtual const Matrix& calcHandJacobian(int hand)
			{
				mHand[hand]->getJ(Jhand[hand]);

				return Jhand[hand];
			}

			virtual const Transform& getHandTransformL()
			{
				return mHand[L]->Toj;
			}

			virtual const Transform& getHandTransformR()
			{
				return mHand[R]->Toj;
			}

			virtual double getBalancing(Vec3 &Force) = 0;

			void getSphere(int s, double &x, double& y, double& z, double &r, std::string &name)
			{
				x = sphere_list[s]->Cworld.x;
				y = sphere_list[s]->Cworld.y;
				z = sphere_list[s]->Cworld.z;
				r = sphere_list[s]->radius;
				name = sphere_list[s]->name;
			}

			int getNSpheres(){ return sphere_list.size(); }

			enum { R = 0, L = 1 };

		protected:
			void calcInterference();

			Component *mRoot;
			Component *mHand[2];

			Matrix qmin;
			Matrix qmax;

			bool gravDirty;
			bool selfDirty;

			std::vector<Interference*> interference;
			std::vector<Cover*> cover_list;
			std::vector<Sphere*> sphere_list;

			std::vector<Component*> solid_part;
			std::vector<Component*> heavy_part;

			Transform T_ROOT;

			Vec3 G;
			Matrix selfDistance;
			Matrix Jgrav;
			Matrix Jself;

			Matrix Jhand[2];
		};

	}
}

#endif