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

#ifndef __R1_MODEL_H___
#define __R1_MODEL_H___

#include <RobotModel.h>

namespace cer
{
	namespace robot_model
	{
		namespace r1
		{

			class R1Model : public RobotModel
			{
			public:
				R1Model();

				int getNDOF(){ return NJOINTS; }

				virtual const Matrix& calcGravity(Vec3 &com)
				{
					bool dirty = gravDirty;

					RobotModel::calcGravity(com);

					if (dirty)
					{
						Jgrav(0, 8) = 0.0; Jgrav(0, 9) = 0.0; Jgrav(0, 10) = 0.0; Jgrav(0, 11) = 0.0;
						Jgrav(1, 8) = 0.0; Jgrav(1, 9) = 0.0; Jgrav(1, 10) = 0.0; Jgrav(1, 11) = 0.0;

						Jgrav(0, 16) = 0.0; Jgrav(0, 17) = 0.0; Jgrav(0, 18) = 0.0; Jgrav(0, 19) = 0.0;
						Jgrav(1, 16) = 0.0; Jgrav(1, 17) = 0.0; Jgrav(1, 18) = 0.0; Jgrav(1, 19) = 0.0;

						Jgrav(0, 20) = 0.0; Jgrav(0, 21) = 0.0;
						Jgrav(1, 20) = 0.0; Jgrav(1, 21) = 0.0;
					}

					return Jgrav;
				}

				double getBalancing(Vec3 &Force)
				{
					static Vec3 C1(0.055 + 0.074, 0.110, 0.0);
					static Vec3 W1(-0.074 + 0.074, 0.170, 0.0);
					static Vec3 C2(-0.244 + 0.074, 0.000, 0.0);
					static Vec3 W2(-0.074 + 0.074, -0.170, 0.0);
					static Vec3 C3(0.055 + 0.074, -0.110, 0.0);

					static Vec3 G0 = (W1 + W2) / 2.0;

					static Vec3 L0 = (W1 - C1).norm();
					static Vec3 L1 = (C2 - W1).norm();
					static Vec3 L2 = (W2 - C2).norm();
					static Vec3 L3 = (C3 - W2).norm();
					static Vec3 L4 = (C1 - C3).norm();

					Vec3 Gp = G;

					Gp.z = 0.0;

					Force = G0 - Gp;

					Vec3 D0 = Gp - C1, D1 = Gp - W1, D2 = Gp - C2, D3 = Gp - W2, D4 = Gp - C3;

					double d0 = (D0%L0).z, d1 = (D1%L1).z, d2 = (D2%L2).z, d3 = (D3%L3).z, d4 = (D4%L4).z;

					if (d0 >= 0.0 || d1 >= 0.0 || d2 >= 0.0 || d3 >= 0.0 || d4 >= 0.0)
					{
						return 0.0;
					}

					D0 = D0 - (D0*L0)*L0;
					D1 = D1 - (D1*L1)*L1;
					D2 = D2 - (D2*L2)*L2;
					D3 = D3 - (D3*L3)*L3;
					D4 = D4 - (D4*L4)*L4;

					d0 = D0.mod(); d1 = D1.mod(); d2 = D2.mod(); d3 = D3.mod(); d4 = D4.mod();

					double margin = 100.0;

					if (d0 < margin) margin = d0;
					if (d1 < margin) margin = d1;
					if (d2 < margin) margin = d2;
					if (d3 < margin) margin = d3;
					if (d4 < margin) margin = d4;

					return margin;
				}

			protected:
				enum
				{
					TORSO_TRIFID_0,
					TORSO_TRIFID_1,
					TORSO_TRIFID_2,
					TORSO_YAW,

					LEFT_SHOULDER_0,
					LEFT_SHOULDER_1,
					LEFT_SHOULDER_2,
					LEFT_ELBOW,

					LEFT_WRIST_ROT,
					LEFT_TRIFID_0,
					LEFT_TRIFID_1,
					LEFT_TRIFID_2,

					RIGHT_SHOULDER_0,
					RIGHT_SHOULDER_1,
					RIGHT_SHOULDER_2,
					RIGHT_ELBOW,

					RIGHT_WRIST_ROT,
					RIGHT_TRIFID_0,
					RIGHT_TRIFID_1,
					RIGHT_TRIFID_2,

					HEAD_PITCH,
					HEAD_YAW,

					NJOINTS
				};

				enum{ BASE, TORSO, LEFT_UPPER_ARM, LEFT_LOWER_ARM, LEFT_HAND, RIGHT_UPPER_ARM, RIGHT_LOWER_ARM, RIGHT_HAND, HEAD, NPARTS };
			};
		}
	}
}

#endif