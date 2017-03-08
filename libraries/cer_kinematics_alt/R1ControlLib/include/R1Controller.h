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

#ifndef __R1_CONTROL_H__
#define __R1_CONTROL_H__

//#include <Geometry.h>
//#include <Matrix.h>
//#include <RobotModel.h>
#include <R1Model.h>

#include <RobotController.h>

#define PERIOD 0.01

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


using namespace cer::robot_model::r1;

namespace cer {
namespace kinematics_alt {
namespace r1{

class R1Controller : public RobotController
{
public:
	R1Controller(R1Model *r1m) : RobotController(r1m)
	{
		NJOINTS = robotModel->getNDOF();

		qzero.resize(NJOINTS);

		qmin.resize(NJOINTS);
		qmax.resize(NJOINTS);
		qdot_max.resize(NJOINTS);

		robotModel->getJointLimits(qmin, qmax);

		for (int j = 0; j<NJOINTS; ++j) // revolute
		{
			qdot_max(j) = 30.0;

			Kc[j] = RAD2DEG;
		}

		qdot_max(0) = qdot_max(1) = qdot_max(2) = 0.015;
		qdot_max(9) = qdot_max(10) = qdot_max(11) = 0.0075;
		qdot_max(17) = qdot_max(18) = qdot_max(19) = 0.0075;

		Kc[0] = Kc[1] = Kc[2] = 1.0;
		Kc[9] = Kc[10] = Kc[11] = 1.0;
		Kc[17] = Kc[18] = Kc[19] = 1.0;

		for (int j = 0; j < NJOINTS; ++j)
		{
			Kz[j] = 1.25 / Kc[j];

			W2[j] = qdot_max(j) / Kc[j];
			W2[j] = W2[j] * W2[j];

			acc_max[j] = 2.0*PERIOD*qdot_max(j);
			dec_max[j] = 10.0*acc_max[j];
		}

		qzero = (qmin+qmax)*0.5;

		qzero(20) = qzero(21) = 0.0;

		setExtensions(DEFAULT_TORSO_EXTENSION, DEFAULT_ARM_EXTENSION, DEFAULT_ARM_EXTENSION);
		//setExtensions(-0.05, 0.14, DEFAULT_ARM_EXTENSION);
	}
	
	void setExtensions(double torsoElong, double armElongL, double armElongR)
	{
		const double TORSO_EXC = 0.75*TORSO_RADIUS*tan(DEG2RAD*TORSO_MAX_TILT);
		const double ARM_EXC = 0.75*ARM_RADIUS*tan(DEG2RAD*WRIST_MAX_TILT);
		const double WRIST_TILT_ZERO_EXC = 0.75*ARM_RADIUS*tan(DEG2RAD*WRIST_TILT_ZERO);

		{
			double qa = torsoElong - TORSO_EXC;
			double qb = torsoElong + TORSO_EXC;

			if (qa<MIN_TORSO_EXTENSION){ qa = MIN_TORSO_EXTENSION; qb += MIN_TORSO_EXTENSION - qa; }
			if (qb>MAX_TORSO_EXTENSION){ qa += MAX_TORSO_EXTENSION - qb; qb = MAX_TORSO_EXTENSION; }

			qzero(0) = qzero(1) = qzero(2) = 0.5*(qa + qb);
			qmin(0) = qmin(1) = qmin(2) = qa;
			qmax(0) = qmax(1) = qmax(2) = qb;
		}

		{
			double qa = armElongL - ARM_EXC;
			double qb = armElongL + ARM_EXC;

			if (qa<MIN_ARM_EXTENSION){ qa = MIN_ARM_EXTENSION; qb += MIN_ARM_EXTENSION - qa; }
			if (qb>MAX_ARM_EXTENSION){ qa += MAX_ARM_EXTENSION - qb; qb = MAX_ARM_EXTENSION; }

			double armExtL = 0.5*(qa + qb); 

			qzero(9) = armExtL + WRIST_TILT_ZERO_EXC;
			qzero(10) = qzero(11) = armExtL - WRIST_TILT_ZERO_EXC;
			qmin(9) = qmin(10) = qmin(11) = qa;
			qmax(9) = qmax(10) = qmax(11) = qb;
		}

		{
			double qa = armElongR - ARM_EXC;
			double qb = armElongR + ARM_EXC;

			if (qa<MIN_ARM_EXTENSION){ qa = MIN_ARM_EXTENSION; qb += MIN_ARM_EXTENSION - qa; }
			if (qb>MAX_ARM_EXTENSION){ qa += MAX_ARM_EXTENSION - qb; qb = MAX_ARM_EXTENSION; }

			double armExtR = 0.5*(qa + qb);

			qzero(17) = armExtR + WRIST_TILT_ZERO_EXC;
			qzero(18) = qzero(19) = armExtR - WRIST_TILT_ZERO_EXC;
			qmin(17) = qmin(18) = qmin(19) = qa;
			qmax(17) = qmax(18) = qmax(19) = qb;
		}
	}
};

}
}
}

#endif
