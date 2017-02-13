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

#include <R1Model.h>

#define JINIT(name) name, qmin(name), qmax(name), newjoint

#define JLIMIT(name, q0, q1) qmin(name) = q0; qmax(name) = q1

#define ROOT NULL

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

//#define STOP_DISTANCE 0.05

using namespace cer::robot_model;

R1Model::R1Model() : RobotModel()
{
	double TORSO_EXC = 0.75*TORSO_RADIUS*tan(DEG2RAD*TORSO_MAX_TILT);
	double ARM_EXC = 0.75*ARM_RADIUS*tan(DEG2RAD*WRIST_MAX_TILT);

	qmin.resize(NJOINTS);
	qmax.resize(NJOINTS);

	JLIMIT(TORSO_TRIFID_0, DEFAULT_TORSO_EXTENSION - TORSO_EXC, DEFAULT_TORSO_EXTENSION + TORSO_EXC);
	JLIMIT(TORSO_TRIFID_1, DEFAULT_TORSO_EXTENSION - TORSO_EXC, DEFAULT_TORSO_EXTENSION + TORSO_EXC);
	JLIMIT(TORSO_TRIFID_2, DEFAULT_TORSO_EXTENSION - TORSO_EXC, DEFAULT_TORSO_EXTENSION + TORSO_EXC);

	JLIMIT(TORSO_YAW, -60, +60);

	JLIMIT(LEFT_SHOULDER_0, -25, +55);
	JLIMIT(LEFT_SHOULDER_1, +11, +65);
	JLIMIT(LEFT_SHOULDER_2, -85, +85);

	JLIMIT(LEFT_ELBOW, +2, +90);

	JLIMIT(LEFT_WRIST_ROT, -90, +90);

	JLIMIT(LEFT_TRIFID_0, DEFAULT_ARM_EXTENSION - ARM_EXC, DEFAULT_ARM_EXTENSION + ARM_EXC);
	JLIMIT(LEFT_TRIFID_1, DEFAULT_ARM_EXTENSION - ARM_EXC, DEFAULT_ARM_EXTENSION + ARM_EXC);
	JLIMIT(LEFT_TRIFID_2, DEFAULT_ARM_EXTENSION - ARM_EXC, DEFAULT_ARM_EXTENSION + ARM_EXC);

	JLIMIT(RIGHT_SHOULDER_0, -25, +55);
	JLIMIT(RIGHT_SHOULDER_1, +11, +65);
	JLIMIT(RIGHT_SHOULDER_2, -85, +85);

	JLIMIT(RIGHT_ELBOW, +2, +90);

	JLIMIT(RIGHT_WRIST_ROT, -90, +90);

	JLIMIT(RIGHT_TRIFID_0, DEFAULT_ARM_EXTENSION - ARM_EXC, DEFAULT_ARM_EXTENSION + ARM_EXC);
	JLIMIT(RIGHT_TRIFID_1, DEFAULT_ARM_EXTENSION - ARM_EXC, DEFAULT_ARM_EXTENSION + ARM_EXC);
	JLIMIT(RIGHT_TRIFID_2, DEFAULT_ARM_EXTENSION - ARM_EXC, DEFAULT_ARM_EXTENSION + ARM_EXC);

	JLIMIT(HEAD_PITCH, -90, +30);

	JLIMIT(HEAD_YAW, -80, +80);



	Component *newjoint;

	mRoot = newjoint = new Link(T_ROOT, ROOT);

	newjoint = new Link(Transform(0, 0, 180, 0.044, 0.0, 0.470), newjoint);

	newjoint = new Trifid(TORSO_RADIUS, JINIT(TORSO_TRIFID_0));

	Component *torsoYaw = newjoint = new RotJoint(JINIT(TORSO_YAW));

	// left arm
	//{
	newjoint = new Link(Transform(-0.084, 0.325869, 104.000002, 180), torsoYaw);

	newjoint = new RotJoint(JINIT(LEFT_SHOULDER_0));

	newjoint = new Link(Transform(0.0, -0.182419, 90, 90), newjoint);

	newjoint = new RotJoint(JINIT(LEFT_SHOULDER_1));

	newjoint = new Link(Transform(0.034, 0.0, -90, -104.000002), newjoint);

	newjoint = new RotJoint(JINIT(LEFT_SHOULDER_2));

	Component *upperArmL = newjoint = new Link(Transform(0.0, -0.251, 90, -90), newjoint);

	newjoint = new RotJoint(JINIT(LEFT_ELBOW));

	newjoint = new Link(Transform(0.0, 0.0, -90, 0), newjoint);

	newjoint = new RotJoint(JINIT(LEFT_WRIST_ROT));

	Component *prosupL = newjoint = new Link(Transform(0.0, -0.291, 180, -90), newjoint);

	Component *lowerArmL = newjoint = new Trifid(ARM_RADIUS, JINIT(LEFT_TRIFID_0));

	mHand[L] = newjoint = new Link(Transform(0, -90, 0, 0.0269, 0, 0.1004), newjoint);
	//}

	// right arm
	//{
	newjoint = new Link(Transform(-0.084, 0.325869, 75.999998, 180), torsoYaw);

	newjoint = new RotJoint(JINIT(RIGHT_SHOULDER_0));

	newjoint = new Link(Transform(0.0, 0.182419, 90, -90), newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_SHOULDER_1));

	newjoint = new Link(Transform(-0.034, 0.0, -90, -104.000002), newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_SHOULDER_2));

	Component *upperArmR = newjoint = new Link(Transform(0.0, 0.251, -90, 90), newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_ELBOW));

	newjoint = new Link(Transform(0.0, 0.0, 90, 0), newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_WRIST_ROT));

	Component *prosupR = newjoint = new Link(Transform(0.0, 0.291, 0, -90), newjoint);

	Component *lowerArmR = newjoint = new Trifid(ARM_RADIUS, JINIT(RIGHT_TRIFID_0));

	mHand[R] = newjoint = new Link(Transform(0, -90, 180, 0.0269, 0, 0.1004), newjoint);
	//}

	// head
	//{
	Component *torso = newjoint = new Link(Transform(-0.084, 0.339, 90, 180), torsoYaw);

	newjoint = new RotJoint(JINIT(HEAD_PITCH));

	newjoint = new Link(Transform(0, 0, -90, 0), newjoint);

	newjoint = new RotJoint(JINIT(HEAD_YAW));

	Component *head = newjoint = new Link(Transform(0, 200, 0, 0), newjoint);
	//}

	///////////////////////////////////////////////////////////////////////////////////

	Matrix q0(NJOINTS);

	// masses

	q0(LEFT_TRIFID_0) = q0(RIGHT_TRIFID_0) = 0.01 + 0.005;

	q0(LEFT_TRIFID_1) = q0(LEFT_TRIFID_2) = q0(RIGHT_TRIFID_1) = q0(RIGHT_TRIFID_2) = 0.01 - 0.005;

	mRoot->calcPosture(q0, T_ROOT);

	mRoot->setGworld(31.0, 0.019, 0.0, 0.081);
	torso->setGworld(12.8, 0.007, 0.0, 0.715);
	upperArmR->setGworld(1.43, -0.041, -0.212, -0.089);
	upperArmL->setGworld(1.43, -0.041, 0.212, -0.089);
	lowerArmR->setGworld(1.13, -0.040, -0.210, 0.438);
	lowerArmL->setGworld(1.13, -0.040, 0.210, 0.438);
	mHand[R]->setGworld(0.667, -0.037, -0.195, 0.222);
	mHand[L]->setGworld(0.667, -0.037, 0.195, 0.222);
	head->setGworld(2.88, 0.012, 0.0, 0.997);

	heavy_part.push_back(mRoot);
	heavy_part.push_back(torso);
	heavy_part.push_back(upperArmR);
	heavy_part.push_back(upperArmL);
	heavy_part.push_back(lowerArmR);
	heavy_part.push_back(lowerArmL);
	heavy_part.push_back(mHand[R]);
	heavy_part.push_back(mHand[L]);
	heavy_part.push_back(head);

	// covers

	solid_part.push_back(NULL); // BASE
	solid_part.push_back(torsoYaw); // TORSO
	solid_part.push_back(upperArmL); // LEFT_UPPER_ARM
	solid_part.push_back(prosupL); // LEFT_LOWER_ARM
	solid_part.push_back(mHand[L]); // LEFT_HAND
	solid_part.push_back(upperArmR); // RIGHT_UPPER_ARM
	solid_part.push_back(prosupR); // RIGHT_LOWER_ARM
	solid_part.push_back(mHand[R]); // RIGHT_HAND
	solid_part.push_back(NULL); // NULL

	Cover *cover[NPARTS];

	cover[BASE] = NULL;
	cover[TORSO] = new Cover(TORSO);
	cover[LEFT_UPPER_ARM] = new Cover(LEFT_UPPER_ARM);
	cover[LEFT_LOWER_ARM] = new Cover(LEFT_LOWER_ARM);
	cover[LEFT_HAND] = new Cover(LEFT_HAND);
	cover[RIGHT_UPPER_ARM] = new Cover(RIGHT_UPPER_ARM);
	cover[RIGHT_LOWER_ARM] = new Cover(RIGHT_LOWER_ARM);
	cover[RIGHT_HAND] = new Cover(RIGHT_HAND);
	cover[HEAD] = NULL;

	for (int p = 0; p < NPARTS; ++p) if (cover[p]) cover_list.push_back(cover[p]);

#define STORE_SPHERE sphere_list.push_back(NULL); sphere_list.back() = 

	STORE_SPHERE cover[TORSO]->addSphere(0.05, 0.0, 0.03, 0.08, "3_0");
	STORE_SPHERE cover[TORSO]->addSphere(-0.04, -0.06, 0.03, 0.08, "3_1");
	STORE_SPHERE cover[TORSO]->addSphere(-0.04, 0.06, 0.03, 0.08, "3_2");
	STORE_SPHERE cover[TORSO]->addSphere(0.08, 0.0, 0.12, 0.09, "3_3");
	STORE_SPHERE cover[TORSO]->addSphere(0.01, -0.06, 0.12, 0.09, "3_4");
	STORE_SPHERE cover[TORSO]->addSphere(0.01, 0.06, 0.12, 0.09, "3_5");
	STORE_SPHERE cover[TORSO]->addSphere(0.1, 0.0, 0.21, 0.09, "3_6");
	STORE_SPHERE cover[TORSO]->addSphere(0.03, -0.07, 0.21, 0.09, "3_7");
	STORE_SPHERE cover[TORSO]->addSphere(0.03, 0.07, 0.21, 0.09, "3_8");
	STORE_SPHERE cover[TORSO]->addSphere(0.12, 0.0, 0.3, 0.08, "3_9");
	STORE_SPHERE cover[TORSO]->addSphere(0.06, -0.1, 0.32, 0.07, "3_10");
	STORE_SPHERE cover[TORSO]->addSphere(0.06, 0.1, 0.32, 0.07, "3_11");
	STORE_SPHERE cover[TORSO]->addSphere(0.06, 0.0, 0.32, 0.07, "3_12");
	STORE_SPHERE cover[TORSO]->addSphere(0.08, -0.19, 0.37, 0.06, "3_13");
	STORE_SPHERE cover[TORSO]->addSphere(0.08, 0.19, 0.37, 0.06, "3_14");

	STORE_SPHERE cover[LEFT_UPPER_ARM]->addSphere(0.0, 0.0, 0.0, 0.035, "6_0");
	STORE_SPHERE cover[LEFT_UPPER_ARM]->addSphere(0.0, 0.06, 0.0, 0.045, "6_1");
	STORE_SPHERE cover[LEFT_UPPER_ARM]->addSphere(0.0, 0.12, 0.0, 0.045, "6_2");
	STORE_SPHERE cover[LEFT_UPPER_ARM]->addSphere(0.0, 0.18, 0.0, 0.045, "6_3");

	STORE_SPHERE cover[LEFT_LOWER_ARM]->addSphere(0.0, 0.0, 0.0, 0.0375, "8_0");
	STORE_SPHERE cover[LEFT_LOWER_ARM]->addSphere(0.0, 0.0, -0.04, 0.04, "8_1");
	STORE_SPHERE cover[LEFT_LOWER_ARM]->addSphere(0.0, 0.0, -0.10, 0.0425, "8_2");
	STORE_SPHERE cover[LEFT_LOWER_ARM]->addSphere(0.0, 0.0, -0.16, 0.045, "8_3");
	STORE_SPHERE cover[LEFT_LOWER_ARM]->addSphere(0.0, 0.0, -0.22, 0.0425, "8_4");

	STORE_SPHERE cover[LEFT_HAND]->addSphere(0.0, 0.0, 0.02, 0.02, "11_0");
	STORE_SPHERE cover[LEFT_HAND]->addSphere(-0.04, 0.0, 0.01, 0.03, "11_1");
	STORE_SPHERE cover[LEFT_HAND]->addSphere(-0.08, 0.0, 0.02, 0.02, "11_2");
	STORE_SPHERE cover[LEFT_HAND]->addSphere(-0.02, 0.0, -0.02, 0.02, "11_3");
	STORE_SPHERE cover[LEFT_HAND]->addSphere(0.02, 0.0, 0.02, 0.016, "11_4");
	STORE_SPHERE cover[LEFT_HAND]->addSphere(-0.01, 0.0, -0.03, 0.016, "11_5");

	STORE_SPHERE cover[RIGHT_UPPER_ARM]->addSphere(0.0, 0.0, 0.0, 0.035, "14_0");
	STORE_SPHERE cover[RIGHT_UPPER_ARM]->addSphere(0.0, 0.06, 0.0, 0.045, "14_1");
	STORE_SPHERE cover[RIGHT_UPPER_ARM]->addSphere(0.0, 0.12, 0.0, 0.045, "14_2");
	STORE_SPHERE cover[RIGHT_UPPER_ARM]->addSphere(0.0, 0.18, 0.0, 0.045, "14_3");

	STORE_SPHERE cover[RIGHT_LOWER_ARM]->addSphere(0.0, 0.0, 0.0, 0.0375, "16_0");
	STORE_SPHERE cover[RIGHT_LOWER_ARM]->addSphere(0.0, 0.0, -0.04, 0.04, "16_1");
	STORE_SPHERE cover[RIGHT_LOWER_ARM]->addSphere(0.0, 0.0, -0.10, 0.0425, "16_2");
	STORE_SPHERE cover[RIGHT_LOWER_ARM]->addSphere(0.0, 0.0, -0.16, 0.045, "16_3");
	STORE_SPHERE cover[RIGHT_LOWER_ARM]->addSphere(0.0, 0.0, -0.22, 0.0425, "16_4");

	STORE_SPHERE cover[RIGHT_HAND]->addSphere(0.0, 0.0, -0.02, 0.02, "19_0");
	STORE_SPHERE cover[RIGHT_HAND]->addSphere(-0.04, 0.0, -0.01, 0.03, "19_1");
	STORE_SPHERE cover[RIGHT_HAND]->addSphere(-0.08, 0.0, -0.02, 0.02, "19_2");
	STORE_SPHERE cover[RIGHT_HAND]->addSphere(-0.02, 0.0, 0.02, 0.02, "19_3");
	STORE_SPHERE cover[RIGHT_HAND]->addSphere(0.02, 0.0,  -0.02, 0.016, "19_4");
	STORE_SPHERE cover[RIGHT_HAND]->addSphere(-0.01, 0.0, 0.03, 0.016, "19_5");

	interference.push_back(new Interference(cover[LEFT_LOWER_ARM], cover[TORSO], 4, 7));
	interference.push_back(new Interference(cover[LEFT_LOWER_ARM], cover[RIGHT_UPPER_ARM], 4, 7));
	interference.push_back(new Interference(cover[LEFT_LOWER_ARM], cover[RIGHT_LOWER_ARM], 4, 7));
	interference.push_back(new Interference(cover[LEFT_LOWER_ARM], cover[RIGHT_HAND], 4, 7));

	interference.push_back(new Interference(cover[RIGHT_LOWER_ARM], cover[TORSO], 12, 15));
	interference.push_back(new Interference(cover[RIGHT_LOWER_ARM], cover[LEFT_UPPER_ARM], 12, 15));
	interference.push_back(new Interference(cover[RIGHT_LOWER_ARM], cover[LEFT_LOWER_ARM], 12, 15));
	interference.push_back(new Interference(cover[RIGHT_LOWER_ARM], cover[LEFT_HAND], 12, 15));

	interference.push_back(new Interference(cover[LEFT_HAND], cover[TORSO], 4, 7));
	interference.push_back(new Interference(cover[LEFT_HAND], cover[RIGHT_UPPER_ARM], 4, 7));
	interference.push_back(new Interference(cover[LEFT_HAND], cover[RIGHT_LOWER_ARM], 4, 7));
	interference.push_back(new Interference(cover[LEFT_HAND], cover[RIGHT_HAND], 4, 7));

	interference.push_back(new Interference(cover[RIGHT_HAND], cover[TORSO], 12, 15));
	interference.push_back(new Interference(cover[RIGHT_HAND], cover[LEFT_UPPER_ARM], 12, 15));
	interference.push_back(new Interference(cover[RIGHT_HAND], cover[LEFT_LOWER_ARM], 12, 15));
	interference.push_back(new Interference(cover[RIGHT_HAND], cover[LEFT_HAND], 12, 15));

	Jself.resize(interference.size(), NJOINTS);
	Jgrav.resize(2, NJOINTS);

	Jhand[0].resize(6, NJOINTS);
	Jhand[1].resize(6, NJOINTS);

	selfDistance.resize(interference.size());
}
