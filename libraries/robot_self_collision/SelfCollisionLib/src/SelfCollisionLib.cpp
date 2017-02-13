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

#include <SelfCollisionLib.h>

#include <R1Model.h>
//#include <iCubModel.h>
//#include <iCub3Model.h>

using namespace cer::robot_model::self_collision;

SelfCollisionLib::SelfCollisionLib(int robot_type)
{
	robotModel = NULL;

	switch (robot_type)
	{
	case R1_MODEL:
		robotModel = new R1Model();
		return;

	case ICUB_MODEL:
		//robotModel = new iCubModel();
		//return;

	case ICUB3_MODEL:
		//robotModel = new iCub3Model();
		//return;

	default:
		printf("Robot type not available in model library.\n");
		return;
	}
}

SelfCollisionLib::~SelfCollisionLib()
{
	if (robotModel) delete robotModel;
}

bool SelfCollisionLib::isOk()
{
	return robotModel != NULL;
}

bool SelfCollisionLib::checkNextConfiguration(const yarp::sig::Vector& qnext, yarp::sig::Vector* margin, yarp::sig::Matrix* Jacobian)
{
	if (!robotModel)
	{
		printf("Robot type not set.\n");

		return false;
	}

	int dof = robotModel->getNDOF();

	if (qnext.length() != dof) return false;

	Matrix q(dof);

	for (int l = 0; l < dof; ++l) q(l) = qnext[l];

	robotModel->calcConfig(q);

	Matrix distance;

	const Matrix& J = robotModel->calcInterference(distance);

	if (Jacobian)
	{
		Jacobian->resize(J.R, J.C);

		for (int r = 0; r < J.R; ++r)
		{
			for (int c = 0; c < J.C; ++c)
			{
				(*Jacobian)(r, c) = J(r, c);
			}
		}
	}

	bool freespace = true;

	if (margin)
	{
		margin->resize(distance.R);

		for (int d = 0; d < distance.R; ++d)
		{
			(*margin)[d] = distance(d);

			if (distance(d) < 0.0) freespace = false;
		}
	}

	return freespace;
}