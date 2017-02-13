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

#include <RobotModel.h>

#define ROOT NULL

using namespace cer::robot_model;

const Matrix& RobotModel::calcInterference(Matrix &distance)
{
	distance.resize(selfDistance.R);

	if (selfDirty)
	{
		selfDirty = false;

		for (unsigned int c = 0; c < cover_list.size(); ++c)
		{
			cover_list[c]->pose(solid_part[cover_list[c]->partID]->Toj);
		}

		Vec3 Xa, Xb, Ud;

		for (unsigned int i = 0; i < interference.size(); ++i)
		{
			Cover *coverA = interference[i]->coverA;

			int partID = coverA->partID;

			Component *solidA = solid_part[partID];

			selfDistance(i) = repulsion(coverA, interference[i]->coverB, Xa, Xb, Ud);

			for (unsigned int d = 0; d < interference[i]->jdep.size(); ++d)
			{
				int j = interference[i]->jdep[d];

				Component *solid = solid_part[partID];

				Jself(i, j) = Ud * (solidA->Voj[j] + (solidA->Zoj[j] % (Xa - solidA->Poj[j])));
			}
		}

		distance = selfDistance;
	}

	return Jself;
}

/*
int RobotModel::calcInterference(Vec3* Xa, Vec3* Xb, Vec3* Ud, double *distance, Matrix &Jself)
{
	for (int c = 0; c < cover_list.size(); ++c)
	{
		cover_list[c]->pose(solid_part[cover_list[c]->partID]->Toj);
	}

	for (int i = 0; i < interference.size(); ++i)
	{
		Cover *coverA = interference[i]->coverA;

		int partID = coverA->partID;

		Component *solidA = solid_part[partID];

		distance[i] = repulsion(coverA, interference[i]->coverB, Xa[i], Xb[i], Ud[i]);

		for (int d = 0; d < interference[i]->jdep.size(); ++d)
		{
			int j = interference[i]->jdep[d];

			Component *solid = solid_part[partID];

			Jself(i, j) = Ud[i] * (solidA->Voj[j] + (solidA->Zoj[j] % (Xa[i] - solidA->Poj[j])));
		}
	}

	return interference.size();
}
*/
