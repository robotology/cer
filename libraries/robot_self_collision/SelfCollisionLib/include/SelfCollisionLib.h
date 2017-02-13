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

#ifndef __SELFCOLLISION_LIB_H__
#define __SELFCOLLISION_LIB_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

namespace cer
{
	namespace robot_model
	{
		class RobotModel;

		namespace self_collision
		{
			class SelfCollisionLib
			{
			public:
				SelfCollisionLib(int robot_type);
				~SelfCollisionLib();

				bool isOk();

				bool checkNextConfiguration(const yarp::sig::Vector& qnext, yarp::sig::Vector* margin, yarp::sig::Matrix *Jacobian = NULL);

				enum { R1_MODEL, ICUB_MODEL, ICUB3_MODEL };

			protected:
				RobotModel* robotModel;
			};
		}
	}
}

#endif