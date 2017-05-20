/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#ifndef __CER_KINEMATICS_HELPERS_H__
#define __CER_KINEMATICS_HELPERS_H__

#include <cmath>
#include <deque>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>

#include <cer_kinematics/utils.h>

namespace cer {
namespace kinematics {

/****************************************************************/
struct TripodParametersExtended : public TripodParameters
{
    double cos_alpha_max;
    std::deque<yarp::sig::Vector> s;
    yarp::sig::Vector z;

    /****************************************************************/
    TripodParametersExtended(const TripodParameters &parameters) :
                             TripodParameters(parameters)
    {
        cos_alpha_max=cos(iCub::ctrl::CTRL_DEG2RAD*alpha_max);

        z.resize(3,0.0);
        z[2]=1.0;

        yarp::sig::Vector v(3,0.0);
        double theta=0.0;
        for (int i=0; i<3; i++)
        {            
            v[0]=r*cos(theta);
            v[1]=r*sin(theta);
            s.push_back(v);

            theta+=iCub::ctrl::CTRL_DEG2RAD*120.0;
        }
    }
};


/****************************************************************/
struct TripodState
{
    yarp::sig::Vector n,u,p;
    yarp::sig::Matrix T;

    /****************************************************************/
    TripodState():n(3,0.0),u(4,0.0),p(3,0.0),T(yarp::math::eye(4,4)) { }
};

}

}

#endif

