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

#include <deque>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <IpTNLP.hpp>

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
    TripodParametersExtended(const TripodParameters &parameters);
};


/****************************************************************/
struct TripodState
{
    yarp::sig::Vector n,u,p;
    yarp::sig::Matrix T;

    /****************************************************************/
    TripodState():n(3,0.0),u(4,0.0),p(3,0.0),T(yarp::math::eye(4,4)) { }
};


/****************************************************************/
class TripodNLPHelper
{
protected:
    /****************************************************************/
    TripodState fkinHelper(const Ipopt::Number *x,
                           const TripodParametersExtended &params,
                           TripodState *internal=NULL);
};

}

}

#endif

