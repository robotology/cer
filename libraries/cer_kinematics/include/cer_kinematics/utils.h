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

#ifndef __CER_KINEMATICS_UTILS_H__
#define __CER_KINEMATICS_UTILS_H__

namespace cer_kinematics
{

/**
 * Structure used to initialize a tripod mechanism.
 * 
 * @author Ugo Pattacini
 */
struct TripodParameters
{
    /**
     * the radius ([m]).
     */
    double r;

    /**
     * the minimum elongation ([m]).
     */
    double l_min;

    /**
     * the minimum elongation ([m]).
     */
    double l_max;

    /**
     * the maximum permitted bending angle ([deg]).
     */
    double alpha_max;

    /**
     * Constructor.
     * 
     * @param r_        the radius ([m]).
     * @param l_min_    the minimum elongation ([m]).
     * @param l_max_    the maximum elongation ([m]).
     * @param alpha_max the maximum permitted bending angle ([deg]).
     */
    TripodParameters(const double r_=0.09, const double l_min_=0.0,
                     const double l_max_=0.2, const double alpha_max_=30.0) :
                     r(r_), l_min(l_min_), l_max(l_max_), alpha_max(alpha_max_) { }
};

}

#endif

