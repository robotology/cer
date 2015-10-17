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

#include <string>
#include <yarp/sig/Matrix.h>
#include <iCub/iKin/iKinFwd.h>

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
    TripodParameters(const double r_=0.09, const double l_min_=-0.05,
                     const double l_max_=0.15, const double alpha_max_=30.0) :
                     r(r_), l_min(l_min_), l_max(l_max_), alpha_max(alpha_max_) { }
};


/**
 * Structure used to initialize a robot arm.
 * 
 * @author Ugo Pattacini
 */
struct ArmParameters
{
    /**
     * the 4-by-4 homogeneous matrix linking the root frame and the 
     * torso. 
     */
    yarp::sig::Matrix T0;

    /**
     * the tripod mechanism for the torso.
     */
    TripodParameters torso;

    /**
     * the serial chain for the upper_arm.
     */
    iCub::iKin::iKinLimb upper_arm;

    /**
     * the tripod mechanism for the lower_arm.
     */
    TripodParameters lower_arm;

    /**
     * the 4-by-4 homogeneous matrix linking the lower_arm with the 
     * end-effector frame. 
     */
    yarp::sig::Matrix TN;

    /**
     * Constructor. 
     *  
     * @param type  a string ["left"|"right"] accounting for the 
     *              hand type.
     */
    ArmParameters(const std::string &type="left");
};


/**
 * Structure used to initialize a solver.
 * 
 * @author Ugo Pattacini
 */
struct SolverParameters
{
    /**
     * if true reaching in position and orientation is enabled; 
     * reaching only in position is selected otherwise. 
     */
    bool full_pose;

    /**
     * if true the robot is allowed to heave.
     */
    bool can_heave;

    /**
     * desired heave assumed by the torso.
     */
    double torso_heave;

    /**
     * desired heave assumed by the lower_arm.
     */
    double lower_arm_heave;

    /**
     * Constructor. 
     *  
     * @param full_pose_        enable/disable full pose reaching. 
     * @param can_heave_        enable/disable robot heave. 
     * @param torso_heave_      the desired heave assumed by the torso. 
     * @param lower_arm_heave_  the desired heave assumed by the 
     *                          lower arm.
     */
    SolverParameters(const bool full_pose_=true, const bool can_heave_=false,
                     const double torso_heave_=0.0, const double lower_arm_heave_=0.0) :
                     full_pose(full_pose_), can_heave(can_heave_),
                     torso_heave(torso_heave_), lower_arm_heave(lower_arm_heave_) { }

    /**
     * Helper that sets full_pose and can_heave according to a mode 
     * string.
     *  
     * @param mode  a string that can be: 
     *              ["full"|"full+heave"|"xyz"|"xyz+heave"].
     * @return true/false on success/failure. 
     */
    bool setMode(const std::string &mode);
};

}

#endif

