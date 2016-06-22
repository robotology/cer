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

#include <limits>
#include <string>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>

namespace cer {
namespace kinematics {

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
     * the maximum elongation ([m]).
     */
    double l_max;

    /**
     * the maximum permitted bending angle ([deg]).
     */
    double alpha_max;

    /**
     * the 4-by-4 homogeneous matrix linking the root frame to the 
     * tripod ([m]).
     */
    yarp::sig::Matrix T0;

    /**
     * Constructor.
     * 
     * @param r_        the radius ([m]).
     * @param l_min_    the minimum elongation ([m]).
     * @param l_max_    the maximum elongation ([m]).
     * @param alpha_max the maximum permitted bending angle ([deg]). 
     * @param T0_       the 4-by-4 root-to-tripod matrix. 
     */
    TripodParameters(const double r_=0.1, const double l_min_=0.0,
                     const double l_max_=0.2, const double alpha_max_=30.0,
                     const yarp::sig::Matrix T0_=yarp::math::eye(4,4));
};


/**
 * Structure used to initialize a robot arm.
 * 
 * @author Ugo Pattacini
 */
struct ArmParameters
{
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
     * end-effector frame ([m]).
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
 * Structure used to initialize a robot head.
 * 
 * @author Ugo Pattacini
 */
struct HeadParameters
{
    /**
     * the tripod mechanism for the torso.
     */
    TripodParameters torso;

    /**
     * the serial chain for the upper_arm.
     */
    iCub::iKin::iKinLimb head;

    /**
     * Constructor. 
     *  
     * @param type  a string ["left"|"center"|"right"] accounting 
     *              for the end frame type.
     */
    HeadParameters(const std::string &type="center");
};


namespace configuration {
    enum {
        no_heave,
        heave,
        no_torso
    };
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
     * select the solver configuration: configuration::no_heave, 
     * configuration::heave, configuration::no_torso. 
     */
    int configuration;

    /**
     * desired heave assumed by the torso.
     */
    double torso_heave;

    /**
     * desired heave assumed by the lower_arm.
     */
    double lower_arm_heave;

    /**
     * weight for postural task of the torso pitch and roll.
     */
    double weight_postural_torso;

    /**
     * weight for postural task of the torso yaw.
     */
    double weight_postural_torso_yaw;

    /**
     * weight for postural task of the upper_arm.
     */
    double weight_postural_upper_arm;

    /**
     * weight for postural task of the lower_arm.
     */
    double weight_postural_lower_arm;

    /**
     * cost function tolerance.
     */
    double tol;

    /**
     * constraints tolerance.
     */
    double constr_tol;

    /**
     * maximum number of iterations.
     */
    int max_iter;

    /**
     * maximum number of cpu seconds.
     */
    double max_cpu_time;

    /**
     * if true compute gradients resorting to central finite 
     * difference approximation instead of forward difference 
     * formula. 
     */
    bool use_central_difference;

    /**
     * if true enable warm start.
     */
    bool warm_start;

    /**
     * Constructor. 
     *  
     * @param full_pose_                    enable/disable full pose
     *                                      reaching.
     * @param configuration_                select solving 
     *                                      configuration.
     * @param torso_heave_                  the desired heave 
     *                                      assumed by the torso.
     * @param lower_arm_heave_              the desired heave 
     *                                      assumed by the lower
     *                                      arm.
     * @param weight_postural_torso_        weight for postural task
     *                                      of the torso pitch and
     *                                      roll.
     * @param weight_postural_torso_yaw_    weight for postural task
     *                                      of the torso yaw.
     * @param weight_postural_upper_arm_    weight for postural task
     *                                      of the upper_arm.
     * @param weight_postural_lower_arm_    weight for postural task
     *                                      of the lower_arm.
     * @param tol_                          cost function tolerance.
     * @param constr_tol_                   constraints tolerance. 
     * @param max_iter_                     maximum number of 
     *                                      iterations.
     * @param max_cpu_time_                 maximum number of cpu 
     *                                      seconds.
     * @param use_central_difference_       compute gradients 
     *                                      resorting to central
     *                                      finite difference
     *                                      approximation instead of
     *                                      forward difference
     *                                      formula.
     * @param warm_start_                   enable warm start.
     */
    SolverParameters(const bool full_pose_=true, const bool configuration_=configuration::no_heave,
                     const double torso_heave_=0.0, const double lower_arm_heave_=0.0,
                     const double weight_postural_torso_=0.0,
                     const double weight_postural_torso_yaw_=0.0,
                     const double weight_postural_upper_arm_=0.0,
                     const double weight_postural_lower_arm_=0.0,
                     const double tol_=0.1, const double constr_tol_=1e-5,
                     const int max_iter_=std::numeric_limits<int>::max(),
                     const double max_cpu_time_=1.0,
                     const bool use_central_difference_=false,
                     const bool warm_start_=false) :
                     full_pose(full_pose_), configuration(configuration_),
                     torso_heave(torso_heave_), lower_arm_heave(lower_arm_heave_),
                     weight_postural_torso(weight_postural_torso_),
                     weight_postural_torso_yaw(weight_postural_torso_yaw_),
                     weight_postural_upper_arm(weight_postural_upper_arm_),
                     weight_postural_lower_arm(weight_postural_lower_arm_),
                     tol(tol_), constr_tol(constr_tol_),
                     max_iter(max_iter_), max_cpu_time(max_cpu_time_),
                     use_central_difference(use_central_difference_),
                     warm_start(warm_start_) { }

    /**
     * Helper to internal state according to a string mode.\n 
     * The helper does also set suitable tolerance values.
     *  
     * @param mode  a string that can be a combination of  
     *              ["full_pose"|"xyz_pose"]+["heave"|"no_heave"|"no_torso"]+["forward_diff"|"central_diff"].
     *              Examples: "full_pose+central_diff",
     *              "xyz_pose+no_heave",
     *              "full_pose+heave+forward_diff".
     * @note the order might affect the setting of internal state, 
     *       therefore the preferred order is: pose + mode + diff.
     * @return true/false on success/failure. 
     */
    bool setMode(const std::string &mode);
};


/**
 * Class to handle callbacks to be attached to solver iterates.
 * 
 * @author Ugo Pattacini
 */
class SolverIterateCallback
{
public:
    /**
     * Constructor.
     */
    SolverIterateCallback() { }

    /**
    * Defines the callback body to be called at each iteration. 
    * @param iter   the number of the current iteration. 
    * @param Hd     the desired 4-by-4 homogeneous matrix [m].
    * @param q      the DOFs of the iteration.
    * @param Hee    the enf-effector 4-by-4 homogeneous matrix [m]. 
    * @return true to stop the solver abruptly, false to let it run.
    */ 
    virtual bool exec(const int iter, const yarp::sig::Matrix &Hd,
                      const yarp::sig::Vector &q, const yarp::sig::Matrix &Hee)=0;

    /**
     * Destructor.
     */
    virtual ~SolverIterateCallback() { }
};


/**
 * Class to handle direct and inverse kinematics of the robot 
 * arm. 
 * 
 * @author Ugo Pattacini
 */
class Solver
{
protected:
    SolverIterateCallback *callback;
    int verbosity;

public:
    /**
     * Constructor.
     * 
     * @param verb  integers greater than 0 enable successive levels 
     *              of verbosity (default=0).
     */
    Solver(const int verb=0) : callback(NULL), verbosity(verb) { }

    /**
     * Specify new verbosity level.
     * 
     * @param verb   the verbosity level.
     */
    virtual void setVerbosity(const int verb)
    {
        verbosity=verb;
    }

    /**
     * Retrieve verbosity level.
     * 
     * @return the current verbosity level.
     */
    virtual int getVerbosity() const
    {
        return verbosity;
    }

    /**
     * Specify the initial DOFs values.
     * 
     * @param q0   initial DOFs values. 
     * @return true/false on success/failure. 
     */
    virtual bool setInitialGuess(const yarp::sig::Vector &q0)=0;

    /**
     * Retrieve the initial guess used for DOFs.
     * 
     * @return the initial DOFs values.
     */
    virtual yarp::sig::Vector getInitialGuess() const=0;

    /**
     * Enable iterate callbacks.
     *  
     * @param clbk  the object defining the callback.
     * @return true/false on success/failure.
     */
    virtual void enableIterateCallback(SolverIterateCallback &clbk)
    {
        callback=&clbk;
    }

    /**
     * Disable iterate callbacks.
     *  
     * @return true/false on success/failure.
     */
    virtual void disableIterateCallback()
    {
        callback=NULL;
    }

    /**
     * Destructor.
     */
    virtual ~Solver() { }
};

}

}

#endif

