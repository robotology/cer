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

#ifndef __CER_SOLVER_H__
#define __CER_SOLVER_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

namespace cer {
namespace kinematics_alt {

#define PERIOD 0.01 // seconds

#define SOLVER_TIMEOUT 40 // steps
#define WRIST_MAX_TILT 35.0 // [deg]
#define TORSO_MAX_TILT 30.0 // [deg]
#define DEFAULT_ARM_EXTENSION   0.01  // [m]
#define DEFAULT_TORSO_EXTENSION 0.0   // [m]
#define DEFAULT_POS_THRESHOLD   0.005 // [m]

// joint identifiers
enum 
{
    TORSO_TRIPOD_1   =  0,
    TORSO_TRIPOD_2   =  1,
    TORSO_TRIPOD_3   =  2,
    TORSO_YAW        =  3,

    L_SHOULDER_PITCH =  4,    
    L_SHOULDER_ROLL  =  5,
    L_SHOULDER_YAW   =  6,
    L_ELBOW          =  7,
    L_PROSUPINATION  =  8,
    L_WRIST_TRIPOD_1 =  9,
    L_WRIST_TRIPOD_2 = 10,
    L_WRIST_TRIPOD_3 = 11,
    LEFT_HAND        = L_WRIST_TRIPOD_3, //alias

    R_SHOULDER_PITCH = 12,    
    R_SHOULDER_ROLL  = 13,
    R_SHOULDER_YAW   = 14,
    R_ELBOW          = 15,
    R_PROSUPINATION  = 16,
    R_WRIST_TRIPOD_1 = 17,
    R_WRIST_TRIPOD_2 = 18,
    R_WRIST_TRIPOD_3 = 19,
    RIGHT_HAND       = R_WRIST_TRIPOD_3, // alias

    NECK_PITCH       = 20,
    NECK_YAW         = 21,
    HEAD             = NECK_YAW // alias
};

class LeftSideSolverImpl;

/**
 * Class to handle direct and inverse kinematics of the robot arm.
 *
 * @author Alessandro Scalzo
 */
class LeftSideSolver
{
public:

    LeftSideSolver();

    virtual ~LeftSideSolver();

    yarp::sig::Vector getCOM();

    /**
     * Set the required precision for position reaching.
     *
     * @param thr the threshold for the position reaching ([m]).
     */
    void setPositionThreshold(double thr=DEFAULT_POS_THRESHOLD);

    /**
     * Forward Kinematics Law.
     *
     * @param qin      the DOFs values ([m]-[deg]-[m]).
     * @param H        the 4-by-4 homogeneous matrix of the specified
     *                 frame ([m]).
     * @param frame    specify the DOF number whose frame is returned.
     *                 Thus, frame is in [0...nDOF-1]; negative
     *                 numbers account for the end-effector frame.
     * @return true/false on success/failure.
     */
    bool fkin(const yarp::sig::Vector &qin, yarp::sig::Matrix &H,int frame=-1);

    /**
     * Inverse Kinematics Law.
     *
     * @param Hd         the desired 4-by-4 homogeneous matrix
     *                   representing the end-effector frame ([m]).
     * @param qout       the solved DOFs ([m]-[deg]-[m]).
     * @param armElong   the desired elongation of the left forearm tripod ([m]).
     * @param torsoElong the desired elongation of the torso tripod ([m]).
     * @return true/false on success/failure.
     */
    bool ikin(const yarp::sig::Matrix &Hd, yarp::sig::Vector &qout,double armElong=DEFAULT_ARM_EXTENSION,double torsoElong=DEFAULT_TORSO_EXTENSION);

protected:
    LeftSideSolverImpl *solverImpl;
};

}
}

#endif


