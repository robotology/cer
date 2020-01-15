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

#ifndef __CER_KINEMATICS_MOBILE_UTILS_H__
#define __CER_KINEMATICS_MOBILE_UTILS_H__

#include <limits>
#include <string>
#include <set>

#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>

#include <cer_kinematics/utils.h>

namespace cer {
namespace mobile_kinematics {

namespace configuration {
    enum {
        no_heave,
        heave,
        no_torso_no_heave,
        no_torso_heave,
        torso_yaw_no_heave
    };
};


/**
 * Structure used to initialize a solver.
 * 
 * @author Jason Chevrie
 */
struct MobileSolverParameters : public cer::kinematics::SolverParameters
{
    /**
     * Helper to set the internal state according to a string 
     * mode.\n The helper does also set suitable tolerance values. 
     *  
     * @param mode  a string that can be a combination of  
     *              ["full_pose"|"xyz_pose"]+["heave"|"no_heave"|"no_torso_no_heave"|"no_torso_heave"|"torso_yaw_no_heave"]+["forward_diff"|"central_diff"].
     *              Examples: "full_pose+central_diff",
     *              "xyz_pose+no_heave",
     *              "full_pose+heave+forward_diff".
     * @note the order might affect the setting of internal state, 
     *       therefore the preferred order is: pose + mode + diff.
     * @return true/false on success/failure. 
     */
    bool setMode(const std::string &mode) override;

    /**
     * Helper to retrieve the string corresponding to the internal 
     * state.
     *  
     * @return the string encoding the internal state. 
     */
    std::string getMode() const override;
};

}

}

#endif

