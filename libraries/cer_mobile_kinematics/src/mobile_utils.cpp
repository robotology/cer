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

#include <cmath>
#include <algorithm>

#include <iCub/ctrl/math.h>

#include <cer_mobile_kinematics/mobile_utils.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace cer::kinematics;
using namespace cer::mobile_kinematics;

namespace cer {
namespace mobile_kinematics {

/****************************************************************/
bool stepModeParser(string &mode, string &submode)
{
    submode=mode;
    size_t found=mode.find_first_of('+');
    if ((found>0) && (found!=string::npos))
    {
        submode=mode.substr(0,found);
        if (found+1<mode.length())
            mode=mode.substr(found+1,mode.length()-found);
        else
            mode.clear();
    }
    else
        mode.clear();    

    return !submode.empty();
}

}

}


/****************************************************************/
bool MobileSolverParameters::setMode(const string &mode)
{
    string mode_=mode;
    string submode;
    bool ret=true;

    while (!mode_.empty())
    {
        if (stepModeParser(mode_,submode))
        {
            if (submode=="full_pose")
            {
                full_pose=true;
                tol=1e-2;
                constr_tol=1e-5;
            }
            else if (submode=="xyz_pose")
            {
                full_pose=false;
                tol=1e-5;
                constr_tol=1e-4;
            }
            else if (submode=="heave")
                configuration=configuration::heave;
            else if (submode=="no_heave")
                configuration=configuration::no_heave;
            else if (submode=="no_torso_heave")
                configuration=configuration::no_torso_heave;
            else if (submode=="no_torso_no_heave")
                configuration=configuration::no_torso_no_heave;
            else if (submode=="torso_yaw_no_heave")
                configuration=configuration::torso_yaw_no_heave;
            else if (submode=="forward_diff")
                use_central_difference=false;                
            else if (submode=="central_diff")
                use_central_difference=true;
            else
                ret=false;
        }
    }

    return ret;
}


/****************************************************************/
string MobileSolverParameters::getMode() const
{
    string mode=(full_pose?"full_pose":"xyz_pose");

    mode+="+";
    if (configuration==configuration::heave)
        mode+="heave";
    else if (configuration==configuration::no_heave)
        mode+="no_heave";
    else if (configuration==configuration::no_torso_heave)
        mode+="no_torso_heave";
    else if (configuration==configuration::no_torso_no_heave)
        mode+="no_torso_no_heave";
    else if (configuration==configuration::torso_yaw_no_heave)
        mode+="torso_yaw_no_heave";

    mode+="+";
    mode+=(use_central_difference?"central_diff":"forward_diff");

    return mode;
}

