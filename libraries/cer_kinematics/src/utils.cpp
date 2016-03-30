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
#include <iCub/ctrl/math.h>
#include <cer_kinematics/utils.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace cer::kinematics;

namespace cer {
namespace kinematics {

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


/****************************************************************/
class UpperArm : public iKinLimb
{
public:
    /****************************************************************/
    UpperArm(const string &type_) : iKinLimb(type_)
    {
        if ((type!="right") && (type!="left"))
            type="left";

        allocate(type);
    }

protected:
    /****************************************************************/
    void allocate(const string &type)
    {
        Matrix HN=eye(4,4); 
        HN(2,3)=0.22;
        setHN(HN);

        if (type=="left")
        {
            pushLink(new iKinLink(-0.084, 0.297614,104.0*CTRL_DEG2RAD, 180.0*CTRL_DEG2RAD,-60.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD)); 
            pushLink(new iKinLink(   0.0,-0.175204, 90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD,-20.0*CTRL_DEG2RAD,100.0*CTRL_DEG2RAD));
            pushLink(new iKinLink( 0.034,      0.0,-90.0*CTRL_DEG2RAD,-104.0*CTRL_DEG2RAD,-10.0*CTRL_DEG2RAD,100.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(   0.0,   -0.251, 90.0*CTRL_DEG2RAD, -90.0*CTRL_DEG2RAD,-90.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(   0.0,      0.0,-90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD,  0.0*CTRL_DEG2RAD,140.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(   0.0,   -0.071,180.0*CTRL_DEG2RAD, -90.0*CTRL_DEG2RAD,-90.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
        }
        else
        {
            pushLink(new iKinLink(-0.084,0.297614, 76.0*CTRL_DEG2RAD,-180.0*CTRL_DEG2RAD,-60.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD)); 
            pushLink(new iKinLink(   0.0,0.175204, 90.0*CTRL_DEG2RAD, -90.0*CTRL_DEG2RAD,-20.0*CTRL_DEG2RAD,100.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(-0.034,     0.0,-90.0*CTRL_DEG2RAD,-104.0*CTRL_DEG2RAD,-10.0*CTRL_DEG2RAD,100.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(   0.0,   0.251,-90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD,-90.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(   0.0,     0.0, 90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD,  0.0*CTRL_DEG2RAD,140.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(   0.0,   0.071,  0.0*CTRL_DEG2RAD, -90.0*CTRL_DEG2RAD,-90.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
        }
    }
};

}

}


/****************************************************************/
TripodParameters::TripodParameters(const double r_, const double l_min_,
                                   const double l_max_, const double alpha_max_,
                                   const Matrix T0_) :
                  r(r_), l_min(l_min_),  l_max(l_max_),
                  alpha_max(alpha_max_), T0(T0_)
{
    yAssert((T0.rows()==4) && (T0.cols()==4));
}


/****************************************************************/
ArmParameters::ArmParameters(const string &type) :
               torso(0.09,-0.05,0.15,30.0),
               upper_arm(UpperArm(type)),
               lower_arm(0.018,0.0,0.14,35.0)
{
    Vector rot(4,0.0);
    rot[2]=1.0; rot[3]=M_PI;
    torso.T0=axis2dcm(rot);

    double hand_ang=CTRL_DEG2RAD*20.0;
    double hand_dist=0.07;
    rot=0.0;
    rot[1]=1.0; rot[3]=-M_PI/2.0+hand_ang;
    TN=axis2dcm(rot);
    TN(0,3)=hand_dist*sin(hand_ang);
    TN(2,3)=hand_dist*cos(hand_ang);
}


/****************************************************************/
bool SolverParameters::setMode(const string &mode)
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
                tol=0.1;
                constr_tol=1e-6;
            }
            else if (submode=="xyz_pose")
            {
                full_pose=false;
                tol=constr_tol=1e-6;
            }
            if (submode=="heave")
                configuration=configuration::heave;
            else if (submode=="no_heave")
                configuration=configuration::no_heave;
            else if (submode=="no_torso")
                configuration=configuration::no_torso;
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

