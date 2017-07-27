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

#include <cer_kinematics/utils.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace cer::kinematics;

namespace cer {
namespace kinematics {

yarp::os::Mutex Solver::makeThreadSafe;

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
struct CER_TORSO : public TripodParameters
{
    /****************************************************************/
    CER_TORSO() : TripodParameters(0.09,0.0,0.17,30.0)
    {
        Vector rot(4,0.0);
        rot[2]=1.0; rot[3]=M_PI;
        T0=axis2dcm(rot);
        T0(0,3)=0.044;
        T0(2,3)=0.470;
    }
};


/****************************************************************/
struct CER_LOWER_ARM : public TripodParameters
{
    /****************************************************************/
    CER_LOWER_ARM() : TripodParameters(0.018,0.0,0.13,30.0) { }
};


/****************************************************************/
class CER_UPPER_ARM : public iKinLimb
{
public:
    /****************************************************************/
    CER_UPPER_ARM(const string &type_) : iKinLimb(type_)
    {
        transform(type.begin(),type.end(),type.begin(),::tolower);
        if ((type!="left") && (type!="right"))
            type="left";

        allocate(type);
    }

protected:
    /****************************************************************/
    void allocate(const string &type)
    {
        if (type=="left")
        {
            pushLink(new iKinLink(-0.084, 0.325869, 104.0*CTRL_DEG2RAD, 180.0*CTRL_DEG2RAD,-60.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD)); 
            pushLink(new iKinLink(   0.0,-0.182419,  90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD,-30.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
            pushLink(new iKinLink( 0.034,      0.0, -90.0*CTRL_DEG2RAD,-104.0*CTRL_DEG2RAD,-10.0*CTRL_DEG2RAD, 75.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(   0.0,   -0.251,  90.0*CTRL_DEG2RAD, -90.0*CTRL_DEG2RAD,-90.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(   0.0,      0.0, -90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD,  0.0*CTRL_DEG2RAD,101.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(   0.0,   -0.291,-180.0*CTRL_DEG2RAD, -90.0*CTRL_DEG2RAD,-90.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
        }
        else
        {
            pushLink(new iKinLink(-0.084,0.325869, 76.0*CTRL_DEG2RAD, 180.0*CTRL_DEG2RAD,-60.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD)); 
            pushLink(new iKinLink(   0.0,0.182419, 90.0*CTRL_DEG2RAD, -90.0*CTRL_DEG2RAD,-30.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(-0.034,     0.0,-90.0*CTRL_DEG2RAD,-104.0*CTRL_DEG2RAD,-10.0*CTRL_DEG2RAD, 75.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(   0.0,   0.251,-90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD,-90.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(   0.0,     0.0, 90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD,  0.0*CTRL_DEG2RAD,101.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(   0.0,   0.291,  0.0*CTRL_DEG2RAD, -90.0*CTRL_DEG2RAD,-90.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
        }
    }
};


/****************************************************************/
class CER_HEAD : public iKinLimb
{
public:
    /****************************************************************/
    CER_HEAD(const string &type_) : iKinLimb(type_)
    {        
        transform(type.begin(),type.end(),type.begin(),::tolower);
        set<string> types=HeadParameters::getTypes();
        if (types.find(type)==types.end())
            type="gaze";

        allocate(type);
    }

protected:
    /****************************************************************/
    void allocate(const string &type)
    {
        pushLink(new iKinLink(  -0.094,     0.4,-90.0*CTRL_DEG2RAD,180.0*CTRL_DEG2RAD,-60.0*CTRL_DEG2RAD,60.0*CTRL_DEG2RAD)); 
        pushLink(new iKinLink(  -0.016,     0.0, 90.0*CTRL_DEG2RAD, 10.0*CTRL_DEG2RAD,-30.0*CTRL_DEG2RAD,50.0*CTRL_DEG2RAD)); 
        pushLink(new iKinLink(-0.03351,0.089481,-80.0*CTRL_DEG2RAD,-90.0*CTRL_DEG2RAD,-80.0*CTRL_DEG2RAD,80.0*CTRL_DEG2RAD)); 

        unsigned int lastLink=getN()-1;
        if (type=="right")
            (*this)[lastLink].setA(-(*this)[lastLink].getA());
        else if (type=="cyclopic")
            (*this)[lastLink].setA(0.0);
        else if (type=="gaze")
        {
            (*this)[lastLink].setA(0.0);
            (*this)[lastLink].setD(0.143908);
        }
        else if (type=="depth")
        {
            (*this)[lastLink].setA(-0.044487);
            (*this)[lastLink].setD(0.202701);
        }
        else if (type=="depth_rgb")
        {
            (*this)[lastLink].setA(-0.018487);
            (*this)[lastLink].setD(0.202701);
        }
        else if (type=="depth_center")
        {
            (*this)[lastLink].setA(0.0);
            (*this)[lastLink].setD(0.202701);
        }

        Matrix HN=eye(4,4);
        if (type=="gaze")
            HN(2,3)=0.051929;
        else if ((type=="depth") || (type=="depth_rgb"))        
            HN(2,3)=0.052119;
        else if (type=="depth_center")
            HN(2,3)=0.052114;
        else
            HN(2,3)=0.064065;
        setHN(HN);
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
               torso(CER_TORSO()),
               upper_arm(CER_UPPER_ARM(type)),
               lower_arm(CER_LOWER_ARM())
{
    TN=zeros(4,4);
    TN(0,0)=0.258819; TN(0,2)=-0.965926; TN(0,3)=0.0269172;
    TN(1,1)=1.0;
    TN(2,0)=-TN(0,2); TN(2,2)=TN(0,0); TN(2,3)=0.100456;
    TN(3,3)=1.0;
    if (upper_arm.getType()=="right")
    {
        TN(0,2)=-TN(0,2);
        TN(1,1)=-TN(1,1);
        TN(2,0)=TN(0,2);
        TN(2,2)=-TN(0,0);
    }
}


/****************************************************************/
HeadParameters::HeadParameters(const string &type) : 
                torso(CER_TORSO()), head(CER_HEAD(type))
{
}


/****************************************************************/
set<string> HeadParameters::getTypes()
{
    set<string> types;
    types.insert("left");
    types.insert("cyclopic");
    types.insert("right");
    types.insert("gaze");
    types.insert("depth");
    types.insert("depth_rgb");
    types.insert("depth_center");
    return types;
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
                tol=1e-2;
                constr_tol=1e-5;
            }
            else if (submode=="xyz_pose")
            {
                full_pose=false;
                tol=1e-5;
                constr_tol=1e-4;
            }
            if (submode=="heave")
                configuration=configuration::heave;
            else if (submode=="no_heave")
                configuration=configuration::no_heave;
            else if (submode=="no_torso_heave")
                configuration=configuration::no_torso_heave;
            else if (submode=="no_torso_no_heave")
                configuration=configuration::no_torso_no_heave;
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
string SolverParameters::getMode() const
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

    mode+="+";
    mode+=(use_central_difference?"central_diff":"forward_diff");

    return mode;
}

