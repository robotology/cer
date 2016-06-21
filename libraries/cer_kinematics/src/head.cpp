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

#include <yarp/os/Log.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>

#include <cer_kinematics/head.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace cer::kinematics;


/****************************************************************/
HeadSolver::HeadSolver(const HeadParameters &headParams, const int verb) :
                       Solver(verb),
                       headParameters(headParams),
                       torso(headParams.torso,verb)
{
}


/****************************************************************/
void HeadSolver::setHeadParameters(const HeadParameters &params)
{
    headParameters=params;
    torso.setParameters(headParameters.torso);
}


/****************************************************************/
void HeadSolver::setVerbosity(const int verb)
{
    verbosity=verb;
    torso.setVerbosity(verb);
}


/****************************************************************/
bool HeadSolver::setInitialGuess(const Vector &q0)
{
    yWarning("Solver does not use any initial guess");
    return true;
}


/****************************************************************/
Vector HeadSolver::getInitialGuess() const
{
    yWarning("Solver does not use any initial guess");
    return Vector(3+headParameters.head.getDOF(),0.0);
}


/****************************************************************/
bool HeadSolver::fkin(const Vector &q, Matrix &H, const int frame)
{
    size_t L=3+headParameters.head.getDOF();
    if (q.length()<L)
    {
        yError("mis-sized DOFs vector!");
        return false;
    }

    Vector p,u;
    torso.fkin(q,p,u);
    headParameters.head.setAng(CTRL_DEG2RAD*q.subVector(3,5));

    double n=norm(u);
    u/=n; u.push_back(n);
    H=axis2dcm(u);
    H.setSubcol(p,0,3);

    int frame_=(frame<0)?L-1:frame;
    if (frame_>=3)
        H*=headParameters.head.getH(std::min(frame_-3,(int)headParameters.head.getDOF()-1));

    return true;
}


/****************************************************************/
bool HeadSolver::ikin(const Vector &q0, const Vector &xd, Vector &q)
{
    if (xd.length()!=3)
    {
        yError("mis-sized desired fixation point!");
        return false;
    }

    Matrix H;
    if (fkin(q0,H))
    {
        Vector xd_=xd;
        xd_.push_back(1.0);
        xd_=SE3inv(H)*xd_;

        q.resize(2);
        q[0]=atan2(xd_[2],xd_[1]);
        q[1]=atan2(xd_[2],xd_[0]);

        q[0]+=q[0]>0.0?-M_PI/2.0:M_PI/2.0;
        q[1]+=q[1]>0.0?-M_PI/2.0:M_PI/2.0;

        q[0]=CTRL_RAD2DEG*headParameters.head.setAng(1,q[0]);
        q[1]=CTRL_RAD2DEG*headParameters.head.setAng(2,q[1]);
    }

    if (verbosity>0)
    {
        yInfo(" *** Head Solver ******************************");
        yInfo(" *** Head Solver:     head = %s",headParameters.head.getType().c_str());
        yInfo(" *** Head Solver:   q0 [*] = (%s)",q0.toString(4,4).c_str());
        yInfo(" *** Head Solver:   xd [m] = (%s)",xd.toString(4,4).c_str());
        yInfo(" *** Head Solver:  q [deg] = (%s)",q.toString(4,4).c_str());
        yInfo(" *** Head Solver ******************************");
    }

    return true;
}

