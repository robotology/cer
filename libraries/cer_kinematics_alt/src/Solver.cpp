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

#include <cer_kinematics_alt/Solver.h>

#include <cer_kinematics_alt/private/SolverImpl.h>

using namespace cer::kinematics_alt;

LeftSideSolver::LeftSideSolver()
{
    solverImpl=new LeftSideSolverImpl();
}

LeftSideSolver::~LeftSideSolver()
{
    delete solverImpl;
}

yarp::sig::Vector LeftSideSolver::getCOM()
{
    yarp::sig::Vector com(3);
    
    Vec3 G=solverImpl->getCOM();
    com(0)=G.x;
    com(1)=G.y;
    com(2)=G.z;

    return com;
}

void LeftSideSolver::setPositionThreshold(double thr)
{ 
    solverImpl->setPositionThreshold(thr); 
}

bool LeftSideSolver::fkin(const yarp::sig::Vector &qin, yarp::sig::Matrix &H,int frame)
{
    return solverImpl->fkin(qin,H,frame);
}

bool LeftSideSolver::ikin(const yarp::sig::Matrix &Hd, yarp::sig::Vector &qout,double armElong,double torsoElong,double timeoutSec)
{
    return solverImpl->ikin(Hd,qout,armElong,torsoElong,timeoutSec);
}
/*
bool LeftSideSolver::controller(yarp::sig::Vector &qposin, yarp::sig::Vector &Vstar, yarp::sig::Vector &Wstar, yarp::sig::Vector &qvelout)
{
    return solverImpl->controller(qposin, Vstar, Wstar, qvelout);
}
*/