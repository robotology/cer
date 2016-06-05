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

KinR1::KinR1()
{
    kinR1Impl=new KinR1Impl();
}

KinR1::~KinR1()
{
    delete kinR1Impl;
}

yarp::sig::Vector KinR1::getCOM()
{
    yarp::sig::Vector com(3);
    
    Vec3 G=kinR1Impl->getCOM();
    com(0)=G.x;
    com(1)=G.y;
    com(2)=G.z;

    return com;
}

void KinR1::getConfig(yarp::sig::Vector& q)
{
    kinR1Impl->getConfig(q);
}

void KinR1::setPositionThreshold(double thr)
{ 
    kinR1Impl->setPositionThreshold(thr); 
}

bool KinR1::fkin(const yarp::sig::Vector &qin, yarp::sig::Matrix &H,int frame)
{
    return kinR1Impl->fkin(qin,H,frame);
}

bool KinR1::ikin_left_solver(const yarp::sig::Matrix &Hd, yarp::sig::Vector &qout,double armElongL,double armElongR,double torsoElong,double timeoutSec)
{
    return kinR1Impl->ikin_left_solver(Hd,qout,armElongL,armElongR,torsoElong,timeoutSec);
}

bool KinR1::ikin_left_ctrl(const yarp::sig::Matrix &Hd, yarp::sig::Vector &qin, yarp::sig::Vector &qdotout, double armElongL, double armElongR, double torsoElong)
{
    return kinR1Impl->ikin_left_ctrl(Hd,qin,qdotout,armElongL,armElongR,torsoElong);
}

bool KinR1::ikin_right_solver(const yarp::sig::Matrix &Hd, yarp::sig::Vector &qout,double armElongL,double armElongR,double torsoElong,double timeoutSec)
{
    return kinR1Impl->ikin_right_solver(Hd,qout,armElongL,armElongR,torsoElong,timeoutSec);
}

bool KinR1::ikin_right_ctrl(const yarp::sig::Matrix &Hd, yarp::sig::Vector &qin, yarp::sig::Vector &qdotout, double armElongL, double armElongR, double torsoElong)
{
    return kinR1Impl->ikin_right_ctrl(Hd,qin,qdotout,armElongL,armElongR,torsoElong);
}

bool KinR1::ikin_2hand_solver(const yarp::sig::Matrix &HdL, const yarp::sig::Matrix &HdR, yarp::sig::Vector &qout, double armElongL, double armElongR, double torsoElong, double timeoutSec)
{
    return kinR1Impl->ikin_2hand_solver(HdL,HdR,qout,armElongL,armElongR,torsoElong,timeoutSec);
}

bool KinR1::ikin_2hand_ctrl(const yarp::sig::Matrix &HdL, const yarp::sig::Matrix &HdR, yarp::sig::Vector &qin, yarp::sig::Vector &qdotout, double armElongL, double armElongR, double torsoElong)
{
    return kinR1Impl->ikin_2hand_ctrl(HdL,HdR,qin,qdotout,armElongL,armElongR,torsoElong);
}