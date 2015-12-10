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

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <cer_kinematics/arm.h>
#include <cer_kinematics_alt/Solver.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace cer::kinematics;
using namespace cer::kinematics_alt;


/****************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.configure(argc,argv);

    string arm_type=rf.check("arm-type",Value("left")).asString().c_str();
    int verbosity=rf.check("verbosity",Value(1)).asInt();    

    ArmSolver solver_0;
    LeftSideSolver solver_1;

    SolverParameters p=solver_0.getSolverParameters();
    p.setMode("full_pose");

    solver_0.setArmParameters(ArmParameters(arm_type));
    solver_0.setSolverParameters(p);
    solver_0.setVerbosity(verbosity);

    Vector qin(12,0.0);

    /*
    qin(0)=-0.01;
    qin(1)=-0.02;
    qin(2)=-0.03;
    
    qin(3)=qin(4)=qin(5)=qin(6)=-5.0;
    qin(7)=10.0;
    qin(8)=-5.0;

    qin(9) =0.010;
    qin(10)=0.012;
    qin(11)=0.014;
    */

    yarp::sig::Matrix H_0;
    solver_0.fkin(qin,H_0);

    int frame=(arm_type=="left"?cer::kinematics_alt::LEFT_HAND:cer::kinematics_alt::RIGHT_HAND);
    yarp::sig::Matrix H_1;
    solver_1.fkin(qin,H_1,frame);

    yInfo()<<"H_0="<<H_0.toString(5,5).c_str();
    yInfo()<<"H_1="<<H_1.toString(5,5).c_str();
}

