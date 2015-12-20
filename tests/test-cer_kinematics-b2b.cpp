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
    int arm_frame=(arm_type=="left"?LEFT_HAND:RIGHT_HAND);

    ArmSolver solver_0;
    LeftSideSolver solver_1;

    solver_0.setArmParameters(ArmParameters(arm_type));
    solver_0.setVerbosity(1);

    // target definition: begin
    double torso_heave=0.1;
    double lower_arm_heave=0.01;
    Vector xd(3),ud(3);
    xd[0]=0.6;
    xd[1]=(arm_type=="left"?-0.1:0.1);
    xd[2]=-0.1;
    ud[0]=0.0;
    ud[1]=M_PI/2.0;
    ud[2]=0.0;

    double n=norm(ud);
    Vector ud_=(1.0/n)*ud;
    ud_.push_back(n);
    Matrix Hd=axis2dcm(ud_);
    Hd(0,3)=xd[0];
    Hd(1,3)=xd[1];
    Hd(2,3)=xd[2];
    // target definition: end

    SolverParameters p=solver_0.getSolverParameters();
    p.setMode("full_pose");
    p.torso_heave=torso_heave;
    p.lower_arm_heave=lower_arm_heave;

    solver_0.setSolverParameters(p);
    solver_0.setInitialGuess(Vector(12,0.0));

    Vector q_0;
    solver_0.ikin(Hd,q_0);

    Vector q_1(12,0.0);
    double t0=Time::now();
    solver_1.ikin(Hd,q_1,lower_arm_heave,torso_heave);
    double t1=Time::now();

    Matrix H_1;
    solver_1.fkin(q_1,H_1);

    Vector x_1=H_1.getCol(3).subVector(0,2);
    Vector u_1=dcm2axis(H_1);
    u_1*=u_1[3]; u_1.pop_back();

    yInfo()<<"------------------------------------------";
    yInfo()<<"    q_1 [*] = "<<q_1.toString(5,5).c_str();
    yInfo()<<"  e_x_1 [m] = "<<norm(xd-x_1);
    yInfo()<<"e_u_1 [rad] = "<<norm(ud-u_1);
    yInfo()<<"  dt_1 [ms] = "<<1000.0*(t1-t0);
}

