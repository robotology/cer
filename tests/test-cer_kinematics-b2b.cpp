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
#include <sstream>
#include <deque>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

#include <cer_kinematics/arm.h>
#include <cer_kinematics_alt/Solver.h>

#define R2D     (180.0/M_PI)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace cer::kinematics;
using namespace cer::kinematics_alt;


/****************************************************************/
struct Input
{
    Vector xd;
    Vector ud;
    Matrix Hd;
    double torso_heave;
    double lower_arm_heave;
    Input() : xd(3,0.0), ud(3,0.0), Hd(eye(4,4)),
              torso_heave(0.0), lower_arm_heave(0.0) { }
    void calcHd()
    {
        double n=norm(ud);
        Vector ud_=(1.0/n)*ud;
        ud_.push_back(n);
        Hd=axis2dcm(ud_);
        Hd(0,3)=xd[0];
        Hd(1,3)=xd[1];
        Hd(2,3)=xd[2];
    }
};


/****************************************************************/
int main()
{
    // define inputs
    deque<Input> input;
    Input in;
    
    // input #0
    in.torso_heave=0.1;
    in.lower_arm_heave=0.01;
    in.xd[0]=0.2; in.xd[1]=-0.4;     in.xd[2]=-0.1;
    in.ud[0]=0.0; in.ud[1]=M_PI/2.0; in.ud[2]=0.0;
    in.calcHd();
    input.push_back(in);

    // input #1
    in.torso_heave=0.1;
    in.lower_arm_heave=0.01;
    in.xd[0]=0.2; in.xd[1]=-0.6;     in.xd[2]=-0.1;
    in.ud[0]=0.0; in.ud[1]=M_PI/2.0; in.ud[2]=0.0;
    in.calcHd();
    input.push_back(in);

    // input #2
    in.torso_heave=0.1;
    in.lower_arm_heave=0.0;
    in.xd[0]=0.4; in.xd[1]=-0.2;     in.xd[2]=0.0;
    in.ud[0]=0.0; in.ud[1]=M_PI/2.0; in.ud[2]=0.0;
    in.calcHd();
    input.push_back(in);

    // declare solvers
    ArmParameters armParams("left");

    ArmSolver solver_0;
    solver_0.setArmParameters(armParams);
    solver_0.setVerbosity(1);

    SolverParameters p=solver_0.getSolverParameters();
    p.setMode("full_pose");
    p.warm_start=true;
    solver_0.setSolverParameters(p);

    LeftSideSolver solver_1;

    // compute outputs and verify results
    for (size_t i=0; i<input.size(); i++)
    {
        yInfo()<<"INPUT #"<<i<<" --------------------------------------------";

        Vector q_0(12,0.0);
        SolverParameters p=solver_0.getSolverParameters();
        p.torso_heave=input[i].torso_heave;
        p.lower_arm_heave=input[i].lower_arm_heave;
        solver_0.setSolverParameters(p);
        solver_0.setInitialGuess(q_0);
        solver_0.ikin(input[i].Hd,q_0);

        Vector q_1(12,0.0);
        double t0=Time::now();
        solver_1.ikin(input[i].Hd,q_1,input[i].lower_arm_heave,input[i].torso_heave);
        double t1=Time::now();

        Matrix H_1;
        solver_1.fkin(q_1,H_1);

        Vector x_1=H_1.getCol(3).subVector(0,2);
        Vector u_1=dcm2axis(H_1);
        u_1*=u_1[3]; u_1.pop_back();

        ostringstream jconstr;
        for (size_t j=0; j<3; j++)
            jconstr<<(((q_1[j]>=armParams.torso.l_min)&&(q_1[j]<=armParams.torso.l_max))?"ok":"fail")<<" ";

        iKinChain &c=*armParams.upper_arm.asChain();
        for (size_t j=0; j<armParams.upper_arm.getN(); j++)
            jconstr<<(((q_1[3+j]>=R2D*c[j].getMin())&&(q_1[3+j]<=R2D*c[j].getMax()))?"ok":"fail")<<" ";

        for (size_t j=3+armParams.upper_arm.getN(); j<q_1.length(); j++)
            jconstr<<(((q_1[j]>=armParams.lower_arm.l_min)&&(q_1[j]<=armParams.lower_arm.l_max))?"ok":"fail")<<" ";

        Matrix H_tmp0;
        solver_0.fkin(q_1,H_tmp0,0);
        double h1=H_tmp0(2,3);
        double alpha1=R2D*acos(H_tmp0(2,2));

        Matrix H_tmp1;
        solver_0.fkin(q_1,H_tmp1,8);
        solver_0.fkin(q_1,H_tmp0,9);
        H_tmp0=SE3inv(H_tmp1)*H_tmp0;
        double h2=H_tmp0(2,3);
        double alpha2=R2D*acos(H_tmp0(2,2));

        yInfo()<<"           q_1 [*] = ("<<q_1.toString(5,5).c_str()<<")";
        yInfo()<<"joints-constraints = ("<<jconstr.str()<<")";
        yInfo()<<"         e_x_1 [m] = "<<norm(input[i].xd-x_1);
        yInfo()<<"       e_u_1 [rad] = "<<norm(input[i].ud-u_1);
        yInfo()<<"        e_h1_1 [m] = "<<fabs(input[i].torso_heave-h1);
        yInfo()<<"    alpha1_1 [deg] = "<<alpha1<<" ("<<(alpha1<=armParams.torso.alpha_max?"ok":"fail")<<")";
        yInfo()<<"        e_h2_1 [m] = "<<fabs(input[i].lower_arm_heave-h2);
        yInfo()<<"    alpha1_2 [deg] = "<<alpha2<<" ("<<(alpha2<=armParams.lower_arm.alpha_max?"ok":"fail")<<")";
        yInfo()<<"         dt_1 [ms] = "<<1000.0*(t1-t0);
        yInfo()<<"";
        yInfo()<<"";
    }

    return 0;
}

