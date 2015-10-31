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
#include <cmath>
#include <limits>
#include <algorithm>

#include <yarp/os/Log.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#include <cer_kinematics/arm.h>
#include <cer_kinematics/private/helpers.h>

// COMMON PART -- begin
#define DELTA_RHO       1e-6

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace cer_kinematics;
// COMMON PART -- end

namespace cer_kinematics {
    #include <cer_kinematics/private/arm_common.h>
    #include <cer_kinematics/private/arm_full_noheave.h>
    #include <cer_kinematics/private/arm_full_heave.h>
    #include <cer_kinematics/private/arm_full_notorso.h>
    #include <cer_kinematics/private/arm_xyz_noheave.h>
    #include <cer_kinematics/private/arm_xyz_heave.h>
    #include <cer_kinematics/private/arm_xyz_notorso.h>
}


/****************************************************************/
ArmSolver::ArmSolver(const ArmParameters &armParams,
                     const SolverParameters &slvParams,
                     const int verb) :
                     Solver(verb),
                     armParameters(armParams),
                     slvParameters(slvParams)
{
}


/****************************************************************/
bool ArmSolver::setInitialGuess(const Vector &q0)
{
    size_t L=3+armParameters.upper_arm.getDOF()+3;
    if (q0.length()<L)
    {
        yError("mis-sized DOFs vector!");
        return false;
    }

    this->q0=q0.subVector(0,L-1);
    return true;
}


/****************************************************************/
bool ArmSolver::fkin(const Vector &q, Matrix &H, const int frame)
{
    size_t L=3+armParameters.upper_arm.getDOF()+3;
    if (q.length()<L)
    {
        yError("mis-sized DOFs vector!");
        return false;
    }

    Ipopt::SmartPtr<ArmFullNoHeaveNLP_ForwardDiff> nlp=new ArmFullNoHeaveNLP_ForwardDiff(*this);
    H=nlp->fkin(q,frame);

    return true;
}


/****************************************************************/
bool ArmSolver::ikin(const Matrix &Hd, Vector &q, int *exit_code)
{
    if ((Hd.rows()!=4) || (Hd.cols()!=4))
    {
        yError("mis-sized desired end-effector frame!");
        return false;
    }

    int print_level=std::max(verbosity-5,0);

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",slvParameters.tol);
    app->Options()->SetNumericValue("constr_viol_tol",slvParameters.constr_tol);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue("mu_strategy","monotone");
    app->Options()->SetIntegerValue("max_iter",500);
    app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    app->Options()->SetNumericValue("nlp_scaling_max_gradient",1.0);
    app->Options()->SetNumericValue("nlp_scaling_min_value",1e-6);
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("warm_start_init_point","no");
    app->Options()->SetStringValue("fixed_variable_treatment","make_parameter");
    app->Options()->SetStringValue("derivative_test",print_level>=4?"first-order":"none");
    app->Options()->SetIntegerValue("print_level",print_level);
    app->Initialize();

    Ipopt::SmartPtr<ArmCommonNLP> nlp;
    if (slvParameters.full_pose)
    {
        switch (slvParameters.configuration)
        {
        case configuration::no_torso:
            if (slvParameters.enable_central_difference)
                nlp=new ArmFullNoTorsoNLP_CentralDiff(*this); 
            else
                nlp=new ArmFullNoTorsoNLP_ForwardDiff(*this); 
            break;
        case configuration::heave:
            if (slvParameters.enable_central_difference)
                nlp=new ArmFullHeaveNLP_CentralDiff(*this); 
            else
                nlp=new ArmFullHeaveNLP_ForwardDiff(*this); 
            break;
        default:
            if (slvParameters.enable_central_difference)
                nlp=new ArmFullNoHeaveNLP_CentralDiff(*this); 
            else
                nlp=new ArmFullNoHeaveNLP_ForwardDiff(*this); 
        }
    }
    else
    {
        switch (slvParameters.configuration)
        {
        case configuration::no_torso:
            if (slvParameters.enable_central_difference)
                nlp=new ArmXyzNoTorsoNLP_CentralDiff(*this); 
            else
                nlp=new ArmXyzNoTorsoNLP_ForwardDiff(*this); 
            break;
        case configuration::heave:
            if (slvParameters.enable_central_difference)
                nlp=new ArmXyzHeaveNLP_CentralDiff(*this); 
            else
                nlp=new ArmXyzHeaveNLP_ForwardDiff(*this);
            break;
        default:
            if (slvParameters.enable_central_difference)
                nlp=new ArmXyzNoHeaveNLP_CentralDiff(*this); 
            else
                nlp=new ArmXyzNoHeaveNLP_ForwardDiff(*this); 
        }
    }

    nlp->set_q0(q0);
    nlp->set_multipliers(zL,zU);
    nlp->set_target(Hd);

    double t0=Time::now();
    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
    double t1=Time::now();

    q=nlp->get_result();
    nlp->get_multipliers(zL,zU);
    if (exit_code!=NULL)
        *exit_code=status;

    if (verbosity>0)
    {
        Vector xd=Hd.getCol(3).subVector(0,2);
        Vector ud=dcm2axis(Hd);
        ud*=ud[3];
        ud.pop_back();

        Matrix H=nlp->fkin(q);
        TripodState d1=nlp->tripod_fkin(1,q);
        TripodState d2=nlp->tripod_fkin(2,q);
        Vector x=H.getCol(3).subVector(0,2);
        Vector u=dcm2axis(H);
        u*=u[3];
        u.pop_back();

        yInfo(" *** Arm Solver ******************************");
        yInfo(" *** Arm Solver:          arm = %s",armParameters.upper_arm.getType().c_str());
        yInfo(" *** Arm Solver:         mode = %s",nlp->get_mode().c_str());
        yInfo(" *** Arm Solver:          tol = %g",slvParameters.tol);
        yInfo(" *** Arm Solver:   constr_tol = %g",slvParameters.constr_tol);
        yInfo(" *** Arm Solver:       q0 [*] = (%s)",q0.toString(4,4).c_str());
        yInfo(" *** Arm Solver:      zd1 [m] = %g",slvParameters.torso_heave);
        yInfo(" *** Arm Solver:      zd2 [m] = %g",slvParameters.lower_arm_heave);
        yInfo(" *** Arm Solver:       xd [m] = (%s)",xd.toString(4,4).c_str());
        yInfo(" *** Arm Solver:     ud [rad] = (%s)",ud.toString(4,4).c_str());
        yInfo(" *** Arm Solver:        q [*] = (%s)",q.toString(4,4).c_str());
        yInfo(" *** Arm Solver:      e_x [m] = %g",norm(xd-x));
        yInfo(" *** Arm Solver:    e_u [rad] = %g",norm(ud-u));
        yInfo(" *** Arm Solver:     e_z1 [m] = %g",fabs(slvParameters.torso_heave-d1.p[2]));
        yInfo(" *** Arm Solver: alpha1 [deg] = %g",CTRL_RAD2DEG*acos(d1.n[2]));
        yInfo(" *** Arm Solver:     e_z2 [m] = %g",fabs(slvParameters.lower_arm_heave-d2.p[2]));
        yInfo(" *** Arm Solver: alpha2 [deg] = %g",CTRL_RAD2DEG*acos(d2.n[2]));
        yInfo(" *** Arm Solver:      dt [ms] = %g",1000.0*(t1-t0));
        yInfo(" *** Arm Solver ******************************");
    }

    switch (status)
    {
        case Ipopt::Solve_Succeeded:
        case Ipopt::Solved_To_Acceptable_Level:
        case Ipopt::Feasible_Point_Found:
        {
            if (verbosity>0)
                yInfo(" *** Arm Solver: IpOpt return code %d",status);
            return true;
        } 

        default:
        {
            if (verbosity>0)
                yWarning(" *** Arm Solver: IpOpt return code %d",status);
            return false;
        } 
    }
}


/****************************************************************/
ArmCOM::ArmCOM(ArmSolver &solver_, const double external_weight,
               const double floor_z) : solver(solver_)
{
    string arm_type=solver.getArmParameters().upper_arm.getType();

    // CoM absolute positions
    // mobilebase_lowertorso
    Vector tmp(4,1.0);
    tmp[0]=-0.048;
    tmp[1]=0.0;
    tmp[2]=-0.415;
    relComs.push_back(tmp);
    
    // head
    tmp[0]=-0.028;
    tmp[1]=0.0;
    tmp[2]=0.521;
    relComs.push_back(tmp);

    // l0
    tmp[0]=-0.014;
    tmp[1]=0.0;
    tmp[2]=0.18;
    relComs.push_back(tmp);
   
    // l3
    tmp[0]=-0.084;
    tmp[1]=0.191*((arm_type=="left")?1.0:-1.0);
    tmp[2]=0.215;
    relComs.push_back(tmp);
   
    // l5
    tmp[0]=-0.084;
    tmp[1]=0.191*((arm_type=="left")?1.0:-1.0);
    tmp[2]=-0.089;
    relComs.push_back(tmp);
    
    // hand
    tmp[0]=0.0;
    tmp[1]=0.0;
    tmp[2]=0.0;
    relComs.push_back(tmp);

    // compute CoMs relative positions wrt q0
    Vector q0(12,0.0);

    Matrix frame;
    solver.fkin(q0,frame,3+0);
    relComs[1]=SE3inv(frame)*relComs[1];
    relComs[2]=SE3inv(frame)*relComs[2];

    solver.fkin(q0,frame,3+3);        
    relComs[3]=SE3inv(frame)*relComs[3];

    solver.fkin(q0,frame,3+5);
    relComs[4]=SE3inv(frame)*relComs[4];

    // same order as per relComs
    weights.push_back(25.0);
    weights.push_back(2.5);
    weights.push_back(7.5);
    weights.push_back(1.5);
    weights.push_back(1.0);
    weights.push_back(0.6+external_weight);
    weight_tot=dot(weights,Vector(weights.length(),1.0));

    // support polygon
    Vector c1(4,1.0);
    c1[0]=0.078; c1[1]=0.09;
    Vector c2(4,1.0);
    c2[0]=-0.244; c2[1]=0.0;
    Vector c3(4,1.0);
    c3[0]=c1[0]; c3[1]=-c1[1];
    Vector w1(4,1.0);
    w1[0]=-0.074; w1[1]=0.17;
    Vector w2(4,1.0);
    w2[0]=w1[0]; w2[1]=-w1[1];

    c1[2]=c2[2]=c3[2]=w1[2]=w2[2]=floor_z;

    // caster polygon reduction
    double caster_reduction=0.023;
    double r,theta;

    r=norm(c1.subVector(0,1))-caster_reduction;
    theta=atan2(c1[1],c1[0]);
    c1[0]=r*cos(theta);
    c1[1]=r*sin(theta);

    r=norm(c2.subVector(0,1))-caster_reduction;
    theta=atan2(c2[1],c2[0]);
    c2[0]=r*cos(theta);
    c2[1]=r*sin(theta);

    r=norm(c3.subVector(0,1))-caster_reduction;
    theta=atan2(c3[1],c3[0]);
    c3[0]=r*cos(theta);
    c3[1]=r*sin(theta);

    supPolygon.push_back(c1);
    supPolygon.push_back(c3);
    supPolygon.push_back(w2);
    supPolygon.push_back(c2);
    supPolygon.push_back(w1);
}


/****************************************************************/
bool ArmCOM::getCOMs(const Vector &q, deque<Vector> &coms) const
{
    int nJoints=3+solver.getArmParameters().upper_arm.getDOF()+3;
    if (q.length()!=nJoints)
    {
        yError(" *** Arm COM: input joints are fewer than %d!",nJoints);
        return false;
    }

    coms.clear(); 
    coms.push_back(relComs[0]);

    Matrix frame;
    solver.fkin(q,frame,3+0);
    coms.push_back(frame*relComs[1]);
    coms.push_back(frame*relComs[2]);

    solver.fkin(q,frame,3+3);
    coms.push_back(frame*relComs[3]);

    solver.fkin(q,frame,3+5);
    coms.push_back(frame*relComs[4]);

    solver.fkin(q,frame);
    coms.push_back(frame*relComs[5]);

    Vector com_tot(4,0.0);
    for (size_t i=0; i<coms.size(); i++)
        com_tot+=weights[i]*coms[i];
    
    com_tot/=weight_tot;
    com_tot[3]=1.0; // reinforce homogeneity 

    coms.push_back(com_tot);
    return true;
}


/****************************************************************/
bool ArmCOM::getSupportMargin(const Vector &com, double &margin) const
{
    if (com.length()!=4)
    {
        yError(" *** Arm COM: input CoM is not homogeneous!");
        return false;
    }

    margin=std::numeric_limits<double>::max();
    Vector neg_mrg;

    for (size_t i=0; i<supPolygon.size(); i++)
    {
        const Vector &p0=supPolygon[i];
        const Vector &p1=supPolygon[(i+1)%supPolygon.size()];

        Vector d=p0-p1;
        Vector rot(4,0.0);
        rot[2]=1.0;
        rot[3]=(d[0]!=0.0)?atan2(d[1],d[0]):M_PI/2.0;
        Matrix R=SE3inv(axis2dcm(rot));

        Vector p0_rot=R*p0;
        if (p0_rot[1]<0.0)
        {
            rot[3]+=M_PI;
            R=SE3inv(axis2dcm(rot));
            p0_rot=R*p0;
        }

        Vector com_rot=R*com;
        double mrg=p0_rot[1]-com_rot[1];
        margin=std::min(mrg,margin);

        if (mrg<0.0)
            neg_mrg.push_back(mrg);
    }

    if (margin<0.0)
    {
        margin=-std::numeric_limits<double>::max();
        for (size_t i=0; i<neg_mrg.length(); i++)
            margin=std::max(neg_mrg[i],margin);

        Vector com_xy=com.subVector(0,1);
        for (size_t i=0; i<supPolygon.size(); i++)
            margin=std::max(-norm(com_xy-supPolygon[i].subVector(0,1)),margin);
    }

    return true;
}

