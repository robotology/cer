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
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#include <cer_kinematics/head.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace cer::kinematics;

namespace cer {
namespace kinematics {

/****************************************************************/
class HeadNLP : public Ipopt::TNLP
{
protected:
    HeadSolver &slv;
    HeadParameters &params;
    
    Matrix H0,Hxd;
    Matrix GeoJacobP,AnaJacobZ;
    Vector xd,q0,q;
    double mod,cosAng;
    double e_ang;

public:
    /****************************************************************/
    HeadNLP(HeadSolver &slv_) : slv(slv_), params(slv_.headParameters)
    {
        xd.resize(3,0.0);
        q0.resize(3+params.head.getDOF(),0.0);
        set_q0(q0);

        q.resize(2,0.0);
        e_ang=0.0;
    }

    /****************************************************************/
    void set_q0(const Vector &q0)
    {
        size_t len=std::min(this->q0.length(),q0.length());

        size_t l1=std::min((size_t)3,len);
        for (size_t i=0; i<l1; i++)
            this->q0[i]=std::max(params.torso.l_min,std::min(params.torso.l_max,q0[i]));
        
        iKinChain &chain=*params.head.asChain();
        size_t l2=std::min((size_t)chain.getDOF(),len-3);
        for (size_t i=0; i<l2; i++)
            this->q0[3+i]=std::max(chain[i].getMin(),std::min(chain[i].getMax(),CTRL_DEG2RAD*q0[3+i]));

        slv.fkin(this->q0,H0,2);
    }

    /****************************************************************/
    void set_xd(const Vector &xd)
    {
        size_t len=std::min(this->xd.length(),xd.length());
        for (size_t i=0; i<len; i++)
            this->xd[i]=xd[i];
    }

    /****************************************************************/
    Vector get_result(double &e_ang) const
    {
        e_ang=this->e_ang;
        return q;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=2;
        m=nnz_jac_g=nnz_h_lag=0;
        index_style=TNLP::C_STYLE;
        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        iKinChain &chain=*params.head.asChain();
        for (Ipopt::Index i=0; i<n; i++)
        {
            x_l[i]=chain[1+i].getMin();
            x_u[i]=chain[1+i].getMax();
        }

        return true;
    }

    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
    {
        x[0]=q0[q0.length()-2];
        x[1]=q0[q0.length()-1];
        return true;
    }

    /************************************************************************/
    virtual void computeQuantities(const Ipopt::Number *x, const bool new_x)
    {        
        if (new_x)
        {
            iKinChain &chain=*params.head.asChain();
            chain.setAng(1,x[0]);
            chain.setAng(2,x[1]);
            Hxd=H0*chain.getH();
            Hxd(0,3)-=xd[0];
            Hxd(1,3)-=xd[1];
            Hxd(2,3)-=xd[2];
            Hxd(3,3)=0.0;
            mod=norm(Hxd,3);
            cosAng=dot(Hxd,2,Hxd,3)/mod;

            GeoJacobP=chain.GeoJacobian();
            AnaJacobZ=chain.AnaJacobian(2);
        }
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        computeQuantities(x,new_x);
        obj_value=cosAng;
        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x,new_x);
        for (Ipopt::Index i=0; i<n; i++)
        {
            Ipopt::Index j=i+1;
            grad_f[i]=(dot(AnaJacobZ,j,Hxd,3)+dot(Hxd,2,GeoJacobP,j))/mod
                      -(cosAng*dot(Hxd,3,GeoJacobP,j))/(mod*mod);
        }
        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        return true;
    }

    /****************************************************************/
    bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number *lambda,
                bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow,
                Ipopt::Index *jCol, Ipopt::Number *values)
    {
        return true;
    }

    /****************************************************************/
    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                           const Ipopt::Number *x, const Ipopt::Number *z_L,
                           const Ipopt::Number *z_U, Ipopt::Index m,
                           const Ipopt::Number *g, const Ipopt::Number *lambda,
                           Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                           Ipopt::IpoptCalculatedQuantities *ip_cq)
    {
        computeQuantities(x,true);

        q.resize(2);
        q[0]=CTRL_RAD2DEG*x[0];
        q[1]=CTRL_RAD2DEG*x[1];

        e_ang=CTRL_RAD2DEG*(M_PI-acos(cosAng));
    }
};

}

}


/****************************************************************/
HeadSolver::HeadSolver(const HeadParameters &headParams,
                       const SolverParameters &slvParams,
                       const int verb) :
                       Solver(verb),
                       headParameters(headParams),
                       slvParameters(slvParams),
                       torso(headParams.torso,verb)
{
    q0.resize(3+headParameters.head.getDOF(),0.0);
    slvParameters.tol=1e-3;
    slvParameters.constr_tol=1e-3;
    slvParameters.max_iter=100;
    slvParameters.max_cpu_time=0.05;
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
    size_t L=3+headParameters.head.getDOF();
    if (q0.length()<L)
    {
        yError("mis-sized DOFs vector!");
        return false;
    }

    this->q0=q0.subVector(0,L-1);
    return true;
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

    torso.fkin(q.subVector(0,2),H);
    headParameters.head.setAng(CTRL_DEG2RAD*q.subVector(3,5));

    int frame_=(frame<0)?L-1:frame;
    if (frame_>=3)
        H*=headParameters.head.getH(std::min(frame_-3,(int)headParameters.head.getDOF()-1));

    return true;
}


/****************************************************************/
bool HeadSolver::ikin(const Vector &xd, Vector &q, int *exit_code)
{
    if (xd.length()!=3)
    {
        yError("mis-sized desired fixation point!");
        return false;
    }

    int print_level=std::max(verbosity-5,0);

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",slvParameters.tol);
    app->Options()->SetNumericValue("constr_viol_tol",slvParameters.constr_tol);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetIntegerValue("max_iter",slvParameters.max_iter);
    app->Options()->SetNumericValue("max_cpu_time",slvParameters.max_cpu_time);
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("derivative_test",print_level>=4?"first-order":"none");
    app->Options()->SetIntegerValue("print_level",print_level);
    app->Initialize();

    Ipopt::SmartPtr<HeadNLP> nlp=new HeadNLP(*this);
    nlp->set_q0(q0);
    nlp->set_xd(xd);

    double t0=Time::now();
    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
    double t1=Time::now();

    double e_ang;
    q=nlp->get_result(e_ang);
    if (exit_code!=NULL)
        *exit_code=status;

    if (verbosity>0)
    {
        yInfo(" *** Head Solver ******************************");
        yInfo(" *** Head Solver:             head = %s",headParameters.head.getType().c_str());
        yInfo(" *** Head Solver:          tol [*] = %g",slvParameters.tol);
        yInfo(" *** Head Solver:   constr_tol [*] = %g",slvParameters.constr_tol);
        yInfo(" *** Head Solver:     max_iter [#] = %d",slvParameters.max_iter);
        yInfo(" *** Head Solver: max_cpu_time [s] = %g",slvParameters.max_cpu_time);
        yInfo(" *** Head Solver:           q0 [*] = (%s)",q0.toString(4,4).c_str());
        yInfo(" *** Head Solver:           xd [m] = (%s)",xd.toString(4,4).c_str());
        yInfo(" *** Head Solver:          q [deg] = (%s)",q.toString(4,4).c_str());
        yInfo(" *** Head Solver:      e_ang [deg] = %g",e_ang);
        yInfo(" *** Head Solver:          dt [ms] = %g",1000.0*(t1-t0));
        yInfo(" *** Head Solver ******************************");
    }

    switch (status)
    {
        case Ipopt::Solve_Succeeded:
        case Ipopt::Solved_To_Acceptable_Level:
        case Ipopt::Feasible_Point_Found:
        {
            if (verbosity>0)
                yInfo(" *** Head Solver: IpOpt return code %d",status);
            return true;
        } 

        default:
        {
            if (verbosity>0)
                yWarning(" *** Head Solver: IpOpt return code %d",status);
            return false;
        } 
    }

    return true;
}


/****************************************************************/
bool HeadSolver::ikin(const Matrix &Hd, Vector &q, int *exit_code)
{
    if ((Hd.rows()!=4) || (Hd.cols()!=4))
    {
        yError("mis-sized desired end-effector frame!");
        return false;
    }

    return ikin(Hd.getCol(3).subVector(0,2),q,exit_code);
}

