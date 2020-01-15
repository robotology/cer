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

#include <algorithm>
#include <cmath>
#include <deque>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#include <cer_kinematics/tripod.h>
#include <cer_kinematics/private/helpers.h>

#define DELTA_RHO           1e-6

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace cer::kinematics;

namespace cer {
namespace kinematics {

/****************************************************************/
class TripodNLP : public TripodNLPHelper,
                  public Ipopt::TNLP
{
protected:
    TripodSolver &slv;
    const TripodParametersExtended params;
        
    Vector rho0,rho;
    double drho;

    double zd;
    Matrix Rd;
    Vector e;

    TripodState d,din;
    Vector lll;

    /****************************************************************/
    TripodState fkin(const Ipopt::Number *x, TripodState *internal=NULL)
    {
        return fkinHelper(x,params,internal);
    }

public:
    /****************************************************************/
    TripodNLP(TripodSolver &slv_) : slv(slv_), params(slv_.parameters)
    {
        zd=(params.l_max+params.l_min)/2.0;
        Rd=eye(4,4);

        rho0.resize(3,zd);
        rho=rho0;
        drho=DELTA_RHO;
        lll=rho0;
    }

    /****************************************************************/
    TripodState fkin(const Vector &x, TripodState *internal=NULL)
    {
        return fkin((Ipopt::Number*)x.data(),internal);
    }

    /****************************************************************/
    void set_rho0(const Vector &rho0)
    {
        size_t len=std::min(this->rho0.length(),rho0.length());
        for (size_t i=0; i<len; i++)
            this->rho0[i]=std::max(params.l_min,std::min(params.l_max,rho0[i]));
    }

    /****************************************************************/
    void set_zd(const double zd)
    {
        this->zd=zd;
    }

    /****************************************************************/
    void set_ud(const Vector &ud)
    {
        yAssert(ud.length()>=4);
        Rd=axis2dcm(ud);
    }

    /****************************************************************/
    Vector get_result() const
    {
        return rho;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=3;
        m=2;
        nnz_jac_g=6;
        nnz_h_lag=0;
        index_style=TNLP::C_STYLE;

        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        for (Ipopt::Index i=0; i<n; i++)
        {
            x_l[i]=params.l_min;
            x_u[i]=params.l_max;
        }

        g_l[0]=g_u[0]=0.0;
        g_l[1]=params.cos_alpha_max; g_u[1]=1.0;

        return true;
    }

    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
    {
        for (Ipopt::Index i=0; i<n; i++)
            x[i]=rho0[i];

        return true;
    }

    /************************************************************************/
    virtual void computeQuantities(const Ipopt::Number *x, const bool new_x)
    {        
        if (new_x)
        {
            for (size_t i=0; i<this->lll.length(); i++)
                this->lll[i]=x[i];

            d=fkin(x,&din);
            e=dcm2axis(Rd*d.T.transposed());
            e*=e[3]; e.pop_back();
        }
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        computeQuantities(x,new_x);
        obj_value=norm2(e);

        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x,new_x);

        Ipopt::Number x_dx[3];
        TripodState d_fw,d_bw;
        Vector e_fw,e_bw;

        x_dx[0]=x[0]+drho;
        x_dx[1]=x[1];
        x_dx[2]=x[2];
        d_fw=fkin(x_dx);
        e_fw=dcm2axis(Rd*d_fw.T.transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        x_dx[0]=x[0]-drho;
        d_bw=fkin(x_dx);
        e_bw=dcm2axis(Rd*d_bw.T.transposed()); e_bw*=e_bw[3]; e_bw.pop_back();        
        grad_f[0]=dot(e,e_fw-e_bw)/drho;

        x_dx[0]=x[0];
        x_dx[1]=x[1]+drho;
        x_dx[2]=x[2];
        d_fw=fkin(x_dx);
        e_fw=dcm2axis(Rd*d_fw.T.transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        x_dx[1]=x[1]-drho;
        d_bw=fkin(x_dx);
        e_bw=dcm2axis(Rd*d_bw.T.transposed()); e_bw*=e_bw[3]; e_bw.pop_back();        
        grad_f[1]=dot(e,e_fw-e_bw)/drho;

        x_dx[0]=x[0];
        x_dx[1]=x[1];
        x_dx[2]=x[2]+drho;
        d_fw=fkin(x_dx);
        e_fw=dcm2axis(Rd*d_fw.T.transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        x_dx[2]=x[2]-drho;
        d_bw=fkin(x_dx);
        e_bw=dcm2axis(Rd*d_bw.T.transposed()); e_bw*=e_bw[3]; e_bw.pop_back();        
        grad_f[2]=dot(e,e_fw-e_bw)/drho;

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        computeQuantities(x,new_x);

        Ipopt::Number tmp=zd-din.p[2];
        g[0]=tmp*tmp;
        g[1]=din.n[2];

        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        if (values==NULL)
        {
            Ipopt::Index idx=0;
            for (Ipopt::Index row=0; row<m; row++)
            {
                for (Ipopt::Index col=0; col<n; col++)
                {
                    iRow[idx]=row;
                    jCol[idx]=col;
                    idx++;
                }
            }
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[3];
            TripodState d_fw,d_bw;
            double tmp=zd-din.p[2];

            x_dx[0]=x[0]+drho;
            x_dx[1]=x[1];
            x_dx[2]=x[2];
            fkin(x_dx,&d_fw);
            x_dx[0]=x[0]-drho;
            fkin(x_dx,&d_bw);
            values[0]=-tmp*(d_fw.p[2]-d_bw.p[2])/drho;
            values[3]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);

            x_dx[0]=x[0];
            x_dx[1]=x[1]+drho;
            x_dx[2]=x[2];
            fkin(x_dx,&d_fw);
            x_dx[1]=x[1]-drho;
            fkin(x_dx,&d_bw);
            values[1]=-tmp*(d_fw.p[2]-d_bw.p[2])/drho;
            values[4]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);

            x_dx[0]=x[0];
            x_dx[1]=x[1];
            x_dx[2]=x[2]+drho;
            fkin(x_dx,&d_fw);
            x_dx[2]=x[2]-drho;
            fkin(x_dx,&d_bw);
            values[2]=-tmp*(d_fw.p[2]-d_bw.p[2])/drho;
            values[5]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
        }

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
    bool intermediate_callback(Ipopt::AlgorithmMode mode, Ipopt::Index iter,
                               Ipopt::Number obj_value, Ipopt::Number inf_pr,
                               Ipopt::Number inf_du, Ipopt::Number mu,
                               Ipopt::Number d_norm, Ipopt::Number regularization_size,
                               Ipopt::Number alpha_du, Ipopt::Number alpha_pr,
                               Ipopt::Index ls_trials, const Ipopt::IpoptData* ip_data,
                               Ipopt::IpoptCalculatedQuantities* ip_cq)
    {
        if (slv.callback!=NULL)
        {
            Matrix Hd=Rd; Hd(2,3)=zd;
            return slv.callback->exec(iter,Hd,lll,d.T);
        }
        else
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
        rho.resize(n);
        for (Ipopt::Index i=0; i<n; i++)
            rho[i]=x[i];
    }
};

}

}


/****************************************************************/
TripodSolver::TripodSolver(const TripodParameters &params,
                           const int verb) :
                           Solver(verb),
                           parameters(params)
{
}


/****************************************************************/
bool TripodSolver::setInitialGuess(const Vector &lll0)
{
    yAssert(lll0.length()>=3);
    this->lll0=lll0.subVector(0,2);
    return true;
}


/****************************************************************/
bool TripodSolver::fkin(const Vector &lll, Vector &p, Vector &u)
{
    yAssert(lll.length()>=3);
    Ipopt::SmartPtr<TripodNLP> nlp=new TripodNLP(*this);
    TripodState d=nlp->fkin(lll);
    p=d.p;
    u=d.u;

    return true;
}


/****************************************************************/
bool TripodSolver::fkin(const Vector &q, Matrix &H, const int frame)
{
    yAssert(q.length()>=3);
    Ipopt::SmartPtr<TripodNLP> nlp=new TripodNLP(*this);
    TripodState d=nlp->fkin(q);
    H=d.T;

    return true;
}


/****************************************************************/
bool TripodSolver::ikin(const double zd, const Vector &ud,
                        Vector &lll, int *exit_code)
{
    lock_guard<mutex> lg(makeThreadSafe);
    yAssert(ud.length()>=4);

    int print_level=std::max(verbosity-5,0);

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",1e-6);
    app->Options()->SetNumericValue("constr_viol_tol",1e-8);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetIntegerValue("max_iter",200);
    app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    app->Options()->SetNumericValue("nlp_scaling_max_gradient",1.0);
    app->Options()->SetNumericValue("nlp_scaling_min_value",1e-6);
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("derivative_test",print_level>=4?"first-order":"none");
    app->Options()->SetIntegerValue("print_level",print_level);
    app->Initialize();

    Ipopt::SmartPtr<TripodNLP> nlp=new TripodNLP(*this);    
    nlp->set_rho0(lll0);
    nlp->set_zd(zd);
    nlp->set_ud(ud);
    
    double t0=Time::now();
    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
    double t1=Time::now();    

    lll=nlp->get_result();
    if (exit_code!=NULL)
        *exit_code=status;

    if (verbosity>0)
    {
        TripodState din;
        TripodState d=nlp->fkin(lll,&din);

        Vector e_u=dcm2axis(axis2dcm(ud)*d.T.transposed());
        e_u*=e_u[3]; e_u.pop_back();

        yInfo()<<" *** Tripod Solver ******************************";
        yInfo()<<" *** Tripod Solver:    lll0 [m] = ("<<lll0.toString(4,4)<<")";
        yInfo()<<" *** Tripod Solver:      zd [m] ="<<zd;
        yInfo()<<" *** Tripod Solver:    ud [rad] = ("<<ud.toString(4,4)<<")";
        yInfo()<<" *** Tripod Solver:     lll [m] = ("<<lll.toString(4,4)<<")";
        yInfo()<<" *** Tripod Solver:     u [rad] = ("<<d.u.toString(4,4)<<")";
        yInfo()<<" *** Tripod Solver:       p [m] = ("<<d.p.toString(4,4)<<")";
        yInfo()<<" *** Tripod Solver:   e_u [rad] ="<<norm(e_u);
        yInfo()<<" *** Tripod Solver:     e_z [m] ="<<fabs(zd-din.p[2]);
        yInfo()<<" *** Tripod Solver: alpha [deg] ="<<CTRL_RAD2DEG*acos(din.n[2]);
        yInfo()<<" *** Tripod Solver:     dt [ms] ="<<1000.0*(t1-t0);
        yInfo()<<" *** Tripod Solver ******************************";
    }

    switch (status)
    {
        case Ipopt::Solve_Succeeded:
        case Ipopt::Solved_To_Acceptable_Level:
        case Ipopt::Feasible_Point_Found:
        {
            if (verbosity>0)
                yInfo()<<" *** Tripod Solver: IpOpt return code"<<status;
            return true;
        } 

        default:
        {
            if (verbosity>0)
                yWarning()<<" *** Tripod Solver: IpOpt return code"<<status;
            return false;
        } 
    }
}


/****************************************************************/
bool TripodSolver::ikin(const Matrix &Hd, Vector &q, int *exit_code)
{
    yAssert((Hd.rows()==4)&&(Hd.cols()==4));
    // get the heave irrespective of the frame
    Matrix Hd_=SE3inv(parameters.T0)*Hd;
    return ikin(Hd_(2,3),dcm2axis(Hd),q,exit_code);
}

