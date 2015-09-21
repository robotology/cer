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

#include <yarp/os/Log.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#define DELTA_RHO           1e-6

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace tripod;


/****************************************************************/
struct TripodParametersExtended : public TripodParameters
{
    double cos_alpha_max;
    deque<Vector> s;
    Vector z;

    /****************************************************************/
    TripodParametersExtended(const TripodParameters &parameters) :
                             TripodParameters(parameters)
    {
        cos_alpha_max=cos(CTRL_DEG2RAD*alpha_max);

        z.resize(3,0.0);
        z[2]=1.0;

        Vector v(3,0.0);
        double theta=0.0;
        for (int i=0; i<3; i++)
        {            
            v[0]=r*cos(theta);
            v[1]=r*sin(theta);
            s.push_back(v);

            theta+=CTRL_DEG2RAD*120.0;
        }
    }
};


/****************************************************************/
struct TripodState
{
    Vector n,u,p;
    Matrix T;

    /****************************************************************/
    TripodState():n(3,0.0),u(3,0.0),p(3,0.0),T(eye(4,4)) { }
};


/****************************************************************/
class TripodNLP : public Ipopt::TNLP
{
protected:
    TripodParametersExtended &params;

    Vector ud;
    double zd;
    Vector rho0,rho;
    double drho;

public:
    /****************************************************************/
    TripodNLP(const TripodParametersExtended &p) : params(p)
    {
        zd=(params.l_max+params.l_min)/2.0;
        ud.resize(3,0.0);

        rho0.resize(3,zd);
        rho=rho0;
        drho=DELTA_RHO;        
    }

    /****************************************************************/
    TripodState fkin(const Vector &x) const
    {
        return fkin((Ipopt::Number*)x.data());
    }

    /****************************************************************/
    TripodState fkin(const Ipopt::Number *x) const
    {
        TripodState d;
        if ((x[0]==x[1]) && (x[1]==x[2]))
        {
            d.n=params.z;
            d.u=0.0;
            d.p[0]=d.p[1]=0.0;
            d.p[2]=x[0];
        }
        else
        {
            Vector v1=params.s[0]+x[0]*params.z;
            Vector v2=params.s[1]+x[1]*params.z;
            Vector v3=params.s[2]+x[2]*params.z;
            d.n=cross(v2-v1,v3-v1);
            d.n/=norm(d.n);

            double q33=sqrt(27.0)*params.r/sqrt(12.0*(x[2]*x[2]-(x[0]+x[1])*x[2]+
                                                x[1]*x[1]-x[0]*x[1]+x[0]*x[0]+
                                                (27.0/12.0)*params.r*params.r));
            double sin_theta=sqrt(1.0-q33*q33);
            d.u[0]=-d.n[1]/sin_theta;
            d.u[1]=d.n[0]/sin_theta;
            d.u[2]=0.0;
            double tmp=(1.0-q33);
            double q11=tmp*d.u[0]*d.u[0]+q33;
            double q22=tmp*d.u[1]*d.u[1]+q33;
            double q21=tmp*d.u[0]*d.u[1];
            double q31=-sin_theta*d.u[1];
            double q32=sin_theta*d.u[0];
            double m1=params.r/q33*(-0.5*q11+1.5*q22);
            d.p[0]=params.r-m1*q11;
            d.p[1]=-m1*q21;
            d.p[2]=x[0]-m1*q31;
            d.u*=acos(q33);

            // transformation matrix
            d.T(0,0)=q11; d.T(0,1)=q21; d.T(0,2)=-q31; d.T(0,3)=d.p[0];
            d.T(1,0)=q21; d.T(1,1)=q22; d.T(1,2)=-q32; d.T(1,3)=d.p[1];
            d.T(2,0)=q31; d.T(2,1)=q32; d.T(2,2)=q33;  d.T(2,3)=d.p[2];
        }

        return d;
    }

    /****************************************************************/
    void set_rho0(const Vector &rho0)
    {
        size_t len=std::min(this->rho0.length(),rho0.length());
        for (size_t i=0; i<len; i++)
            this->rho0[i]=std::max(params.l_min,std::min(params.l_max,rho0[0]));
    }

    /****************************************************************/
    void set_zd(const double zd)
    {
        this->zd=zd;
    }

    /****************************************************************/
    void set_ud(const Vector &ud)
    {
        size_t len=std::min(this->ud.length(),ud.length());
        for (size_t i=0; i<len; i++)
            this->ud[i]=ud[i];
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
            x_l[i]=param.l_min;
            x_u[i]=params.l_max;
        }

        g_l[0]=g_u[0]=0.0;

        g_l[1]=params.cos_alpha_max;
        g_u[1]=1.0;

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

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        TripodState d=fkin(x);
        obj_value=norm2(ud-d.u);

        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        TripodState d=fkin(x);

        Ipopt::Number x_dx[3];
        TripodState d_fw,d_bw;
        Vector tmp=ud-d.u;

        x_dx[0]=x[0]+drho;
        x_dx[1]=x[1];
        x_dx[2]=x[2];
        d_fw=fkin(x_dx);
        x_dx[0]=x[0]-drho;
        d_bw=fkin(x_dx);
        grad_f[0]=-dot(tmp,d_fw.u-d_bw.u)/drho;

        x_dx[0]=x[0];
        x_dx[1]=x[1]+drho;
        x_dx[2]=x[2];
        d_fw=fkin(x_dx);
        x_dx[1]=x[1]-drho;
        d_bw=fkin(x_dx);
        grad_f[1]=-dot(tmp,d_fw.u-d_bw.u)/drho;

        x_dx[0]=x[0];
        x_dx[1]=x[1];
        x_dx[2]=x[2]+drho;
        d_fw=fkin(x_dx);
        x_dx[2]=x[2]-drho;
        d_bw=fkin(x_dx);
        grad_f[2]=-dot(tmp,d_fw.u-d_bw.u)/drho;

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        TripodState d=fkin(x);

        Ipopt::Number tmp=(zd-d.p[2]);
        g[0]=tmp*tmp;
        g[1]=d.n[2];

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
            TripodState d=fkin(x);

            Ipopt::Number x_dx[3];
            TripodState d_fw,d_bw;
            double tmp=zd-d.p[2];

            x_dx[0]=x[0]+drho;
            x_dx[1]=x[1];
            x_dx[2]=x[2];
            d_fw=fkin(x_dx);
            x_dx[0]=x[0]-drho;
            d_bw=fkin(x_dx);
            values[0]=-tmp*(d_fw.p[2]-d_bw.p[2])/drho;
            values[3]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);

            x_dx[0]=x[0];
            x_dx[1]=x[1]+drho;
            x_dx[2]=x[2];
            d_fw=fkin(x_dx);
            x_dx[1]=x[1]-drho;
            d_bw=fkin(x_dx);
            values[1]=-tmp*(d_fw.p[2]-d_bw.p[2])/drho;
            values[4]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);

            x_dx[0]=x[0];
            x_dx[1]=x[1];
            x_dx[2]=x[2]+drho;
            d_fw=fkin(x_dx);
            x_dx[2]=x[2]-drho;
            d_bw=fkin(x_dx);
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


/****************************************************************/
TripodSolver::TripodSolver(const TripodParameters &params,
                           const int verb) :
                           parameters(params), verbosity(verb)
{
}


/****************************************************************/
bool TripodSolver::setInitialGuess(const Vector lll0)
{
    if (lll0.length()<3)
    {
        yError("mis-sized elongation vector!");
        return false;
    }

    this->lll0=lll0.subVector(0,2);
    return true;
}


/****************************************************************/
bool TripodSolver::fkin(const Vector &lll, Vector &p, Vector &u) const
{
    if (lll.length()<3)
    {
        yError("mis-sized elongation vector!");
        return false;
    }

    Ipopt::SmartPtr<TripodNLP> nlp=new TripodNLP(parameters);
    TripodState d=nlp->fkin(lll);
    p=d.p;
    u.d.u;

    return true;
}


/****************************************************************/
bool TripodSolver::fkin(const Vector &lll, Vector &hpr) const
{
    if (lll.length()<3)
    {
        yError("mis-sized elongation vector!");
        return false;
    }

    Ipopt::SmartPtr<TripodNLP> nlp=new TripodNLP(parameters);
    TripodState d=nlp->fkin(lll);
    
    Vector rpy=dcm2rpy(d.T);
    hpr.resize(3);
    hpr[0]=d.p[2];
    hpr[1]=rpy[1];
    hpr[2]=rpy[0];

    return true;
}


/****************************************************************/
bool TripodSolver::ikin(const double zd, const Vector &ud,
                        Vector &lll) const
{
    if (ud.length()<3)
    {
        yError("mis-sized orientation vector!");
        return false;
    }

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication; 
    app->Options()->SetNumericValue("tol",1e-6);
    app->Options()->SetNumericValue("acceptable_tol",1e-6);
    app->Options()->SetIntegerValue("acceptable_iter",10);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetIntegerValue("max_iter",200);
    app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("derivative_test","none");
    app->Options()->SetIntegerValue("print_level",0);
    app->Initialize();

    Ipopt::SmartPtr<TripodNLP> nlp=new TripodNLP(parameters);

    nlp->set_rho0(lll0);
    nlp->set_zd(zd);
    nlp->set_ud(ud);

    double t0=Time::now();
    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
    double t1=Time::now();

    lll=nlp->get_result();
    if (verbosity>0)
    {
        TripodState d=nlp->fkin(lll);
        yInfo(" *** Tripod Solver: lll0 = (%s) [m]",lll0.toString(15,15).c_str());
        yInfo(" *** Tripod Solver: zd = %g [m]",zd);
        yInfo(" *** Tripod Solver: ud = (%s) [rad]",ud.toString(15,15).c_str());
        yInfo(" *** Tripod Solver: lll = (%s) [m]",lll.toString(15,15).c_str());
        yInfo(" *** Tripod Solver: u = (%s) [rad]",d.u.toString(15,15).c_str());
        yInfo(" *** Tripod Solver: p = (%s) [m]",d.p.toString(15,15).c_str());
        yInfo(" *** Tripod Solver: alpha = %g [deg]",CTRL_RAD2DEG*acos(d.n[2]));
        yInfo(" *** Tripod Solver: solving time = %g [ms]",1000.0*(t1-t0));
    }
    
    return true;
}


/****************************************************************/
bool TripodSolver::ikin(const Vector &hpr, Vector &lll) const
{
    if (hpr.length()<3)
    {
        yError("mis-sized orientation vector!");
        return false;
    }

    Vector rpy(3,0.0);
    rpy[0]=CTRL_DEG2RAD*hpr[2];
    rpy[1]=CTRL_DEG2RAD*hpr[1];

    Vector ud=dcm2axis(rpy2dcm(rpy));
    ud=ud[3]*ud;
    ud.pop_back();

    return ikin(hpr[0],ud,lll);
}

