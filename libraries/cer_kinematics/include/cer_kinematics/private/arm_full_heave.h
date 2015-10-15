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
#include <deque>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#define DELTA_RHO               1e-6
#define WEIGHT_POSTURAL_TORSO   0.01
#define WEIGHT_POSTURAL_ARM     0.0

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/****************************************************************/
struct TripodParameters
{
    double l,L_min,L_max,cos_alpha_M;
    deque<Vector> s;
    Vector z;

    /****************************************************************/
    TripodParameters(const double l_, const double L_min_,
                     const double L_max_, const double alpha_M)
    {
        l=l_;
        L_min=L_min_;
        L_max=L_max_;
        cos_alpha_M=cos(alpha_M);

        z.resize(3,0.0);
        z[2]=1.0;

        Vector v(3,0.0);
        double theta=0.0;
        for (int i=0; i<3; i++)
        {            
            v[0]=l*cos(theta);
            v[1]=l*sin(theta);
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
class Arm : public iKinLimb
{
public:
    /****************************************************************/
    Arm() : iKinLimb()
    {
        allocate("don't care");
    }

protected:
    /****************************************************************/
    void allocate(const string &type)
    {
        Matrix HN=eye(4,4);
        HN(2,3)=0.22;
        setHN(HN);

        // see #628
        pushLink(new iKinLink(-0.084, 0.312317,100.0*CTRL_DEG2RAD, 180.0*CTRL_DEG2RAD,-90.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   0.0,-0.159422, 90.0*CTRL_DEG2RAD,  90.0*CTRL_DEG2RAD,-20.0*CTRL_DEG2RAD,100.0*CTRL_DEG2RAD));
        pushLink(new iKinLink( 0.034,      0.0,-90.0*CTRL_DEG2RAD,-100.0*CTRL_DEG2RAD,-10.0*CTRL_DEG2RAD,100.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   0.0,   -0.251, 90.0*CTRL_DEG2RAD, -90.0*CTRL_DEG2RAD,-90.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   0.0,      0.0,-90.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD,  0.0*CTRL_DEG2RAD,140.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   0.0,   -0.071,180.0*CTRL_DEG2RAD, -90.0*CTRL_DEG2RAD,-90.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));

        // this is required since IpOpt initially relaxes constraints
        setAllConstraints(false);
    }
};


/****************************************************************/
class ChainNLP : public Ipopt::TNLP
{
protected:
    Arm arm;
    TripodParameters tripod1,tripod2;
    double drho;

    Matrix H0,HN,T0,TN,Rd;
    Vector x0,x;
    Vector xd,ud;

public:
    /****************************************************************/
    ChainNLP() : tripod1(0.09,-0.05,0.15,CTRL_DEG2RAD*30.0),
                 tripod2(0.018,0.0,0.14,CTRL_DEG2RAD*35.0)
    {
        drho=DELTA_RHO;

        H0=arm.getH0();
        HN=arm.getHN();

        Vector rot(4,0.0);
        rot[2]=1.0; rot[3]=M_PI;
        T0=axis2dcm(rot);

        double hand_ang=CTRL_DEG2RAD*20.0;
        double hand_dist=0.07;
        rot=0.0;
        rot[1]=1.0; rot[3]=-M_PI/2.0+hand_ang;
        TN=axis2dcm(rot);
        TN(0,3)=hand_dist*sin(hand_ang);
        TN(2,3)=hand_dist*cos(hand_ang);

        x0.resize(12,0.0);

        iKinChain *chain=arm.asChain();
        for (size_t i=0; i<arm.getDOF(); i++)
            x0[3+i]=0.5*((*chain)[i].getMin()+(*chain)[i].getMax());
        
        x=x0;

        xd.resize(3,0.0);
        ud.resize(3,0.0);
        set_ud(ud); // --> to set Rd accordingly        
    }

    /****************************************************************/
    TripodState computeTripodFK(const int which, const Vector &x) const
    {
        return computeTripodFK(which,(Ipopt::Number*)x.data());
    }

    /****************************************************************/
    TripodState computeTripodFK(const int which, const Ipopt::Number *x) const
    {
        const TripodParameters &params=(which==1)?tripod1:tripod2;
        int offs=(which==1)?0:9;

        double q33=sqrt(27.0)*params.l/sqrt(12.0*(x[offs+2]*x[offs+2]-(x[offs+0]+x[offs+1])*x[offs+2]+
                                            x[offs+1]*x[offs+1]-x[offs+0]*x[offs+1]+x[offs+0]*x[offs+0]+
                                            (27.0/12.0)*params.l*params.l));

        TripodState d;
        if (q33>=1.0)
        {
            d.n=params.z;
            d.u=0.0;
            d.p[0]=d.p[1]=0.0;
            d.p[2]=d.T(2,3)=x[offs+0];
        }
        else
        {
            Vector v1=params.s[0]+x[offs+0]*params.z;
            Vector v2=params.s[1]+x[offs+1]*params.z;
            Vector v3=params.s[2]+x[offs+2]*params.z;
            d.n=cross(v2-v1,v3-v1);
            d.n/=norm(d.n);

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
            double m1=params.l/q33*(-0.5*q11+1.5*q22);
            d.p[0]=params.l-m1*q11;
            d.p[1]=-m1*q21;
            d.p[2]=x[offs+0]-m1*q31;
            d.u*=acos(q33);

            // transformation matrix
            d.T(0,0)=q11; d.T(0,1)=q21; d.T(0,2)=-q31; d.T(0,3)=d.p[0];
            d.T(1,0)=q21; d.T(1,1)=q22; d.T(1,2)=-q32; d.T(1,3)=d.p[1];
            d.T(2,0)=q31; d.T(2,1)=q32; d.T(2,2)=q33;  d.T(2,3)=d.p[2];
        }

        return d;
    }

    /****************************************************************/
    Matrix computeFK(const Vector &x)
    {
        TripodState d1=computeTripodFK(1,x);
        TripodState d2=computeTripodFK(2,x);

        return T0*d1.T*arm.getH(CTRL_DEG2RAD*x.subVector(3,8))*d2.T*TN;
    }

    /****************************************************************/
    void set_q0(const Vector &x0)
    {
        this->x0[0]=std::max(tripod1.L_min,std::min(tripod1.L_max,x0[0]));
        this->x0[1]=std::max(tripod1.L_min,std::min(tripod1.L_max,x0[1]));
        this->x0[2]=std::max(tripod1.L_min,std::min(tripod1.L_max,x0[2]));

        iKinChain *chain=arm.asChain();
        for (size_t i=0; i<arm.getDOF(); i++)
            this->x0[3+i]=std::max((*chain)[i].getMin(),std::min((*chain)[i].getMax(),CTRL_DEG2RAD*x0[i]));

        this->x0[9]=std::max(tripod2.L_min,std::min(tripod2.L_max,x0[9]));
        this->x0[10]=std::max(tripod2.L_min,std::min(tripod2.L_max,x0[10]));
        this->x0[11]=std::max(tripod2.L_min,std::min(tripod2.L_max,x0[11]));
    }

    /****************************************************************/
    void set_xd(const Vector &xd)
    {
        this->xd=xd;
    }

    /****************************************************************/
    void set_ud(const Vector &ud)
    {
        this->ud=ud;

        double d=norm(ud);
        Vector tmp=(1.0/d)*ud;
        tmp.push_back(d);

        Rd=axis2dcm(tmp);
    }

    /****************************************************************/
    Vector get_result() const
    {
        Vector x_=x;
        for (size_t i=0; i<arm.getDOF(); i++)
            x_[3+i]*=CTRL_RAD2DEG;

        return x_;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=x0.length();
        m=1+1+1;
        nnz_jac_g=3+3+n;
        nnz_h_lag=0;
        index_style=TNLP::C_STYLE;

        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        size_t offs;

        offs=0;
        for (size_t i=0; i<3; i++)
        {
            x_l[offs+i]=tripod1.L_min;
            x_u[offs+i]=tripod1.L_max;
        }

        offs+=3;
        iKinChain *chain=arm.asChain();
        for (size_t i=0; i<arm.getDOF(); i++)
        {
            x_l[offs+i]=(*chain)[i].getMin();
            x_u[offs+i]=(*chain)[i].getMax();
        }

        offs+=arm.getDOF();
        for (size_t i=0; i<3; i++)
        {
            x_l[offs+i]=tripod2.L_min;
            x_u[offs+i]=tripod2.L_max;
        }

        g_l[0]=tripod1.cos_alpha_M; g_u[0]=1.0;
        g_l[1]=tripod2.cos_alpha_M; g_u[1]=1.0;
        g_l[2]=g_u[2]=0.0;

        return true;
    }

    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
    {
        for (Ipopt::Index i=0; i<n; i++)
            x[i]=x0[i];

        return true;
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        Vector q(arm.getDOF());
        for (size_t i=0; i<q.length(); i++)
            q[i]=x[3+i];

        TripodState d1=computeTripodFK(1,x);
        TripodState d2=computeTripodFK(2,x);

        Matrix T=T0*d1.T*arm.getH(q)*d2.T*TN;
        Vector e=dcm2axis(Rd*SE3inv(T));
        e*=e[3]; e.pop_back();

        Ipopt::Number postural_torso=0.0;
        Ipopt::Number postural_arm=0.0;
        Ipopt::Number tmp;

        tmp=x[0]-x[1];
        postural_torso+=tmp*tmp;
        tmp=x[1]-x[2];
        postural_torso+=tmp*tmp;

        for (size_t i=0; i<arm.getDOF(); i++)
        {
            tmp=x[3+i]-x0[3+i];
            postural_arm+=tmp*tmp;
        }
        
        obj_value=norm2(e)+WEIGHT_POSTURAL_TORSO*postural_torso+
                           WEIGHT_POSTURAL_ARM*postural_arm;

        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        Vector q(arm.getDOF());
        for (size_t i=0; i<q.length(); i++)
            q[i]=x[3+i];

        TripodState d1=computeTripodFK(1,x);
        TripodState d2=computeTripodFK(2,x);

        Matrix H=arm.getH(q);
        Matrix T=T0*d1.T*H*d2.T*TN;
        Vector e=dcm2axis(Rd*SE3inv(T));
        e*=e[3]; e.pop_back();

        Ipopt::Number x_dx[12];
        for (Ipopt::Index i=0; i<n; i++)
            x_dx[i]=x[i];

        TripodState d_fw,d_bw;
        Vector de_fw,de_bw;
        Matrix M;

        // tripod-1
        M=H*d2.T*TN;

        x_dx[0]=x[0]+drho;
        d_fw=computeTripodFK(1,x_dx);
        de_fw=dcm2axis(Rd*SE3inv(T0*d_fw.T*M)); de_fw*=de_fw[3]; de_fw.pop_back();
        x_dx[0]=x[0]-drho;
        d_bw=computeTripodFK(1,x_dx);
        de_bw=dcm2axis(Rd*SE3inv(T0*d_bw.T*M)); de_bw*=de_bw[3]; de_bw.pop_back();
        grad_f[0]=dot(e,de_fw-de_bw)/drho + 2.0*WEIGHT_POSTURAL_TORSO*(x[0]-x[1]);
        x_dx[0]=x[0];

        x_dx[1]=x[1]+drho;
        d_fw=computeTripodFK(1,x_dx);
        de_fw=dcm2axis(Rd*SE3inv(T0*d_fw.T*M)); de_fw*=de_fw[3]; de_fw.pop_back();
        x_dx[1]=x[1]-drho;
        d_bw=computeTripodFK(1,x_dx);
        de_bw=dcm2axis(Rd*SE3inv(T0*d_bw.T*M)); de_bw*=de_bw[3]; de_bw.pop_back();
        grad_f[1]=dot(e,de_fw-de_bw)/drho + 2.0*WEIGHT_POSTURAL_TORSO*(2.0*x[1]-x[0]-x[2]);
        x_dx[1]=x[1];

        x_dx[2]=x[2]+drho;
        d_fw=computeTripodFK(1,x_dx);
        de_fw=dcm2axis(Rd*SE3inv(T0*d_fw.T*M)); de_fw*=de_fw[3]; de_fw.pop_back();
        x_dx[2]=x[2]-drho;
        d_bw=computeTripodFK(1,x_dx);
        de_bw=dcm2axis(Rd*SE3inv(T0*d_bw.T*M)); de_bw*=de_bw[3]; de_bw.pop_back();
        grad_f[2]=dot(e,de_fw-de_bw)/drho + 2.0*WEIGHT_POSTURAL_TORSO*(x[2]-x[1]);
        x_dx[2]=x[2];

        // arm
        arm.setH0(T0*d1.T*H0); arm.setHN(HN*d2.T*TN);
                
        Vector eax=dcm2axis(Rd*SE3inv(arm.getH()));
        eax*=eax[3]; eax.pop_back();

        Matrix J=arm.GeoJacobian().submatrix(3,5,0,arm.getDOF()-1);
        Vector grad=-2.0*(J.transposed()*eax);

        arm.setH0(H0); arm.setHN(HN);

        for (size_t i=0; i<grad.length(); i++)
            grad_f[3+i]=grad[i] + 2.0*WEIGHT_POSTURAL_ARM*(x[3+i]-x0[3+i]);

        // tripod-2
        M=T0*d1.T*H;

        x_dx[9]=x[9]+drho;
        d_fw=computeTripodFK(2,x_dx);
        de_fw=dcm2axis(Rd*SE3inv(M*d_fw.T*TN)); de_fw*=de_fw[3]; de_fw.pop_back();
        x_dx[9]=x[9]-drho;
        d_bw=computeTripodFK(2,x_dx);
        de_bw=dcm2axis(Rd*SE3inv(M*d_bw.T*TN)); de_bw*=de_bw[3]; de_bw.pop_back();
        grad_f[9]=dot(e,de_fw-de_bw)/drho;
        x_dx[9]=x[9];

        x_dx[10]=x[10]+drho;
        d_fw=computeTripodFK(2,x_dx);
        de_fw=dcm2axis(Rd*SE3inv(M*d_fw.T*TN)); de_fw*=de_fw[3]; de_fw.pop_back();
        x_dx[10]=x[10]-drho;
        d_bw=computeTripodFK(2,x_dx);
        de_bw=dcm2axis(Rd*SE3inv(M*d_bw.T*TN)); de_bw*=de_bw[3]; de_bw.pop_back();
        grad_f[10]=dot(e,de_fw-de_bw)/drho;
        x_dx[10]=x[10];

        x_dx[11]=x[11]+drho;
        d_fw=computeTripodFK(2,x_dx);
        de_fw=dcm2axis(Rd*SE3inv(M*d_fw.T*TN)); de_fw*=de_fw[3]; de_fw.pop_back();
        x_dx[11]=x[11]-drho;
        d_bw=computeTripodFK(2,x_dx);
        de_bw=dcm2axis(Rd*SE3inv(M*d_bw.T*TN)); de_bw*=de_bw[3]; de_bw.pop_back();
        grad_f[11]=dot(e,de_fw-de_bw)/drho;
        x_dx[11]=x[11];

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        Vector q(arm.getDOF());
        for (size_t i=0; i<q.length(); i++)
            q[i]=x[3+i];

        TripodState d1=computeTripodFK(1,x);
        g[0]=d1.n[2];

        TripodState d2=computeTripodFK(2,x);
        g[1]=d2.n[2];

        Matrix T=T0*d1.T*arm.getH(q)*d2.T*TN;
        g[2]=norm2(xd-T.getCol(3).subVector(0,2));

        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        if (values==NULL)
        {
            // g[0] (tripod-1)
            iRow[0]=0; jCol[0]=0;
            iRow[1]=0; jCol[1]=1;
            iRow[2]=0; jCol[2]=2;

            // g[1] (tripod-2)
            iRow[3]=1; jCol[3]=9;
            iRow[4]=1; jCol[4]=10;
            iRow[5]=1; jCol[5]=11;

            // g[2]
            Ipopt::Index idx=6;
            for (Ipopt::Index col=0; col<n; col++)
            {
                iRow[idx]=2; jCol[idx]=col;
                idx++;
            }
        }
        else
        {
            Ipopt::Number x_dx[12];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw,d_bw;

            // g[0] (tripod-1)
            TripodState d1=computeTripodFK(1,x);

            x_dx[0]=x[0]+drho;
            d_fw=computeTripodFK(1,x_dx);
            x_dx[0]=x[0]-drho;
            d_bw=computeTripodFK(1,x_dx);
            values[0]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[0]=x[0];

            x_dx[1]=x[1]+drho;
            d_fw=computeTripodFK(1,x_dx);
            x_dx[1]=x[1]-drho;
            d_bw=computeTripodFK(1,x_dx);
            values[1]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[1]=x[1];

            x_dx[2]=x[2]+drho;
            d_fw=computeTripodFK(1,x_dx);
            x_dx[2]=x[2]-drho;
            d_bw=computeTripodFK(1,x_dx);
            values[2]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[2]=x[2];

            // g[1] (tripod-2)
            TripodState d2=computeTripodFK(2,x);

            x_dx[9]=x[9]+drho;
            d_fw=computeTripodFK(2,x_dx);
            x_dx[9]=x[9]-drho;
            d_bw=computeTripodFK(2,x_dx);
            values[3]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            d_fw=computeTripodFK(2,x_dx);
            x_dx[10]=x[10]-drho;
            d_bw=computeTripodFK(2,x_dx);
            values[4]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            d_fw=computeTripodFK(2,x_dx);
            x_dx[11]=x[11]-drho;
            d_bw=computeTripodFK(2,x_dx);
            values[5]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[11]=x[11];

            // g[3] (init)
            Vector q(arm.getDOF());
            for (size_t i=0; i<q.length(); i++)
                q[i]=x[3+i];

            Matrix H=arm.getH(q);
            Matrix T=T0*d1.T*H*d2.T*TN;
            Vector e=xd-T.getCol(3).subVector(0,2);
            Vector de_fw,de_bw;
            Matrix M;

            // g[3] (tripod-1)
            M=H*d2.T*TN;

            x_dx[0]=x[0]+drho;
            d_fw=computeTripodFK(1,x_dx);
            de_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            x_dx[0]=x[0]-drho;
            d_bw=computeTripodFK(1,x_dx);
            de_bw=xd-(T0*d_bw.T*M).getCol(3).subVector(0,2);
            values[6]=dot(e,de_fw-de_bw)/drho;
            x_dx[0]=x[0];

            x_dx[1]=x[1]+drho;
            d_fw=computeTripodFK(1,x_dx);
            de_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            x_dx[1]=x[1]-drho;
            d_bw=computeTripodFK(1,x_dx);
            de_bw=xd-(T0*d_bw.T*M).getCol(3).subVector(0,2);
            values[7]=dot(e,de_fw-de_bw)/drho;
            x_dx[1]=x[1];

            x_dx[2]=x[2]+drho;
            d_fw=computeTripodFK(1,x_dx);
            de_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            x_dx[2]=x[2]-drho;
            d_bw=computeTripodFK(1,x_dx);
            de_bw=xd-(T0*d_bw.T*M).getCol(3).subVector(0,2);
            values[8]=dot(e,de_fw-de_bw)/drho;
            x_dx[2]=x[2];

            // g[3] (arm)
            arm.setH0(T0*d1.T*H0); arm.setHN(HN*d2.T*TN);

            Matrix J=arm.GeoJacobian().submatrix(0,2,0,arm.getDOF()-1);
            Vector grad=-2.0*(J.transposed()*e);

            arm.setH0(H0); arm.setHN(HN);

            for (size_t i=0; i<grad.length(); i++)
                values[9+i]=grad[i];

            // g[3] (tripod-2)
            M=T0*d1.T*H;

            x_dx[9]=x[9]+drho;
            d_fw=computeTripodFK(2,x_dx);
            de_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[9]=x[9]-drho;
            d_bw=computeTripodFK(2,x_dx);
            de_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[15]=dot(e,de_fw-de_bw)/drho;
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            d_fw=computeTripodFK(2,x_dx);
            de_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[10]=x[10]-drho;
            d_bw=computeTripodFK(2,x_dx);
            de_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[16]=dot(e,de_fw-de_bw)/drho;
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            d_fw=computeTripodFK(2,x_dx);
            de_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[11]=x[11]-drho;
            d_bw=computeTripodFK(2,x_dx);
            de_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[17]=dot(e,de_fw-de_bw)/drho;
            x_dx[11]=x[11];
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
        for (Ipopt::Index i=0; i<n; i++)
            this->x[i]=x[i];
    }
};


/****************************************************************/
class IKSolver : public RFModule
{
    BufferedPort<Bottle> portTarget;
    BufferedPort<Bottle> portSolution;
    Vector q;

public:
    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        q.resize(12,0.0);
        portTarget.open("/solver/target:i");
        portSolution.open("/solver/solution:o");

        return true;
    }

    /****************************************************************/
    bool close()
    {
        portTarget.close();
        portSolution.close();

        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /****************************************************************/
    bool updateModule()
    {
        Bottle *target=portTarget.read(false);
        if (target!=NULL)
        {
            if (target->size()<6)
            {
                yError("wrong target size!");
                return true;
            }

            Vector xd(3),ud(3);

            xd[0]=target->get(0).asDouble();
            xd[1]=target->get(1).asDouble();
            xd[2]=target->get(2).asDouble();
            ud[0]=target->get(3).asDouble();
            ud[1]=target->get(4).asDouble();
            ud[2]=target->get(5).asDouble();

            Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
            app->Options()->SetNumericValue("tol",1e-3);
            app->Options()->SetNumericValue("acceptable_tol",1e-3);
            app->Options()->SetIntegerValue("acceptable_iter",10);
            app->Options()->SetStringValue("mu_strategy","adaptive");
            app->Options()->SetIntegerValue("max_iter",1000);
            app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
            app->Options()->SetStringValue("hessian_approximation","limited-memory");
            app->Options()->SetStringValue("derivative_test","none");
            app->Options()->SetIntegerValue("print_level",0);
            app->Initialize();

            Ipopt::SmartPtr<ChainNLP> nlp=new ChainNLP();

            nlp->set_q0(q);
            nlp->set_xd(xd);
            nlp->set_ud(ud);

            yInfo(" INPUT");
            yInfo(" q0 = (%s) [*]",q.toString(15,15).c_str());
            yInfo(" xd = (%s) [m]",xd.toString(15,15).c_str());
            yInfo(" ud = (%s) [rad]",ud.toString(15,15).c_str());
            yInfo("");

            double t0=Time::now();
            Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
            double t1=Time::now();

            q=nlp->get_result();
            TripodState d1=nlp->computeTripodFK(1,q);
            TripodState d2=nlp->computeTripodFK(2,q);
            Matrix T=nlp->computeFK(q);
            Vector x=T.getCol(3).subVector(0,2);
            Vector u=dcm2axis(T);
            u*=u[3]; u.pop_back();

            yInfo(" OUTPUT");
            yInfo(" q      = (%s) [*]",q.toString(15,15).c_str());
            yInfo(" e_x    = %g [m]",norm(xd-x));
            yInfo(" e_u    = %g [rad]",norm(ud-u));
            yInfo(" alpha1 = %g [deg]",CTRL_RAD2DEG*acos(d1.n[2]));
            yInfo(" alpha2 = %g [deg]",CTRL_RAD2DEG*acos(d2.n[2]));
            yInfo(" dt     = %g [ms]",1000.0*(t1-t0));
            yInfo("");

            Vector solution=cat(cat(xd,ud),q);
            portSolution.prepare().read(solution);
            portSolution.writeStrict();
        }
        
        return true;
    }
};


/****************************************************************/
int main()
{
#if 0
    Vector xd(3);
    xd[0]=0.3;
    xd[1]=0.1;
    xd[2]=0.1;

    Vector ud(3,0.0);
    ud[1]=M_PI/2.0;

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",1e-3);
    app->Options()->SetNumericValue("acceptable_tol",1e-3);
    app->Options()->SetIntegerValue("acceptable_iter",10);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetIntegerValue("max_iter",1000);
    app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("derivative_test","first-order");
    app->Options()->SetIntegerValue("print_level",4);
    app->Initialize();

    Ipopt::SmartPtr<ChainNLP> nlp=new ChainNLP();

    nlp->set_xd(xd);
    nlp->set_ud(ud);

    yInfo(" INPUT");
    yInfo(" xd = (%s) [m]",xd.toString(15,15).c_str());
    yInfo(" ud = (%s) [rad]",ud.toString(15,15).c_str());
    yInfo("");

    double t0=Time::now();
    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
    double t1=Time::now();

    Vector q=nlp->get_result();
    TripodState d1=nlp->computeTripodFK(1,q);
    TripodState d2=nlp->computeTripodFK(2,q);
    Matrix T=nlp->computeFK(q);
    Vector x=T.getCol(3).subVector(0,2);
    Vector u=dcm2axis(T);
    u*=u[3]; u.pop_back();

    yInfo(" OUTPUT");
    yInfo(" q      = (%s) [*]",q.toString(15,15).c_str());
    yInfo(" e_x    = %g [m]",norm(xd-x));
    yInfo(" e_u    = %g [rad]",norm(ud-u));
    yInfo(" alpha1 = %g [deg]",CTRL_RAD2DEG*acos(d1.n[2]));
    yInfo(" alpha2 = %g [deg]",CTRL_RAD2DEG*acos(d2.n[2]));
    yInfo(" dt     = %g [ms]",1000.0*(t1-t0));
    yInfo("");

    return 0;
#endif    

    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    IKSolver solver;
    ResourceFinder rf;
    return solver.runModule(rf);
}

