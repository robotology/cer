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


/****************************************************************/
class ArmFullNLP : public Ipopt::TNLP
{
protected:
    const TripodParametersExtended torso;    
    const TripodParametersExtended lower_arm;
    iKinLimb &upper_arm;

    double drho;

    Matrix &T0,&TN;
    double &zd1,&zd2;

    Matrix H0,HN,Rd;
    Vector x0,x;
    Vector xd,ud;

public:
    /****************************************************************/
    ArmFullNLP(ArmParameters &pa, SolverParameters &ps) :
               torso(pa.torso), upper_arm(pa.upper_arm), lower_arm(pa.lower_arm),
               T0(pa.T0), TN(pa.TN),
               zd1(ps.torso_heave), zd2(ps.lower_arm_heave)
    {
        drho=DELTA_RHO;

        H0=upper_arm.getH0();
        HN=upper_arm.getHN();

        x0.resize(12,0.0);

        iKinChain *chain=upper_arm.asChain();
        for (size_t i=0; i<upper_arm.getDOF(); i++)
            x0[3+i]=0.5*((*chain)[i].getMin()+(*chain)[i].getMax());
        
        x=x0;

        set_target(eye(4,4));
    }

    /****************************************************************/
    TripodState tripod_fkin(const int which, const Vector &x) const
    {
        return tripod_fkin(which,(Ipopt::Number*)x.data());
    }

    /****************************************************************/
    TripodState tripod_fkin(const int which, const Ipopt::Number *x) const
    {
        const TripodParametersExtended &params=((which==1)?torso:lower_arm);
        int offs=(which==1)?0:9;

        double q33=sqrt(27.0)*params.r/sqrt(12.0*(x[offs+2]*x[offs+2]-(x[offs+0]+x[offs+1])*x[offs+2]+
                                            x[offs+1]*x[offs+1]-x[offs+0]*x[offs+1]+x[offs+0]*x[offs+0]+
                                            (27.0/12.0)*params.r*params.r));

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
            double m1=params.r/q33*(-0.5*q11+1.5*q22);
            d.p[0]=params.r-m1*q11;
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
    Matrix fkin(const Vector &x)
    {
        TripodState d1=tripod_fkin(1,x);
        TripodState d2=tripod_fkin(2,x);

        return T0*d1.T*upper_arm.getH(CTRL_DEG2RAD*x.subVector(3,8))*d2.T*TN;
    }

    /****************************************************************/
    void set_q0(const Vector &x0)
    {
        this->x0[0]=std::max(torso.l_min,std::min(torso.l_max,x0[0]));
        this->x0[1]=std::max(torso.l_min,std::min(torso.l_max,x0[1]));
        this->x0[2]=std::max(torso.l_min,std::min(torso.l_max,x0[2]));

        iKinChain *chain=upper_arm.asChain();
        for (size_t i=0; i<upper_arm.getDOF(); i++)
            this->x0[3+i]=std::max((*chain)[i].getMin(),std::min((*chain)[i].getMax(),CTRL_DEG2RAD*x0[i]));

        this->x0[9]=std::max(lower_arm.l_min,std::min(lower_arm.l_max,x0[9]));
        this->x0[10]=std::max(lower_arm.l_min,std::min(lower_arm.l_max,x0[10]));
        this->x0[11]=std::max(lower_arm.l_min,std::min(lower_arm.l_max,x0[11]));
    }

    /****************************************************************/
    void set_target(const Matrix &Hd)
    {
        xd=Hd.getCol(3).subVector(0,2);

        Rd=Hd;
        Rd(0,3)=Rd(1,3)=Rd(2,3)=0.0;

        ud=dcm2axis(Rd);
        ud*=ud[3];
        ud.pop_back();
    }

    /****************************************************************/
    Vector get_result() const
    {
        Vector x_=x;
        for (size_t i=0; i<upper_arm.getDOF(); i++)
            x_[3+i]*=CTRL_RAD2DEG;

        return x_;
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=x0.length();
        m=2+2+1;
        nnz_jac_g=6+6+n;
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
            x_l[offs+i]=torso.l_min;
            x_u[offs+i]=torso.l_max;
        }

        offs+=3;
        iKinChain *chain=upper_arm.asChain();
        for (size_t i=0; i<upper_arm.getDOF(); i++)
        {
            x_l[offs+i]=(*chain)[i].getMin();
            x_u[offs+i]=(*chain)[i].getMax();
        }

        offs+=upper_arm.getDOF();
        for (size_t i=0; i<3; i++)
        {
            x_l[offs+i]=lower_arm.l_min;
            x_u[offs+i]=lower_arm.l_max;
        }

        g_l[0]=g_u[0]=0.0;
        g_l[1]=torso.cos_alpha_max; g_u[1]=1.0;

        g_l[2]=g_u[2]=0.0;
        g_l[3]=lower_arm.cos_alpha_max; g_u[3]=1.0;

        g_l[4]=g_u[4]=0.0;

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
        Vector q(upper_arm.getDOF());
        for (size_t i=0; i<q.length(); i++)
            q[i]=x[3+i];

        TripodState d1=tripod_fkin(1,x);
        TripodState d2=tripod_fkin(2,x);

        Matrix T=T0*d1.T*upper_arm.getH(q)*d2.T*TN;
        Vector e=dcm2axis(Rd*SE3inv(T));
        e*=e[3]; e.pop_back();

        Ipopt::Number postural_torso=0.0;
        Ipopt::Number postural_arm=0.0;
        Ipopt::Number tmp;

        tmp=x[0]-x[1];
        postural_torso+=tmp*tmp;
        tmp=x[1]-x[2];
        postural_torso+=tmp*tmp;

        for (size_t i=0; i<upper_arm.getDOF(); i++)
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
        Vector q(upper_arm.getDOF());
        for (size_t i=0; i<q.length(); i++)
            q[i]=x[3+i];

        TripodState d1=tripod_fkin(1,x);
        TripodState d2=tripod_fkin(2,x);

        Matrix H=upper_arm.getH(q);
        Matrix T=T0*d1.T*H*d2.T*TN;
        Vector e=dcm2axis(Rd*SE3inv(T));
        e*=e[3]; e.pop_back();

        Ipopt::Number x_dx[12];
        for (Ipopt::Index i=0; i<n; i++)
            x_dx[i]=x[i];

        TripodState d_fw,d_bw;
        Vector de_fw,de_bw;
        Matrix M;

        // torso
        M=H*d2.T*TN;

        x_dx[0]=x[0]+drho;
        d_fw=tripod_fkin(1,x_dx);
        de_fw=dcm2axis(Rd*SE3inv(T0*d_fw.T*M)); de_fw*=de_fw[3]; de_fw.pop_back();
        x_dx[0]=x[0]-drho;
        d_bw=tripod_fkin(1,x_dx);
        de_bw=dcm2axis(Rd*SE3inv(T0*d_bw.T*M)); de_bw*=de_bw[3]; de_bw.pop_back();
        grad_f[0]=dot(e,de_fw-de_bw)/drho + 2.0*WEIGHT_POSTURAL_TORSO*(x[0]-x[1]);
        x_dx[0]=x[0];

        x_dx[1]=x[1]+drho;
        d_fw=tripod_fkin(1,x_dx);
        de_fw=dcm2axis(Rd*SE3inv(T0*d_fw.T*M)); de_fw*=de_fw[3]; de_fw.pop_back();
        x_dx[1]=x[1]-drho;
        d_bw=tripod_fkin(1,x_dx);
        de_bw=dcm2axis(Rd*SE3inv(T0*d_bw.T*M)); de_bw*=de_bw[3]; de_bw.pop_back();
        grad_f[1]=dot(e,de_fw-de_bw)/drho + 2.0*WEIGHT_POSTURAL_TORSO*(2.0*x[1]-x[0]-x[2]);
        x_dx[1]=x[1];

        x_dx[2]=x[2]+drho;
        d_fw=tripod_fkin(1,x_dx);
        de_fw=dcm2axis(Rd*SE3inv(T0*d_fw.T*M)); de_fw*=de_fw[3]; de_fw.pop_back();
        x_dx[2]=x[2]-drho;
        d_bw=tripod_fkin(1,x_dx);
        de_bw=dcm2axis(Rd*SE3inv(T0*d_bw.T*M)); de_bw*=de_bw[3]; de_bw.pop_back();
        grad_f[2]=dot(e,de_fw-de_bw)/drho + 2.0*WEIGHT_POSTURAL_TORSO*(x[2]-x[1]);
        x_dx[2]=x[2];

        // upper_arm
        upper_arm.setH0(T0*d1.T*H0); upper_arm.setHN(HN*d2.T*TN);
                
        Vector eax=dcm2axis(Rd*SE3inv(upper_arm.getH()));
        eax*=eax[3]; eax.pop_back();

        Matrix J=upper_arm.GeoJacobian().submatrix(3,5,0,upper_arm.getDOF()-1);
        Vector grad=-2.0*(J.transposed()*eax);

        upper_arm.setH0(H0); upper_arm.setHN(HN);

        for (size_t i=0; i<grad.length(); i++)
            grad_f[3+i]=grad[i] + 2.0*WEIGHT_POSTURAL_ARM*(x[3+i]-x0[3+i]);

        // lower_arm
        M=T0*d1.T*H;

        x_dx[9]=x[9]+drho;
        d_fw=tripod_fkin(2,x_dx);
        de_fw=dcm2axis(Rd*SE3inv(M*d_fw.T*TN)); de_fw*=de_fw[3]; de_fw.pop_back();
        x_dx[9]=x[9]-drho;
        d_bw=tripod_fkin(2,x_dx);
        de_bw=dcm2axis(Rd*SE3inv(M*d_bw.T*TN)); de_bw*=de_bw[3]; de_bw.pop_back();
        grad_f[9]=dot(e,de_fw-de_bw)/drho;
        x_dx[9]=x[9];

        x_dx[10]=x[10]+drho;
        d_fw=tripod_fkin(2,x_dx);
        de_fw=dcm2axis(Rd*SE3inv(M*d_fw.T*TN)); de_fw*=de_fw[3]; de_fw.pop_back();
        x_dx[10]=x[10]-drho;
        d_bw=tripod_fkin(2,x_dx);
        de_bw=dcm2axis(Rd*SE3inv(M*d_bw.T*TN)); de_bw*=de_bw[3]; de_bw.pop_back();
        grad_f[10]=dot(e,de_fw-de_bw)/drho;
        x_dx[10]=x[10];

        x_dx[11]=x[11]+drho;
        d_fw=tripod_fkin(2,x_dx);
        de_fw=dcm2axis(Rd*SE3inv(M*d_fw.T*TN)); de_fw*=de_fw[3]; de_fw.pop_back();
        x_dx[11]=x[11]-drho;
        d_bw=tripod_fkin(2,x_dx);
        de_bw=dcm2axis(Rd*SE3inv(M*d_bw.T*TN)); de_bw*=de_bw[3]; de_bw.pop_back();
        grad_f[11]=dot(e,de_fw-de_bw)/drho;
        x_dx[11]=x[11];

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        Vector q(upper_arm.getDOF());
        for (size_t i=0; i<q.length(); i++)
            q[i]=x[3+i];

        TripodState d1=tripod_fkin(1,x);
        double e1=zd1-d1.p[2];
        g[0]=e1*e1;
        g[1]=d1.n[2];

        TripodState d2=tripod_fkin(2,x);
        double e2=zd2-d2.p[2];
        g[2]=e2*e2;
        g[3]=d2.n[2];

        Matrix T=T0*d1.T*upper_arm.getH(q)*d2.T*TN;
        g[4]=norm2(xd-T.getCol(3).subVector(0,2));

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

            // g[1] (tripod-1)
            iRow[3]=1; jCol[3]=0;
            iRow[4]=1; jCol[4]=1;
            iRow[5]=1; jCol[5]=2;

            // g[2] (tripod-2)
            iRow[6]=2; jCol[6]=9;
            iRow[7]=2; jCol[7]=10;
            iRow[8]=2; jCol[8]=11;

            // g[3] (tripod-2)
            iRow[9]=3;  jCol[9]=9;
            iRow[10]=3; jCol[10]=10;
            iRow[11]=3; jCol[11]=11;

            // g[4]
            Ipopt::Index idx=12;
            for (Ipopt::Index col=0; col<n; col++)
            {
                iRow[idx]=4; jCol[idx]=col;
                idx++;
            }
        }
        else
        {
            Ipopt::Number x_dx[12];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw,d_bw;

            // g[0,1] (torso)
            TripodState d1=tripod_fkin(1,x);
            double e1=zd1-d1.p[2];

            x_dx[0]=x[0]+drho;
            d_fw=tripod_fkin(1,x_dx);
            x_dx[0]=x[0]-drho;
            d_bw=tripod_fkin(1,x_dx);
            values[0]=-e1*(d_fw.p[2]-d_bw.p[2])/drho;
            values[3]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[0]=x[0];

            x_dx[1]=x[1]+drho;
            d_fw=tripod_fkin(1,x_dx);
            x_dx[1]=x[1]-drho;
            d_bw=tripod_fkin(1,x_dx);
            values[1]=-e1*(d_fw.p[2]-d_bw.p[2])/drho;
            values[4]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[1]=x[1];

            x_dx[2]=x[2]+drho;
            d_fw=tripod_fkin(1,x_dx);
            x_dx[2]=x[2]-drho;
            d_bw=tripod_fkin(1,x_dx);
            values[2]=-e1*(d_fw.p[2]-d_bw.p[2])/drho;
            values[5]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[2]=x[2];

            // g[2,3] (lower_arm)
            TripodState d2=tripod_fkin(2,x);
            double e2=zd2-d2.p[2];

            x_dx[9]=x[9]+drho;
            d_fw=tripod_fkin(2,x_dx);
            x_dx[9]=x[9]-drho;
            d_bw=tripod_fkin(2,x_dx);
            values[6]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;
            values[9]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            d_fw=tripod_fkin(2,x_dx);
            x_dx[10]=x[10]-drho;
            d_bw=tripod_fkin(2,x_dx);
            values[7]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;
            values[10]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            d_fw=tripod_fkin(2,x_dx);
            x_dx[11]=x[11]-drho;
            d_bw=tripod_fkin(2,x_dx);
            values[8]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;
            values[11]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[11]=x[11];

            // g[4] (init)
            Vector q(upper_arm.getDOF());
            for (size_t i=0; i<q.length(); i++)
                q[i]=x[3+i];

            Matrix H=upper_arm.getH(q);
            Matrix T=T0*d1.T*H*d2.T*TN;
            Vector e=xd-T.getCol(3).subVector(0,2);
            Vector de_fw,de_bw;
            Matrix M;

            // g[4] (torso)
            M=H*d2.T*TN;

            x_dx[0]=x[0]+drho;
            d_fw=tripod_fkin(1,x_dx);
            de_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            x_dx[0]=x[0]-drho;
            d_bw=tripod_fkin(1,x_dx);
            de_bw=xd-(T0*d_bw.T*M).getCol(3).subVector(0,2);
            values[12]=dot(e,de_fw-de_bw)/drho;
            x_dx[0]=x[0];

            x_dx[1]=x[1]+drho;
            d_fw=tripod_fkin(1,x_dx);
            de_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            x_dx[1]=x[1]-drho;
            d_bw=tripod_fkin(1,x_dx);
            de_bw=xd-(T0*d_bw.T*M).getCol(3).subVector(0,2);
            values[13]=dot(e,de_fw-de_bw)/drho;
            x_dx[1]=x[1];

            x_dx[2]=x[2]+drho;
            d_fw=tripod_fkin(1,x_dx);
            de_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            x_dx[2]=x[2]-drho;
            d_bw=tripod_fkin(1,x_dx);
            de_bw=xd-(T0*d_bw.T*M).getCol(3).subVector(0,2);
            values[14]=dot(e,de_fw-de_bw)/drho;
            x_dx[2]=x[2];

            // g[4] (upper_arm)
            upper_arm.setH0(T0*d1.T*H0); upper_arm.setHN(HN*d2.T*TN);

            Matrix J=upper_arm.GeoJacobian().submatrix(0,2,0,upper_arm.getDOF()-1);
            Vector grad=-2.0*(J.transposed()*e);

            upper_arm.setH0(H0); upper_arm.setHN(HN);

            for (size_t i=0; i<grad.length(); i++)
                values[15+i]=grad[i];

            // g[4] (lower_arm)
            M=T0*d1.T*H;

            x_dx[9]=x[9]+drho;
            d_fw=tripod_fkin(2,x_dx);
            de_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[9]=x[9]-drho;
            d_bw=tripod_fkin(2,x_dx);
            de_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[21]=dot(e,de_fw-de_bw)/drho;
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            d_fw=tripod_fkin(2,x_dx);
            de_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[10]=x[10]-drho;
            d_bw=tripod_fkin(2,x_dx);
            de_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[22]=dot(e,de_fw-de_bw)/drho;
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            d_fw=tripod_fkin(2,x_dx);
            de_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[11]=x[11]-drho;
            d_bw=tripod_fkin(2,x_dx);
            de_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[23]=dot(e,de_fw-de_bw)/drho;
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

