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
class MobileArmFullNoTorsoNoHeaveNLP_ForwardDiff : public MobileArmCommonNLP
{
public:
    /****************************************************************/
    MobileArmFullNoTorsoNoHeaveNLP_ForwardDiff(MobileArmSolver &slv_) : MobileArmCommonNLP(slv_)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "full_pose+mobile_base+no_torso_no_heave+forward_diff";
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=x0.length();
        m=2+1+1;
        nnz_jac_g=6+(n-4)+2;
        if(domain_constr)
        {
            m++;
            nnz_jac_g+=2;
        }
        nnz_h_lag=0;
        index_style=TNLP::C_STYLE;
yDebug() << "nb constr" << m;
        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        size_t offs;

        for (size_t i=0; i<2; i++)
        {
            x_l[idx_b+i]=std::numeric_limits<double>::lowest();
            x_u[idx_b+i]=std::numeric_limits<double>::max();
        }
        x_l[idx_b+2]=-M_PI;
        x_u[idx_b+2]=M_PI;

        for (size_t i=0; i<3; i++)
            x_l[idx_t+i]=x_u[idx_t+i]=x0[idx_t+i];

        x_l[idx_ua+0]=x_u[idx_ua+0]=x0[idx_ua+0];
        iKinChain *chain=upper_arm.asChain();        
        for (size_t i=1; i<upper_arm.getDOF(); i++)
        {
            x_l[idx_ua+i]=(*chain)[i].getMin();
            x_u[idx_ua+i]=(*chain)[i].getMax();
        }

        for (size_t i=0; i<3; i++)
        {
            x_l[idx_la+i]=lower_arm.l_min;
            x_u[idx_la+i]=lower_arm.l_max;
        }

        g_l[0]=g_u[0]=0.0;
        g_l[1]=lower_arm.cos_alpha_max; g_u[1]=1.0;

        g_l[2]=g_u[2]=0.0;

        g_l[3]=cover_shoulder_avoidance[1]; g_u[3]=std::numeric_limits<double>::max();

        if(domain_constr)
        {
            g_l[4]=0.0;
            g_u[4]=std::numeric_limits<double>::max();
        }

        latch_idx.clear();
        latch_gl.clear();
        latch_gu.clear();

        latch_idx.push_back(1);
        latch_gl.push_back(g_l[1]);
        latch_gu.push_back(g_u[1]);

        return true;
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        computeQuantities(x,new_x);

        Vector e=dcm2axis(Rd*(Rb*T).transposed());
        e*=e[3]; e.pop_back();

        Ipopt::Number postural_upper_arm=0.0;
        Ipopt::Number postural_lower_arm=0.0;
        Ipopt::Number tmp;

        if (wpostural_upper_arm!=0.0)
        {
            for (size_t i=1; i<upper_arm.getDOF(); i++)
            {
                tmp=x[idx_ua+i]-x0[idx_ua+i];
                postural_upper_arm+=tmp*tmp;
            }
        }

        if (wpostural_lower_arm!=0.0)
        {
            tmp=x[idx_la+0]-x[idx_la+1];
            postural_lower_arm+=tmp*tmp;
            tmp=x[idx_la+1]-x[idx_la+2];
            postural_lower_arm+=tmp*tmp;
        }

        obj_value=norm2(e)+
                  wpostural_upper_arm*postural_upper_arm+
                  wpostural_lower_arm*postural_lower_arm;

        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x,new_x);

        Vector e=dcm2axis(Rd*(Rb*T).transposed());
        e*=e[3]; e.pop_back();

        Ipopt::Number x_dx[n];
        for (Ipopt::Index i=0; i<n; i++)
            x_dx[i]=x[i];

        // base
        grad_f[idx_b+0]=0.0;
        grad_f[idx_b+1]=0.0;
        grad_f[idx_b+2]=-2.0*e[2];

        // torso
        grad_f[idx_t+0]=0.0;
        grad_f[idx_t+1]=0.0;
        grad_f[idx_t+2]=0.0;

        // upper_arm
        grad_f[idx_ua+0]=0.0;
        Vector eax=dcm2axis(Rd*(Rb*H_).transposed());
        eax*=eax[3]; eax.pop_back();
        Vector grad=-2.0*((Rb.submatrix(0,2,0,2)*J_.submatrix(3,5,0,upper_arm.getDOF()-1)).transposed()*eax);
        for (size_t i=1; i<grad.length(); i++)
            grad_f[idx_ua+i]=grad[i] + 2.0*wpostural_upper_arm*(x[idx_ua+i]-x0[idx_ua+i]);

        // lower_arm
        TripodState d_fw;
        Vector e_fw;
        Matrix M=d1.T*H;

        x_dx[idx_la+0]=x[idx_la+0]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=dcm2axis(Rd*(Rb*M*d_fw.T*TN).transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        grad_f[idx_la+0]=2.0*(dot(e,e_fw-e)/drho + wpostural_lower_arm*(x[idx_la+0]-x[idx_la+1]));
        x_dx[idx_la+0]=x[idx_la+0];

        x_dx[idx_la+1]=x[idx_la+1]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=dcm2axis(Rd*(Rb*M*d_fw.T*TN).transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        grad_f[idx_la+1]=2.0*(dot(e,e_fw-e)/drho + wpostural_lower_arm*(2.0*x[idx_la+1]-x[idx_la+0]-x[idx_la+2]));
        x_dx[idx_la+1]=x[idx_la+1];

        x_dx[idx_la+2]=x[idx_la+2]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=dcm2axis(Rd*(Rb*M*d_fw.T*TN).transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        grad_f[idx_la+2]=2.0*(dot(e,e_fw-e)/drho + wpostural_lower_arm*(x[idx_la+2]-x[idx_la+1]));
        x_dx[idx_la+2]=x[idx_la+2];

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        computeQuantities(x,new_x);

        double e2=hd2-din2.p[2];
        g[0]=e2*e2;
        g[1]=din2.n[2];

        Vector xe=Hb*T.getCol(3).subVector(0,3);
        xe.pop_back();

        g[2]=norm2(xd-xe);

        g[3]=-cover_shoulder_avoidance[0]*x[idx_ua+1]+x[idx_ua+2];

        if(domain_constr)
            g[4] = domain_dist;

        latch_x_verifying_alpha(n,x,g);

        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        if (values==NULL)
        {
            // g[0] (lower_arm)
            iRow[0]=0; jCol[0]=idx_la+0;
            iRow[1]=0; jCol[1]=idx_la+1;
            iRow[2]=0; jCol[2]=idx_la+2;

            // g[1] (lower_arm)
            iRow[3]=1; jCol[3]=idx_la+0;
            iRow[4]=1; jCol[4]=idx_la+1;
            iRow[5]=1; jCol[5]=idx_la+2;

            // g[2] (reaching position)
            Ipopt::Index idx=6;
            for (Ipopt::Index col=0; col<3; col++)
            {
                iRow[idx]=2; jCol[idx]=col;
                idx++;
            }
            for (Ipopt::Index col=idx_ua+1; col<n; col++)
            {
                iRow[idx]=2; jCol[idx]=col;
                idx++;
            }

            // g[3] (cover constraints)
            iRow[idx]=3; jCol[idx]=idx_ua+1;idx++;
            iRow[idx]=3; jCol[idx]=idx_ua+2;idx++;

            // g[4] (domain boundaries constraints)
            if(domain_constr)
            {
                iRow[idx]=4; jCol[idx]=0;idx++;
                iRow[idx]=4; jCol[idx]=1;
            }
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[n];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw;

            // g[0,1] (lower_arm)
            double e2=hd2-din2.p[2];

            x_dx[idx_la+0]=x[idx_la+0]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            values[0]=-2.0*e2*(d_fw.p[2]-din2.p[2])/drho;
            values[3]=(d_fw.n[2]-din2.n[2])/drho;
            x_dx[idx_la+0]=x[idx_la+0];

            x_dx[idx_la+1]=x[idx_la+1]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            values[1]=-2.0*e2*(d_fw.p[2]-din2.p[2])/drho;
            values[4]=(d_fw.n[2]-din2.n[2])/drho;
            x_dx[idx_la+1]=x[idx_la+1];

            x_dx[idx_la+2]=x[idx_la+2]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            values[2]=-2.0*e2*(d_fw.p[2]-din2.p[2])/drho;
            values[5]=(d_fw.n[2]-din2.n[2])/drho;
            x_dx[idx_la+2]=x[idx_la+2];

            // g[2] (init)
            Vector xe=Hb*T.getCol(3).subVector(0,3);
            xe.pop_back();

            Vector e=xd-xe;

            Ipopt::Index idx=6;

            // g[2] (base)

            values[idx]=-2.0*e[0];idx++;
            values[idx]=-2.0*e[1];idx++;

            Vector o(4,0.0);
            o[2] = 1.0;
            o[3] = M_PI/2.0+x[idx_b+2];
            Matrix Ro=axis2dcm(o);
            Vector v=Ro*T.getCol(3).subVector(0,3);
            v[2]=0.0;
            v.pop_back();
            values[idx]=-2.0*dot(v,e);idx++;

            // g[2] (upper_arm)
            Vector grad=-2.0*((Hb.submatrix(0,2,0,2)*J_.submatrix(0,2,0,upper_arm.getDOF()-1)).transposed()*e);
            for (size_t i=1; i<grad.length(); i++)
            {
                values[idx]=grad[i];
                idx++;
            }

            // g[2] (lower_arm)
            Vector e_fw;
            Matrix M=d1.T*H;

            x_dx[idx_la+0]=x[idx_la+0]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(Hb*M*d_fw.T*TN).getCol(3).subVector(0,2);
            values[idx]=2.0*dot(e,e_fw-e)/drho;idx++;
            x_dx[idx_la+0]=x[idx_la+0];

            x_dx[idx_la+1]=x[idx_la+1]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(Hb*M*d_fw.T*TN).getCol(3).subVector(0,2);
            values[idx]=2.0*dot(e,e_fw-e)/drho;idx++;
            x_dx[idx_la+1]=x[idx_la+1];

            x_dx[idx_la+2]=x[idx_la+2]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(Hb*M*d_fw.T*TN).getCol(3).subVector(0,2);
            values[idx]=2.0*dot(e,e_fw-e)/drho;idx++;
            x_dx[idx_la+2]=x[idx_la+2];

            // g[3]
            values[idx]=-cover_shoulder_avoidance[0];idx++;
            values[idx]=1.0;idx++;

            // g[4] (domain boundaries constraints)
            if(domain_constr)
            {
                double d = cv::pointPolygonTest(domain_poly, cv::Point2d(x[idx_b+0]+drho, x[idx_b+1]), true);
                values[idx]=(d-domain_dist)/drho;idx++;
                d = cv::pointPolygonTest(domain_poly, cv::Point2d(x[idx_b+0], x[idx_b+1]+drho), true);
                values[idx]=(d-domain_dist)/drho;
            }
        }

        return true;
    }
};


/****************************************************************/
class MobileArmFullNoTorsoNoHeaveNLP_CentralDiff : public MobileArmFullNoTorsoNoHeaveNLP_ForwardDiff
{
public:
    /****************************************************************/
    MobileArmFullNoTorsoNoHeaveNLP_CentralDiff(MobileArmSolver &slv_) :
        MobileArmFullNoTorsoNoHeaveNLP_ForwardDiff(slv_)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "full_pose+no_torso_no_heave+central_diff";
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x,new_x);

        Vector e=dcm2axis(Rd*(Rb*T).transposed());
        e*=e[3]; e.pop_back();

        Ipopt::Number x_dx[n];
        for (Ipopt::Index i=0; i<n; i++)
            x_dx[i]=x[i];

        // base
        grad_f[idx_b+0]=0.0;
        grad_f[idx_b+1]=0.0;
        grad_f[idx_b+2]=-2.0*e[2];

        // torso
        grad_f[idx_t+0]=0.0;
        grad_f[idx_t+1]=0.0;
        grad_f[idx_t+2]=0.0;

        // upper_arm
        grad_f[idx_ua+0]=0.0;
        Vector eax=dcm2axis(Rd*(Rb*H_).transposed());
        eax*=eax[3]; eax.pop_back();
        Vector grad=-2.0*((Rb.submatrix(0,2,0,2)*J_.submatrix(3,5,0,upper_arm.getDOF()-1)).transposed()*eax);
        for (size_t i=1; i<grad.length(); i++)
            grad_f[idx_ua+i]=grad[i] + 2.0*wpostural_upper_arm*(x[idx_ua+i]-x0[idx_ua+i]);

        // lower_arm
        TripodState d_fw,d_bw;
        Vector e_fw,e_bw;
        Matrix M=d1.T*H;

        x_dx[idx_la+0]=x[idx_la+0]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=dcm2axis(Rd*(Rb*M*d_fw.T*TN).transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        x_dx[idx_la+0]=x[idx_la+0]-drho;
        d_bw=tripod_fkin(2,x_dx);
        e_bw=dcm2axis(Rd*(Rb*M*d_bw.T*TN).transposed()); e_bw*=e_bw[3]; e_bw.pop_back();
        grad_f[idx_la+0]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_lower_arm*(x[idx_la+0]-x[idx_la+1]);
        x_dx[idx_la+0]=x[idx_la+0];

        x_dx[idx_la+1]=x[idx_la+1]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=dcm2axis(Rd*(Rb*M*d_fw.T*TN).transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        x_dx[idx_la+1]=x[idx_la+1]-drho;
        d_bw=tripod_fkin(2,x_dx);
        e_bw=dcm2axis(Rd*(Rb*M*d_bw.T*TN).transposed()); e_bw*=e_bw[3]; e_bw.pop_back();
        grad_f[idx_la+1]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_lower_arm*(2.0*x[idx_la+1]-x[idx_la+0]-x[idx_la+2]);
        x_dx[idx_la+1]=x[idx_la+1];

        x_dx[idx_la+2]=x[idx_la+2]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=dcm2axis(Rd*(Rb*M*d_fw.T*TN).transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        x_dx[idx_la+2]=x[idx_la+2]-drho;
        d_bw=tripod_fkin(2,x_dx);
        e_bw=dcm2axis(Rd*(Rb*M*d_bw.T*TN).transposed()); e_bw*=e_bw[3]; e_bw.pop_back();
        grad_f[idx_la+2]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_lower_arm*(x[idx_la+2]-x[idx_la+1]);
        x_dx[idx_la+2]=x[idx_la+2];

        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {        
        if (values==NULL)
        {
            // g[0] (lower_arm)
            iRow[0]=0; jCol[0]=idx_la+0;
            iRow[1]=0; jCol[1]=idx_la+1;
            iRow[2]=0; jCol[2]=idx_la+2;

            // g[1] (lower_arm)
            iRow[3]=1; jCol[3]=idx_la+0;
            iRow[4]=1; jCol[4]=idx_la+1;
            iRow[5]=1; jCol[5]=idx_la+2;

            // g[2] (reaching position)
            Ipopt::Index idx=6;
            for (Ipopt::Index col=0; col<3; col++)
            {
                iRow[idx]=2; jCol[idx]=col;
                idx++;
            }
            for (Ipopt::Index col=idx_ua+1; col<n; col++)
            {
                iRow[idx]=2; jCol[idx]=col;
                idx++;
            }

            // g[3] (cover constraints)
            iRow[idx]=3; jCol[idx]=idx_ua+1;
            iRow[idx+1]=3; jCol[idx+1]=idx_ua+2;
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[n];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw,d_bw;

            // g[0,1] (lower_arm)
            double e2=hd2-din2.p[2];

            x_dx[idx_la+0]=x[idx_la+0]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            x_dx[idx_la+0]=x[idx_la+0]-drho;
            tripod_fkin(2,x_dx,&d_bw);
            values[0]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;
            values[3]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[idx_la+0]=x[idx_la+0];

            x_dx[idx_la+1]=x[idx_la+1]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            x_dx[idx_la+1]=x[idx_la+1]-drho;
            tripod_fkin(2,x_dx,&d_bw);
            values[1]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;
            values[4]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[idx_la+1]=x[idx_la+1];

            x_dx[idx_la+2]=x[idx_la+2]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            x_dx[idx_la+2]=x[idx_la+2]-drho;
            tripod_fkin(2,x_dx,&d_bw);
            values[2]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;
            values[5]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[idx_la+2]=x[idx_la+2];

            // g[2] (init)
            Vector xe=Hb*T.getCol(3).subVector(0,3);
            xe.pop_back();

            Vector e=xd-xe;

            Ipopt::Index idx=6;

            // g[2] (base)

            values[idx]=-2.0*e[0];idx++;
            values[idx]=-2.0*e[1];idx++;

            Vector o(4,0.0);
            o[2] = 1.0;
            o[3] = M_PI/2.0+x[idx_b+2];
            Matrix Ro=axis2dcm(o);
            Vector v=Ro*T.getCol(3).subVector(0,3);
            v[2]=0.0;
            v.pop_back();
            values[idx]=-2.0*dot(v,e);idx++;

            // g[2] (upper_arm)
            Vector grad=-2.0*((Hb.submatrix(0,2,0,2)*J_.submatrix(0,2,0,upper_arm.getDOF()-1)).transposed()*e);
            for (size_t i=1; i<grad.length(); i++)
            {
                values[idx]=grad[i];
                idx++;
            }

            // g[2] (lower_arm)
            Vector e_fw,e_bw;
            Matrix M=d1.T*H;

            x_dx[idx_la+0]=x[idx_la+0]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(Hb*M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[idx_la+0]=x[idx_la+0]-drho;
            d_bw=tripod_fkin(2,x_dx);
            e_bw=xd-(Hb*M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[idx]=dot(e,e_fw-e_bw)/drho;idx++;
            x_dx[idx_la+0]=x[idx_la+0];

            x_dx[idx_la+1]=x[idx_la+1]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(Hb*M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[idx_la+1]=x[idx_la+1]-drho;
            d_bw=tripod_fkin(2,x_dx);
            e_bw=xd-(Hb*M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[idx]=dot(e,e_fw-e_bw)/drho;idx++;
            x_dx[idx_la+1]=x[idx_la+1];

            x_dx[idx_la+2]=x[idx_la+2]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(Hb*M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[idx_la+2]=x[idx_la+2]-drho;
            d_bw=tripod_fkin(2,x_dx);
            e_bw=xd-(Hb*M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[idx]=dot(e,e_fw-e_bw)/drho;idx++;
            x_dx[idx_la+2]=x[idx_la+2];

            // g[3]
            values[idx]=-cover_shoulder_avoidance[0];
            values[idx+1]=1.0;
        }

        return true;
    }
};


