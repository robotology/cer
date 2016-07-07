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
class ArmFullNoTorsoHeaveNLP_ForwardDiff : public ArmCommonNLP
{
public:
    /****************************************************************/
    ArmFullNoTorsoHeaveNLP_ForwardDiff(ArmSolver &slv_) : ArmCommonNLP(slv_)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "full_pose+no_torso_heave+forward_diff";
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=x0.length();
        m=1+1;
        nnz_jac_g=3+(n-4);
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
            x_l[offs+i]=x_u[offs+i]=x0[i];

        offs+=3;
        x_l[offs+0]=x_u[offs+0]=x0[offs+0];
        iKinChain *chain=upper_arm.asChain();
        for (size_t i=1; i<upper_arm.getDOF(); i++)
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

        g_l[0]=lower_arm.cos_alpha_max; g_u[0]=1.0;
        g_l[1]=g_u[1]=0.0;

        latch_idx.clear();
        latch_gl.clear();
        latch_gu.clear();

        latch_idx.push_back(0);
        latch_gl.push_back(g_l[0]);
        latch_gu.push_back(g_u[0]);

        return true;
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        computeQuantities(x,new_x);

        Vector e=dcm2axis(Rd*T.transposed());
        e*=e[3]; e.pop_back();

        Ipopt::Number postural_upper_arm=0.0;
        Ipopt::Number postural_lower_arm=0.0;
        Ipopt::Number tmp;

        if (wpostural_upper_arm!=0.0)
        {
            for (size_t i=1; i<upper_arm.getDOF(); i++)
            {
                tmp=x[3+i]-x0[3+i];
                postural_upper_arm+=tmp*tmp;
            }
        }

        if (wpostural_lower_arm!=0.0)
        {
            tmp=x[9]-x[10];
            postural_lower_arm+=tmp*tmp;
            tmp=x[10]-x[11];
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

        Vector e=dcm2axis(Rd*T.transposed());
        e*=e[3]; e.pop_back();

        Ipopt::Number x_dx[12];
        for (Ipopt::Index i=0; i<n; i++)
            x_dx[i]=x[i];

        // torso
        grad_f[0]=0.0;
        grad_f[1]=0.0;
        grad_f[2]=0.0;
        grad_f[3]=0.0;

        // upper_arm
        Vector eax=dcm2axis(Rd*H_.transposed());
        eax*=eax[3]; eax.pop_back();
        Vector grad=-2.0*(J_.submatrix(3,5,0,upper_arm.getDOF()-1).transposed()*eax);
        for (size_t i=1; i<grad.length(); i++)
            grad_f[3+i]=grad[i] + 2.0*wpostural_upper_arm*(x[3+i]-x0[3+i]);

        // lower_arm
        TripodState d_fw;
        Vector e_fw;
        Matrix M=d1.T*H;

        x_dx[9]=x[9]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=dcm2axis(Rd*(M*d_fw.T*TN).transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        grad_f[9]=2.0*(dot(e,e_fw-e)/drho + wpostural_lower_arm*(x[9]-x[10]));
        x_dx[9]=x[9];

        x_dx[10]=x[10]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=dcm2axis(Rd*(M*d_fw.T*TN).transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        grad_f[10]=2.0*(dot(e,e_fw-e)/drho + wpostural_lower_arm*(2.0*x[10]-x[9]-x[11]));
        x_dx[10]=x[10];

        x_dx[11]=x[11]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=dcm2axis(Rd*(M*d_fw.T*TN).transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        grad_f[11]=2.0*(dot(e,e_fw-e)/drho + wpostural_lower_arm*(x[11]-x[10]));
        x_dx[11]=x[11];

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        computeQuantities(x,new_x);

        g[0]=din2.n[0];
        g[1]=norm2(xd-T.getCol(3).subVector(0,2));

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
            iRow[0]=0; jCol[0]=9;
            iRow[1]=0; jCol[1]=10;
            iRow[2]=0; jCol[2]=11;

            // g[1]
            Ipopt::Index idx=3;
            for (Ipopt::Index col=4; col<n; col++)
            {
                iRow[idx]=1; jCol[idx]=col;
                idx++;
            }
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[12];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw;

            // g[0] (lower_arm)
            x_dx[9]=x[9]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            values[0]=(d_fw.n[2]-din2.n[2])/drho;
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            values[1]=(d_fw.n[2]-din2.n[2])/drho;
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            values[2]=(d_fw.n[2]-din2.n[2])/drho;
            x_dx[11]=x[11];

            // g[1] (init)
            Vector e=xd-T.getCol(3).subVector(0,2);

            // g[1] (upper_arm)
            Vector grad=-2.0*(J_.submatrix(0,2,0,upper_arm.getDOF()-1).transposed()*e);
            for (size_t i=1; i<grad.length(); i++)
                values[2+i]=grad[i];

            // g[1] (lower_arm)
            Vector e_fw;
            Matrix M=d1.T*H;

            x_dx[9]=x[9]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            values[8]=2.0*dot(e,e_fw-e)/drho;
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            values[9]=2.0*dot(e,e_fw-e)/drho;
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            values[10]=2.0*dot(e,e_fw-e)/drho;
            x_dx[11]=x[11];
        }

        return true;
    }
};


/****************************************************************/
class ArmFullNoTorsoHeaveNLP_CentralDiff : public ArmFullNoTorsoHeaveNLP_ForwardDiff
{
public:
    /****************************************************************/
    ArmFullNoTorsoHeaveNLP_CentralDiff(ArmSolver &slv_) :
        ArmFullNoTorsoHeaveNLP_ForwardDiff(slv_)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "full_pose+no_torso_heave+central_diff";
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x,new_x);

        Vector e=dcm2axis(Rd*T.transposed());
        e*=e[3]; e.pop_back();

        Ipopt::Number x_dx[12];
        for (Ipopt::Index i=0; i<n; i++)
            x_dx[i]=x[i];

        // torso
        grad_f[0]=0.0;
        grad_f[1]=0.0;
        grad_f[2]=0.0;
        grad_f[3]=0.0;

        // upper_arm
        Vector eax=dcm2axis(Rd*H_.transposed());
        eax*=eax[3]; eax.pop_back();
        Vector grad=-2.0*(J_.submatrix(3,5,0,upper_arm.getDOF()-1).transposed()*eax);
        for (size_t i=1; i<grad.length(); i++)
            grad_f[3+i]=grad[i] + 2.0*wpostural_upper_arm*(x[3+i]-x0[3+i]);

        // lower_arm
        TripodState d_fw,d_bw;
        Vector e_fw,e_bw;
        Matrix M=d1.T*H;

        x_dx[9]=x[9]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=dcm2axis(Rd*(M*d_fw.T*TN).transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        x_dx[9]=x[9]-drho;
        d_bw=tripod_fkin(2,x_dx);
        e_bw=dcm2axis(Rd*(M*d_bw.T*TN).transposed()); e_bw*=e_bw[3]; e_bw.pop_back();
        grad_f[9]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_lower_arm*(x[9]-x[10]);
        x_dx[9]=x[9];

        x_dx[10]=x[10]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=dcm2axis(Rd*(M*d_fw.T*TN).transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        x_dx[10]=x[10]-drho;
        d_bw=tripod_fkin(2,x_dx);
        e_bw=dcm2axis(Rd*(M*d_bw.T*TN).transposed()); e_bw*=e_bw[3]; e_bw.pop_back();
        grad_f[10]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_lower_arm*(2.0*x[10]-x[9]-x[11]);
        x_dx[10]=x[10];

        x_dx[11]=x[11]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=dcm2axis(Rd*(M*d_fw.T*TN).transposed()); e_fw*=e_fw[3]; e_fw.pop_back();
        x_dx[11]=x[11]-drho;
        d_bw=tripod_fkin(2,x_dx);
        e_bw=dcm2axis(Rd*(M*d_bw.T*TN).transposed()); e_bw*=e_bw[3]; e_bw.pop_back();
        grad_f[11]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_lower_arm*(x[11]-x[10]);
        x_dx[11]=x[11];

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
            iRow[0]=0; jCol[0]=9;
            iRow[1]=0; jCol[1]=10;
            iRow[2]=0; jCol[2]=11;

            // g[1]
            Ipopt::Index idx=3;
            for (Ipopt::Index col=4; col<n; col++)
            {
                iRow[idx]=1; jCol[idx]=col;
                idx++;
            }
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[12];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw,d_bw;

            // g[0] (lower_arm)
            x_dx[9]=x[9]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            x_dx[9]=x[9]-drho;
            tripod_fkin(2,x_dx,&d_bw);
            values[0]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            x_dx[10]=x[10]-drho;
            tripod_fkin(2,x_dx,&d_bw);
            values[1]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            x_dx[11]=x[11]-drho;
            tripod_fkin(2,x_dx,&d_bw);
            values[2]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[11]=x[11];

            // g[1] (init)
            Vector e=xd-T.getCol(3).subVector(0,2);

            // g[1] (upper_arm)
            Vector grad=-2.0*(J_.submatrix(0,2,0,upper_arm.getDOF()-1).transposed()*e);
            for (size_t i=1; i<grad.length(); i++)
                values[2+i]=grad[i];

            // g[1] (lower_arm)
            Vector e_fw,e_bw;
            Matrix M=d1.T*H;

            x_dx[9]=x[9]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[9]=x[9]-drho;
            d_bw=tripod_fkin(2,x_dx);
            e_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[8]=dot(e,e_fw-e_bw)/drho;
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[10]=x[10]-drho;
            d_bw=tripod_fkin(2,x_dx);
            e_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[9]=dot(e,e_fw-e_bw)/drho;
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[11]=x[11]-drho;
            d_bw=tripod_fkin(2,x_dx);
            e_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[10]=dot(e,e_fw-e_bw)/drho;
            x_dx[11]=x[11];
        }

        return true;
    }
};


