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
class ArmXyzNoTorsoNoHeaveNLP_ForwardDiff : public ArmCommonNLP
{
public:
    /****************************************************************/
    ArmXyzNoTorsoNoHeaveNLP_ForwardDiff(ArmSolver &slv_) : ArmCommonNLP(slv_)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "xyz_pose+no_torso_no_heave+forward_diff";
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=x0.length();
        m=2+1;
        nnz_jac_g=6+2;
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

        latch_idx.clear();
        latch_gl.clear();
        latch_gu.clear();

        g_l[0]=g_u[0]=0.0;
        g_l[1]=lower_arm.cos_alpha_max;     g_u[1]=1.0;
        g_l[2]=cover_shoulder_avoidance[1]; g_u[2]=std::numeric_limits<double>::max();

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

        obj_value=norm2(xd-T.getCol(3).subVector(0,2))+
                  wpostural_upper_arm*postural_upper_arm+
                  wpostural_lower_arm*postural_lower_arm;

        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x,new_x);

        Vector e=xd-T.getCol(3).subVector(0,2);

        Ipopt::Number x_dx[12];
        for (Ipopt::Index i=0; i<n; i++)
            x_dx[i]=x[i];

        // torso
        grad_f[0]=0.0;
        grad_f[1]=0.0;
        grad_f[2]=0.0;
        grad_f[3]=0.0;

        // (upper_arm)
        Vector grad=-2.0*(J_.submatrix(0,2,0,upper_arm.getDOF()-1).transposed()*e);
        for (size_t i=1; i<grad.length(); i++)
            grad_f[3+i]=grad[i] + 2.0*wpostural_upper_arm*(x[3+i]-x0[3+i]);

        // (lower_arm)
        TripodState d_fw;
        Vector e_fw;
        Matrix M=d1.T*H;

        x_dx[9]=x[9]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
        grad_f[9]=2.0*(dot(e,e_fw-e)/drho + wpostural_lower_arm*(x[9]-x[10]));
        x_dx[9]=x[9];

        x_dx[10]=x[10]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
        grad_f[10]=2.0*(dot(e,e_fw-e)/drho + wpostural_lower_arm*(2.0*x[10]-x[9]-x[11]));
        x_dx[10]=x[10];

        x_dx[11]=x[11]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
        grad_f[11]=2.0*(dot(e,e_fw-e)/drho + wpostural_lower_arm*(x[11]-x[10]));
        x_dx[11]=x[11];

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
        g[2]=-cover_shoulder_avoidance[0]*x[4]+x[5];

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

            // g[1] (lower_arm)
            iRow[3]=1; jCol[3]=9;
            iRow[4]=1; jCol[4]=10;
            iRow[5]=1; jCol[5]=11;

            // g[2] (cover constraints)
            iRow[6]=2; jCol[6]=4;
            iRow[7]=2; jCol[7]=5;
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[12];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw;

            // g[0,1] (lower_arm)
            double e2=hd2-din2.p[2];

            x_dx[9]=x[9]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            values[0]=-2.0*e2*(d_fw.p[2]-din2.p[2])/drho;
            values[3]=(d_fw.n[2]-din2.n[2])/drho;
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            values[1]=-2.0*e2*(d_fw.p[2]-din2.p[2])/drho;
            values[4]=(d_fw.n[2]-din2.n[2])/drho;
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            values[2]=-2.0*e2*(d_fw.p[2]-din2.p[2])/drho;
            values[5]=(d_fw.n[2]-din2.n[2])/drho;
            x_dx[11]=x[11];

            // g[2]
            values[6]=-cover_shoulder_avoidance[0];
            values[7]=1.0;
        }

        return true;
    }
};


/****************************************************************/
class ArmXyzNoTorsoNoHeaveNLP_CentralDiff : public ArmXyzNoTorsoNoHeaveNLP_ForwardDiff
{
public:
    /****************************************************************/
    ArmXyzNoTorsoNoHeaveNLP_CentralDiff(ArmSolver &slv_) :
        ArmXyzNoTorsoNoHeaveNLP_ForwardDiff(slv_)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "xyz_pose+no_torso_no_heave+central_diff";
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x,new_x);

        Vector e=xd-T.getCol(3).subVector(0,2);

        Ipopt::Number x_dx[12];
        for (Ipopt::Index i=0; i<n; i++)
            x_dx[i]=x[i];

        // torso
        grad_f[0]=0.0;
        grad_f[1]=0.0;
        grad_f[2]=0.0;
        grad_f[3]=0.0;

        // (upper_arm)
        Vector grad=-2.0*(J_.submatrix(0,2,0,upper_arm.getDOF()-1).transposed()*e);
        for (size_t i=1; i<grad.length(); i++)
            grad_f[3+i]=grad[i] + 2.0*wpostural_upper_arm*(x[3+i]-x0[3+i]);

        // (lower_arm)
        TripodState d_fw,d_bw;
        Vector e_fw,e_bw;
        Matrix M=d1.T*H;

        x_dx[9]=x[9]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
        x_dx[9]=x[9]-drho;
        d_bw=tripod_fkin(2,x_dx);
        e_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
        grad_f[9]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_lower_arm*(x[9]-x[10]);
        x_dx[9]=x[9];

        x_dx[10]=x[10]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
        x_dx[10]=x[10]-drho;
        d_bw=tripod_fkin(2,x_dx);
        e_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
        grad_f[10]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_lower_arm*(2.0*x[10]-x[9]-x[11]);
        x_dx[10]=x[10];

        x_dx[11]=x[11]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
        x_dx[11]=x[11]-drho;
        d_bw=tripod_fkin(2,x_dx);
        e_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
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

            // g[1] (lower_arm)
            iRow[3]=1; jCol[3]=9;
            iRow[4]=1; jCol[4]=10;
            iRow[5]=1; jCol[5]=11;

            // g[2] (cover constraints)
            iRow[6]=2; jCol[6]=4;
            iRow[7]=2; jCol[7]=5;
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[12];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw,d_bw;

            // g[0,1] (lower_arm)
            double e2=hd2-din2.p[2];

            x_dx[9]=x[9]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            x_dx[9]=x[9]-drho;
            tripod_fkin(2,x_dx,&d_bw);
            values[0]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;
            values[3]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            x_dx[10]=x[10]-drho;
            tripod_fkin(2,x_dx,&d_bw);
            values[1]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;
            values[4]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            x_dx[11]=x[11]-drho;
            tripod_fkin(2,x_dx,&d_bw);
            values[2]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;
            values[5]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[11]=x[11];

            // g[2]
            values[6]=-cover_shoulder_avoidance[0];
            values[7]=1.0;
        }

        return true;
    }
};


