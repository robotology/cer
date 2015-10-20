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
class ArmFullHeaveNLP_ForwardDiff : public ArmFullNLP_ForwardDiff
{
public:
    /****************************************************************/
    ArmFullHeaveNLP_ForwardDiff(ArmParameters &pa, SolverParameters &ps) :
                                ArmFullNLP_ForwardDiff(pa,ps)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "full_pose+heave+forward_diff";
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

        g_l[0]=torso.cos_alpha_max;     g_u[0]=1.0;
        g_l[1]=lower_arm.cos_alpha_max; g_u[1]=1.0;
        g_l[2]=g_u[2]=0.0;

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        computeQuantities(x,new_x);

        g[0]=d1.n[2];
        g[1]=d2.n[2];
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
            // g[0] (torso)
            iRow[0]=0; jCol[0]=0;
            iRow[1]=0; jCol[1]=1;
            iRow[2]=0; jCol[2]=2;

            // g[1] (lower_arm)
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
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[12];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw;

            // g[0] (torso)
            x_dx[0]=x[0]+drho;
            d_fw=tripod_fkin(1,x_dx);
            values[0]=(d_fw.n[2]-d1.n[2])/drho;
            x_dx[0]=x[0];

            x_dx[1]=x[1]+drho;
            d_fw=tripod_fkin(1,x_dx);
            values[1]=(d_fw.n[2]-d1.n[2])/drho;
            x_dx[1]=x[1];

            x_dx[2]=x[2]+drho;
            d_fw=tripod_fkin(1,x_dx);
            values[2]=(d_fw.n[2]-d1.n[2])/drho;
            x_dx[2]=x[2];

            // g[1] (lower_arm)
            x_dx[9]=x[9]+drho;
            d_fw=tripod_fkin(2,x_dx);
            values[3]=(d_fw.n[2]-d2.n[2])/drho;
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            d_fw=tripod_fkin(2,x_dx);
            values[4]=(d_fw.n[2]-d2.n[2])/drho;
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            d_fw=tripod_fkin(2,x_dx);
            values[5]=(d_fw.n[2]-d2.n[2])/drho;
            x_dx[11]=x[11];

            // g[3] (init)
            Vector e=xd-T.getCol(3).subVector(0,2);
            Vector e_fw;
            Matrix M;

            // g[3] (torso)
            M=H*d2.T*TN;

            x_dx[0]=x[0]+drho;
            d_fw=tripod_fkin(1,x_dx);
            e_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            values[6]=2.0*dot(e,e_fw-e)/drho;
            x_dx[0]=x[0];

            x_dx[1]=x[1]+drho;
            d_fw=tripod_fkin(1,x_dx);
            e_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            values[7]=2.0*dot(e,e_fw-e)/drho;
            x_dx[1]=x[1];

            x_dx[2]=x[2]+drho;
            d_fw=tripod_fkin(1,x_dx);
            e_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            values[8]=2.0*dot(e,e_fw-e)/drho;
            x_dx[2]=x[2];

            // g[3] (upper_arm)
            Vector grad=-2.0*(J_.submatrix(0,2,0,upper_arm.getDOF()-1).transposed()*e);
            for (size_t i=0; i<grad.length(); i++)
                values[9+i]=grad[i];

            // g[3] (lower_arm)
            M=T0*d1.T*H;

            x_dx[9]=x[9]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            values[15]=2.0*dot(e,e_fw-e)/drho;
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            values[16]=2.0*dot(e,e_fw-e)/drho;
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            values[17]=2.0*dot(e,e_fw-e)/drho;
            x_dx[11]=x[11];
        }

        return true;
    }
};


/****************************************************************/
class ArmFullHeaveNLP_CentralDiff : public ArmFullNLP_CentralDiff
{
public:
    /****************************************************************/
    ArmFullHeaveNLP_CentralDiff(ArmParameters &pa, SolverParameters &ps) :
                                ArmFullNLP_CentralDiff(pa,ps)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "full_pose+heave+central_diff";
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

        g_l[0]=torso.cos_alpha_max;     g_u[0]=1.0;
        g_l[1]=lower_arm.cos_alpha_max; g_u[1]=1.0;
        g_l[2]=g_u[2]=0.0;

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        computeQuantities(x,new_x);

        g[0]=d1.n[2];
        g[1]=d2.n[2];
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
            // g[0] (torso)
            iRow[0]=0; jCol[0]=0;
            iRow[1]=0; jCol[1]=1;
            iRow[2]=0; jCol[2]=2;

            // g[1] (lower_arm)
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
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[12];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw,d_bw;

            // g[0] (torso)
            x_dx[0]=x[0]+drho;
            d_fw=tripod_fkin(1,x_dx);
            x_dx[0]=x[0]-drho;
            d_bw=tripod_fkin(1,x_dx);
            values[0]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[0]=x[0];

            x_dx[1]=x[1]+drho;
            d_fw=tripod_fkin(1,x_dx);
            x_dx[1]=x[1]-drho;
            d_bw=tripod_fkin(1,x_dx);
            values[1]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[1]=x[1];

            x_dx[2]=x[2]+drho;
            d_fw=tripod_fkin(1,x_dx);
            x_dx[2]=x[2]-drho;
            d_bw=tripod_fkin(1,x_dx);
            values[2]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[2]=x[2];

            // g[1] (lower_arm)
            x_dx[9]=x[9]+drho;
            d_fw=tripod_fkin(2,x_dx);
            x_dx[9]=x[9]-drho;
            d_bw=tripod_fkin(2,x_dx);
            values[3]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            d_fw=tripod_fkin(2,x_dx);
            x_dx[10]=x[10]-drho;
            d_bw=tripod_fkin(2,x_dx);
            values[4]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            d_fw=tripod_fkin(2,x_dx);
            x_dx[11]=x[11]-drho;
            d_bw=tripod_fkin(2,x_dx);
            values[5]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[11]=x[11];

            // g[3] (init)
            Vector e=xd-T.getCol(3).subVector(0,2);
            Vector e_fw,e_bw;
            Matrix M;

            // g[3] (torso)
            M=H*d2.T*TN;

            x_dx[0]=x[0]+drho;
            d_fw=tripod_fkin(1,x_dx);
            e_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            x_dx[0]=x[0]-drho;
            d_bw=tripod_fkin(1,x_dx);
            e_bw=xd-(T0*d_bw.T*M).getCol(3).subVector(0,2);
            values[6]=dot(e,e_fw-e_bw)/drho;
            x_dx[0]=x[0];

            x_dx[1]=x[1]+drho;
            d_fw=tripod_fkin(1,x_dx);
            e_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            x_dx[1]=x[1]-drho;
            d_bw=tripod_fkin(1,x_dx);
            e_bw=xd-(T0*d_bw.T*M).getCol(3).subVector(0,2);
            values[7]=dot(e,e_fw-e_bw)/drho;
            x_dx[1]=x[1];

            x_dx[2]=x[2]+drho;
            d_fw=tripod_fkin(1,x_dx);
            e_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            x_dx[2]=x[2]-drho;
            d_bw=tripod_fkin(1,x_dx);
            e_bw=xd-(T0*d_bw.T*M).getCol(3).subVector(0,2);
            values[8]=dot(e,e_fw-e_bw)/drho;
            x_dx[2]=x[2];

            // g[3] (upper_arm)
            Vector grad=-2.0*(J_.submatrix(0,2,0,upper_arm.getDOF()-1).transposed()*e);
            for (size_t i=0; i<grad.length(); i++)
                values[9+i]=grad[i];

            // g[3] (lower_arm)
            M=T0*d1.T*H;

            x_dx[9]=x[9]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[9]=x[9]-drho;
            d_bw=tripod_fkin(2,x_dx);
            e_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[15]=dot(e,e_fw-e_bw)/drho;
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[10]=x[10]-drho;
            d_bw=tripod_fkin(2,x_dx);
            e_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[16]=dot(e,e_fw-e_bw)/drho;
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            d_fw=tripod_fkin(2,x_dx);
            e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[11]=x[11]-drho;
            d_bw=tripod_fkin(2,x_dx);
            e_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[17]=dot(e,e_fw-e_bw)/drho;
            x_dx[11]=x[11];
        }

        return true;
    }
};


