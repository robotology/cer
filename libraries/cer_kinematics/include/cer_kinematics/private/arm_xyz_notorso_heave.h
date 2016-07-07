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
class ArmXyzNoTorsoHeaveNLP_ForwardDiff : public ArmXyzNoTorsoNoHeaveNLP_ForwardDiff
{
public:
    /****************************************************************/
    ArmXyzNoTorsoHeaveNLP_ForwardDiff(ArmSolver &slv_) :
        ArmXyzNoTorsoNoHeaveNLP_ForwardDiff(slv_)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "xyz_pose+no_torso_heave+forward_diff";
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=x0.length();
        m=1;
        nnz_jac_g=3;
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

        g_l[0]=lower_arm.cos_alpha_max; g_u[0]=1.0;

        latch_idx.push_back(0);
        latch_gl.push_back(g_l[0]);
        latch_gu.push_back(g_u[0]);

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        computeQuantities(x,new_x);

        g[0]=din2.n[2];
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
        }

        return true;
    }
};


/****************************************************************/
class ArmXyzNoTorsoHeaveNLP_CentralDiff : public ArmXyzNoTorsoNoHeaveNLP_CentralDiff
{
public:
    /****************************************************************/
    ArmXyzNoTorsoHeaveNLP_CentralDiff(ArmSolver &slv_) :
        ArmXyzNoTorsoNoHeaveNLP_CentralDiff(slv_)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "xyz_pose+no_torso_heave+central_diff";
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=x0.length();
        m=1;
        nnz_jac_g=3;
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

        g_l[0]=lower_arm.cos_alpha_max; g_u[0]=1.0;

        latch_idx.push_back(0);
        latch_gl.push_back(g_l[0]);
        latch_gu.push_back(g_u[0]);

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        computeQuantities(x,new_x);

        g[0]=din2.n[2];
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
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[12];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw,d_bw;

            // g[0] (lower_arm)
            double e2=hd2-din2.p[2];

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
        }

        return true;
    }
};


