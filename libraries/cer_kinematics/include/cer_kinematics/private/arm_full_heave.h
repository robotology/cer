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
class ArmFullHeaveNLP : public ArmFullNLP
{
public:
    /****************************************************************/
    ArmFullHeaveNLP(ArmParameters &pa, SolverParameters &ps) : ArmFullNLP(pa,ps)
    {
        lambda0.resize(3,0.0);
        lambda=lambda0;
    }

    /****************************************************************/
    string get_mode() const
    {
        return "full+heave";
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

        g_l[0]=torso.cos_alpha_max; g_u[0]=1.0;
        g_l[1]=lower_arm.cos_alpha_max; g_u[1]=1.0;
        g_l[2]=g_u[2]=0.0;

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
        g[0]=d1.n[2];

        TripodState d2=tripod_fkin(2,x);
        g[1]=d2.n[2];

        Matrix T=T0*d1.T*upper_arm.getH(q)*d2.T*TN;
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
            Ipopt::Number x_dx[12];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw,d_bw;

            // g[0] (torso)
            TripodState d1=tripod_fkin(1,x);

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
            TripodState d2=tripod_fkin(2,x);

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
            Vector q(upper_arm.getDOF());
            for (size_t i=0; i<q.length(); i++)
                q[i]=x[3+i];

            Matrix H=upper_arm.getH(q);
            Matrix T=T0*d1.T*H*d2.T*TN;
            Vector e=xd-T.getCol(3).subVector(0,2);
            Vector de_fw,de_bw;
            Matrix M;

            // g[3] (torso)
            M=H*d2.T*TN;

            x_dx[0]=x[0]+drho;
            d_fw=tripod_fkin(1,x_dx);
            de_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            x_dx[0]=x[0]-drho;
            d_bw=tripod_fkin(1,x_dx);
            de_bw=xd-(T0*d_bw.T*M).getCol(3).subVector(0,2);
            values[6]=dot(e,de_fw-de_bw)/drho;
            x_dx[0]=x[0];

            x_dx[1]=x[1]+drho;
            d_fw=tripod_fkin(1,x_dx);
            de_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            x_dx[1]=x[1]-drho;
            d_bw=tripod_fkin(1,x_dx);
            de_bw=xd-(T0*d_bw.T*M).getCol(3).subVector(0,2);
            values[7]=dot(e,de_fw-de_bw)/drho;
            x_dx[1]=x[1];

            x_dx[2]=x[2]+drho;
            d_fw=tripod_fkin(1,x_dx);
            de_fw=xd-(T0*d_fw.T*M).getCol(3).subVector(0,2);
            x_dx[2]=x[2]-drho;
            d_bw=tripod_fkin(1,x_dx);
            de_bw=xd-(T0*d_bw.T*M).getCol(3).subVector(0,2);
            values[8]=dot(e,de_fw-de_bw)/drho;
            x_dx[2]=x[2];

            // g[3] (upper_arm)
            upper_arm.setH0(T0*d1.T*H0); upper_arm.setHN(HN*d2.T*TN);

            Matrix J=upper_arm.GeoJacobian().submatrix(0,2,0,upper_arm.getDOF()-1);
            Vector grad=-2.0*(J.transposed()*e);

            upper_arm.setH0(H0); upper_arm.setHN(HN);

            for (size_t i=0; i<grad.length(); i++)
                values[9+i]=grad[i];

            // g[3] (lower_arm)
            M=T0*d1.T*H;

            x_dx[9]=x[9]+drho;
            d_fw=tripod_fkin(2,x_dx);
            de_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[9]=x[9]-drho;
            d_bw=tripod_fkin(2,x_dx);
            de_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[15]=dot(e,de_fw-de_bw)/drho;
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            d_fw=tripod_fkin(2,x_dx);
            de_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[10]=x[10]-drho;
            d_bw=tripod_fkin(2,x_dx);
            de_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[16]=dot(e,de_fw-de_bw)/drho;
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            d_fw=tripod_fkin(2,x_dx);
            de_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
            x_dx[11]=x[11]-drho;
            d_bw=tripod_fkin(2,x_dx);
            de_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
            values[17]=dot(e,de_fw-de_bw)/drho;
            x_dx[11]=x[11];
        }

        return true;
    }
};


