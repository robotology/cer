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

Vector iExpMap(const Matrix &M, const double &delta_t)
{
    Vector v(6,0.0);
    double theta,si,co,sinc,mcosc,msinc,det;
    Vector u;

    Matrix Rd=M.submatrix(0,2, 0,2);
    u=dcm2axis(Rd);
    u*=u[3];
    u.pop_back();

    for (int i=0; i<3; i++)
        v[3+i]=u[i];

    theta=sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
    si=sin(theta);
    co=cos(theta);
    if (fabs(theta)<1e-8)
        sinc=1.0;
    else
        sinc=si/theta;
    if (fabs(theta) < 2.4e-4)
    {
        mcosc=0.5;
        msinc=1./6.0;
    }
    else
    {
        mcosc=(1.0-co)/(theta*theta);
        msinc=((1.0-si/theta)/theta/theta) ;
    }

    Matrix a(3,3);
    a[0][0] = sinc + u[0]*u[0]*msinc;
    a[0][1] = u[0]*u[1]*msinc - u[2]*mcosc;
    a[0][2] = u[0]*u[2]*msinc + u[1]*mcosc;

    a[1][0] = u[0]*u[1]*msinc + u[2]*mcosc;
    a[1][1] = sinc + u[1]*u[1]*msinc;
    a[1][2] = u[1]*u[2]*msinc - u[0]*mcosc;

    a[2][0] = u[0]*u[2]*msinc - u[1]*mcosc;
    a[2][1] = u[1]*u[2]*msinc + u[0]*mcosc;
    a[2][2] = sinc + u[2]*u[2]*msinc;

    det = a[0][0]*a[1][1]*a[2][2] + a[1][0]*a[2][1]*a[0][2]
         + a[0][1]*a[1][2]*a[2][0] - a[2][0]*a[1][1]*a[0][2]
         - a[1][0]*a[0][1]*a[2][2] - a[0][0]*a[2][1]*a[1][2];

    if (fabs(det) > 1.e-5)
    {
        v[0] =  (M[0][3]*a[1][1]*a[2][2]
                +   M[1][3]*a[2][1]*a[0][2]
                +   M[2][3]*a[0][1]*a[1][2]
                -   M[2][3]*a[1][1]*a[0][2]
                -   M[1][3]*a[0][1]*a[2][2]
                -   M[0][3]*a[2][1]*a[1][2])/det;
        v[1] =  (a[0][0]*M[1][3]*a[2][2]
                +   a[1][0]*M[2][3]*a[0][2]
                +   M[0][3]*a[1][2]*a[2][0]
                -   a[2][0]*M[1][3]*a[0][2]
                -   a[1][0]*M[0][3]*a[2][2]
                -   a[0][0]*M[2][3]*a[1][2])/det;
        v[2] =  (a[0][0]*a[1][1]*M[2][3]
                +   a[1][0]*a[2][1]*M[0][3]
                +   a[0][1]*M[1][3]*a[2][0]
                -   a[2][0]*a[1][1]*M[0][3]
                -   a[1][0]*a[0][1]*M[2][3]
                -   a[0][0]*a[2][1]*M[1][3])/det;
    }
    else
    {
        v[0] = M[0][3];
        v[1] = M[1][3];
        v[2] = M[2][3];
    }

    // Apply the sampling time to the computed velocity
    v /= delta_t;

    return v;
}

/****************************************************************/
class MobileArmFullNoTorsoNoHeaveNLP_ForwardDiff : public MobileArmCommonNLP
{
protected:
    const double s_pos=0.01;
    const double s_ang=0.001;
    const double max_angle=M_PI/4;
public:
    /****************************************************************/
    MobileArmFullNoTorsoNoHeaveNLP_ForwardDiff(MobileArmSolver &slv_, int nb_targets=1) : MobileArmCommonNLP(slv_, nb_targets)
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
        n=x.length();
        m=nb_targets*(2+3+3+1+1);
        nnz_jac_g=nb_targets*(2*3+3*(3+nb_kin_DOF-4)+3*(1+nb_kin_DOF-4)+2+3);
        if(domain_constr)
        {
            m++;
            nnz_jac_g+=2;
        }
        nnz_h_lag=0;
        index_style=TNLP::C_STYLE;

        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        if(n>0)
        {
            for (size_t i=0; i<3; i++)
            {
                x_l[idx_b+i]=std::numeric_limits<double>::lowest();
                x_u[idx_b+i]=std::numeric_limits<double>::max();
            }

            for (size_t i=0; i<nb_targets; i++)
            {
                for (size_t j=0; j<3; j++)
                    x_l[idx_t[i]+j]=x_u[idx_t[i]+j]=x0[idx_t[0]+j];

                x_l[idx_ua[i]+0]=x_u[idx_ua[i]+0]=x0[idx_ua[0]+0];
                iKinChain *chain=upper_arm.asChain();
                for (size_t j=1; j<upper_arm.getDOF(); j++)
                {
                    x_l[idx_ua[i]+j]=(*chain)[j].getMin();
                    x_u[idx_ua[i]+j]=(*chain)[j].getMax();
                }

                for (size_t j=0; j<3; j++)
                {
                    x_l[idx_la[i]+j]=lower_arm.l_min;
                    x_u[idx_la[i]+j]=lower_arm.l_max;
                }
            }
        }

        if(m>0)
        {
            for (Ipopt::Index i=0; i<nb_targets; i++)
            {
                g_l[10*i+0]=g_u[10*i+0]=0.0;

                g_l[10*i+1]=lower_arm.cos_alpha_max;
                g_u[10*i+1]=1.0;

                g_l[10*i+2]=g_u[10*i+2]=0.0;
                g_l[10*i+3]=g_u[10*i+3]=0.0;
                g_l[10*i+4]=g_u[10*i+4]=0.0;

                g_l[10*i+5]=cover_shoulder_avoidance[1];
                g_u[10*i+5]=std::numeric_limits<double>::max();

                g_l[10*i+6]=g_u[10*i+6]=0.0;
                g_l[10*i+7]=g_u[10*i+7]=0.0;
                g_l[10*i+8]=g_u[10*i+8]=0.0;

                g_l[10*i+9]=-max_angle;
                g_u[10*i+9]=max_angle;
            }

            if(domain_constr)
            {
                g_l[10*nb_targets]=0.0;
                g_u[10*nb_targets]=std::numeric_limits<double>::max();
            }

            latch_idx.clear();
            latch_gl.clear();
            latch_gu.clear();

            for (size_t i=0; i<nb_targets; i++)
            {
                latch_idx.push_back(1);
                latch_gl.push_back(g_l[10*i+1]);
                latch_gu.push_back(g_u[10*i+1]);
            }
        }

        return true;
    }

    Vector computeManipulability(Ipopt::Index n, const Ipopt::Number *x, int target_idx, bool new_x)
    {
        // Kinematics Jacobian
        Matrix Jtorso(6,3);
        Matrix Jtripod(6,3);
        Matrix Jarm;

        Ipopt::Number x_dx[n];
        for (Ipopt::Index i=0; i<n; i++)
            x_dx[i]=x[i];

        TripodState d_fw;

        if(new_x)
        {
            // Base

            // Torso

            // Upper arm
            Vector q_loc;
            for (size_t i=0; i<upper_arm.getDOF(); i++)
                q_loc.push_back(x[idx_ua[target_idx]+i]);

            TripodState d1_loc=tripod_fkin(1,x,nullptr,target_idx);
            TripodState d2_loc=tripod_fkin(2,x,nullptr,target_idx);

            upper_arm.setH0(H0);
            upper_arm.setHN(HN);
            Matrix H_loc=upper_arm.getH(q_loc);
            Matrix T_loc=d1_loc.T*H_loc*d2_loc.T*TN;

            upper_arm.setH0(d1_loc.T*H0);
            upper_arm.setHN(HN*d2_loc.T*TN);
            Jarm=upper_arm.GeoJacobian();

            upper_arm.setH0(H0);
            upper_arm.setHN(HN);

            // Lower arm
            Matrix fw;
            Matrix M=d1_loc.T*H_loc;
            Matrix He=M*d2_loc.T*TN;
            Matrix R=T_loc.submatrix(0,2,0,2);

            for(int i=0 ; i<3 ; i++)
            {
                x_dx[idx_la[target_idx]+i]=x[idx_la[target_idx]+i]+drho;
                d_fw = tripod_fkin(2,x_dx,nullptr,target_idx);
                fw = M*d_fw.T*TN;
                Vector w = iExpMap(SE3inv(He)*fw, drho);
                Jtripod.setSubcol(R*w.subVector(0,2), 0,i);
                Jtripod.setSubcol(R*w.subVector(3,5), 3,i);
                x_dx[idx_la[target_idx]+i]=x[idx_la[target_idx]+i];
            }
        }
        else
        {
            // Torso

            // Upper arm
            Jarm = J_[target_idx];

            // Lower arm
            Matrix fw;
            Matrix M=d1[target_idx].T*H[target_idx];
            Matrix He=M*d2[target_idx].T*TN;

            for(int i=0 ; i<3 ; i++)
            {
                x_dx[idx_la[target_idx]+i]=x[idx_la[target_idx]+i]+drho;
                d_fw = tripod_fkin(2,x_dx,nullptr,target_idx);
                fw = M*d_fw.T*TN;
                Vector w = iExpMap(SE3inv(He)*fw, drho);
                Jtripod.setSubcol(T[target_idx].submatrix(0,2,0,2)*w.subVector(0,2), 0,i);
                Jtripod.setSubcol(T[target_idx].submatrix(0,2,0,2)*w.subVector(3,5), 3,i);
                x_dx[idx_la[target_idx]+i]=x[idx_la[target_idx]+i];
            }
        }

        Matrix Jkin = cat(cat(Jtorso,Jarm),Jtripod);

        // Constraints Jacobian

        Matrix Jconstr(2,nb_kin_DOF);

        // torso yaw locked
        Jconstr[0][3]=1.0;

        // equality constraint g[0] (wrist heave fixed)
        if(new_x)
        {
            TripodState din2_loc;
            tripod_fkin(2,x,&din2_loc,target_idx);

            for(int i=0 ; i<3 ; i++)
            {
                x_dx[idx_la[target_idx]+i]=x[idx_la[target_idx]+i]+drho;
                tripod_fkin(2,x_dx,&d_fw,target_idx);
                Jconstr[1][idx_la[0]-3+i]=(d_fw.p[2]-din2_loc.p[2])/drho;
                x_dx[idx_la[target_idx]+i]=x[idx_la[target_idx]+i];
            }
        }
        else
        {
            for(int i=0 ; i<3 ; i++)
            {
                x_dx[idx_la[target_idx]+i]=x[idx_la[target_idx]+i]+drho;
                tripod_fkin(2,x_dx,&d_fw,target_idx);
                Jconstr[1][idx_la[0]-3+i]=(d_fw.p[2]-din2[target_idx].p[2])/drho;
                x_dx[idx_la[target_idx]+i]=x[idx_la[target_idx]+i];
            }
        }

        Matrix Jconstrained = Jkin * (eye(nb_kin_DOF,nb_kin_DOF)-pinv(Jconstr)*Jconstr);

        // Joint limit penalty

        Ipopt::Number x_l[n];
        Ipopt::Number x_u[n];
        get_bounds_info(n, x_l, x_u, -1, nullptr, nullptr);

        Matrix J(6, nb_kin_DOF);
        for(int j=3 ; j<nb_kin_DOF ; j++)
        {
            int idx=idx_t[target_idx]+j;
            double t=x[idx];
            double l=x_l[idx];
            double u=x_u[idx];

            double coef=0.0;
            if(u-l>0 && (u-t)>std::numeric_limits<double>::epsilon() && (t-l)>std::numeric_limits<double>::epsilon())
                coef=1.0/sqrt(1+fabs(0.25*(u-l)*(u-l)*(2*t-u-l)/((u-t)*(u-t)*(t-l)*(t-l))));

            for(int i=0 ; i<6 ; i++)
                J[i][j]=coef*Jconstrained[i][j];
        }

yDebug() << "Jkin\n" << Jkin.toString();
yDebug() << "manip kin" << sqrt(det(Jkin.submatrix(0,2,0,nb_kin_DOF-1)*Jkin.submatrix(0,2,0,nb_kin_DOF-1).transposed())) << sqrt(det(Jkin*Jkin.transposed()));
yDebug() << "Jconstrained\n" << Jconstrained.toString();
yDebug() << "manip constr" << sqrt(det(Jconstrained.submatrix(0,2,0,nb_kin_DOF-1)*Jconstrained.submatrix(0,2,0,nb_kin_DOF-1).transposed())) << sqrt(det(Jconstrained*Jconstrained.transposed()));
yDebug() << "Jlim\n" << J.toString();
yDebug() << "manip lim" << sqrt(det(J.submatrix(0,2,0,nb_kin_DOF-1)*J.submatrix(0,2,0,nb_kin_DOF-1).transposed())) << sqrt(det(J*J.transposed()));

        Vector manip(9);
        double d=det(J*J.transposed());
        manip[0]=(d>std::numeric_limits<double>::epsilon())?sqrt(d):0;
        d=det(J.submatrix(0,2,0,nb_kin_DOF-1)*J.submatrix(0,2,0,nb_kin_DOF-1).transposed());
        manip[1]=(d>std::numeric_limits<double>::epsilon())?sqrt(d):0;
        d=det(J.submatrix(3,5,0,nb_kin_DOF-1)*J.submatrix(3,5,0,nb_kin_DOF-1).transposed());
        manip[2]=(d>std::numeric_limits<double>::epsilon())?sqrt(d):0;

        d=det(Jconstrained*Jconstrained.transposed());
        manip[3]=(d>std::numeric_limits<double>::epsilon())?sqrt(d):0;
        d=det(Jconstrained.submatrix(0,2,0,nb_kin_DOF-1)*Jconstrained.submatrix(0,2,0,nb_kin_DOF-1).transposed());
        manip[4]=(d>std::numeric_limits<double>::epsilon())?sqrt(d):0;
        d=det(Jconstrained.submatrix(3,5,0,nb_kin_DOF-1)*Jconstrained.submatrix(3,5,0,nb_kin_DOF-1).transposed());
        manip[5]=(d>std::numeric_limits<double>::epsilon())?sqrt(d):0;

        d=det(Jkin*Jkin.transposed());
        manip[6]=(d>std::numeric_limits<double>::epsilon())?sqrt(d):0;
        d=det(Jkin.submatrix(0,2,0,nb_kin_DOF-1)*Jkin.submatrix(0,2,0,nb_kin_DOF-1).transposed());
        manip[7]=(d>std::numeric_limits<double>::epsilon())?sqrt(d):0;
        d=det(Jkin.submatrix(3,5,0,nb_kin_DOF-1)*Jkin.submatrix(3,5,0,nb_kin_DOF-1).transposed());
        manip[8]=(d>std::numeric_limits<double>::epsilon())?sqrt(d):0;

        for(int i=0; i<manip.size(); i++)
        {
            if(manip[i]>std::numeric_limits<double>::epsilon())
                manip[i] = 1.0/manip[i];
            else
                manip[i] = 1.0/std::numeric_limits<double>::epsilon();
        }
Matrix U;
Vector S;
Matrix V ;
SVD(J,U,S,V);
yDebug() << "singv" << S.toString();
yDebug() << "cond" << S[0] / S[5];
        //manip[0]=manip[3];
        //manip[0]=S[0] / S[5];
        //manip[0] = manip[3];
        return manip;

    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        computeQuantities(x,new_x);

        Ipopt::Number manipulability=0.0;
        Ipopt::Number postural_upper_arm=0.0;
        Ipopt::Number postural_lower_arm=0.0;
        Ipopt::Number tmp;

        if (wpostural_upper_arm!=0.0)
        {
            for (size_t i=0; i<nb_targets; i++)
            {
                for (size_t j=1; j<upper_arm.getDOF(); j++)
                {
                    tmp=x[idx_ua[i]+j]-xref[idx_ua[0]+j];
                    postural_upper_arm+=tmp*tmp;
                }
            }
        }

        if (wpostural_lower_arm!=0.0)
        {
            for (size_t i=0; i<nb_targets; i++)
            {
                tmp=x[idx_la[i]+0]-x[idx_la[i]+1];
                postural_lower_arm+=tmp*tmp;
                tmp=x[idx_la[i]+1]-x[idx_la[i]+2];
                postural_lower_arm+=tmp*tmp;
            }
        }

        for (size_t i=0; i<nb_targets; i++)
        {
            tmp=computeManipulability(n,x,i, false)[0];
            manipulability+=tmp;
        }

        obj_value=manipulability+
                  wpostural_upper_arm*postural_upper_arm+
                  wpostural_lower_arm*postural_lower_arm;

        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x,new_x);

        // base
        grad_f[idx_b+0]=0;
        grad_f[idx_b+1]=0;
        grad_f[idx_b+2]=0;

        for (size_t i=0; i<nb_targets; i++)
        {
            Ipopt::Number x_dx[n];
            for (Ipopt::Index j=0; j<n; j++)
                x_dx[j]=x[j];

            double manip=computeManipulability(n,x_dx,i,false)[0];

            // base
            for (size_t j=0; j<3; j++)
            {
                x_dx[idx_b+j]=x[idx_b+j]+drho;
                grad_f[idx_b+j]+=(computeManipulability(n,x_dx,i,true)[0]-manip)/drho;
                x_dx[idx_b+j]=x[idx_b+j];
            }

            // torso
            grad_f[idx_t[i]+0]=0.0;
            grad_f[idx_t[i]+1]=0.0;
            grad_f[idx_t[i]+2]=0.0;

            // upper_arm
            grad_f[idx_ua[i]+0]=0.0;
            for (size_t j=1; j<upper_arm.getDOF(); j++)
            {
                grad_f[idx_ua[i]+j]=2.0*wpostural_upper_arm*(x[idx_ua[i]+j]-xref[idx_ua[0]+j]);
                x_dx[idx_ua[i]+j]=x[idx_ua[i]+j]+drho;
                grad_f[idx_ua[i]+j]+=(computeManipulability(n,x_dx,i,true)[0]-manip)/drho;
                x_dx[idx_ua[i]+j]=x[idx_ua[i]+j];
            }

            // lower_arm
            grad_f[idx_la[i]+0]=2.0*wpostural_lower_arm*(x[idx_la[i]+0]-x[idx_la[i]+1]);
            grad_f[idx_la[i]+1]=2.0*wpostural_lower_arm*(2.0*x[idx_la[i]+1]-x[idx_la[i]+0]-x[idx_la[i]+2]);
            grad_f[idx_la[i]+2]=2.0*wpostural_lower_arm*(x[idx_la[i]+2]-x[idx_la[i]+1]);
            for (size_t j=0; j<3; j++)
            {
                x_dx[idx_la[i]+j]=x[idx_la[i]+j]+drho;
                grad_f[idx_la[i]+j]+=(computeManipulability(n,x_dx,i,true)[0]-manip)/drho;
                x_dx[idx_la[i]+j]=x[idx_la[i]+j];
            }
        }

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        computeQuantities(x,new_x);

        for (Ipopt::Index i=0; i<nb_targets; i++)
        {
            double e2=hd2-din2[i].p[2];
            g[10*i+0]=e2*e2;
            g[10*i+1]=din2[i].n[2];

            Vector xe=Hb*Hbt*T[i].getCol(3).subVector(0,3);
            xe.pop_back();
            for(Ipopt::Index j=0; j<3 ;j++)
                g[10*i+2+j]=s_pos*(xd[i][j]-xe[j]);

            g[10*i+5]=-cover_shoulder_avoidance[0]*x[idx_ua[i]+1]+x[idx_ua[i]+2];

            Vector e=dcm2axis(Rd[i]*(Rb*T[i]).transposed());
            e*=s_ang*e[3];
            for(Ipopt::Index j=0; j<3 ;j++)
                g[10*i+6+j]=e[j];

            g[10*i+9]=remainder(atan2(xd[i][1]-x[idx_b+1], xd[i][0]-x[idx_b+0])-x[idx_b+2], 2.0*M_PI);
        }

        if(domain_constr)
            g[10*nb_targets] = domain_dist;

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
            Ipopt::Index idx=0;
            for (size_t i=0; i<nb_targets; i++)
            {
                // g[0,1] (lower_arm)
                iRow[idx]=10*i+0; jCol[idx]=idx_la[i]+0;idx++;
                iRow[idx]=10*i+1; jCol[idx]=idx_la[i]+0;idx++;
                iRow[idx]=10*i+0; jCol[idx]=idx_la[i]+1;idx++;
                iRow[idx]=10*i+1; jCol[idx]=idx_la[i]+1;idx++;
                iRow[idx]=10*i+0; jCol[idx]=idx_la[i]+2;idx++;
                iRow[idx]=10*i+1; jCol[idx]=idx_la[i]+2;idx++;

                // g[2] (reaching position)
                for (Ipopt::Index col=0; col<3; col++)
                {
                    for (Ipopt::Index j=0; j<3; j++)
                    {
                        iRow[idx]=10*i+2+j; jCol[idx]=col;
                        idx++;
                    }
                }
                for (Ipopt::Index col=idx_ua[i]+1; col<idx_la[i]+3; col++)
                {
                    for (Ipopt::Index j=0; j<3; j++)
                    {
                        iRow[idx]=10*i+2+j; jCol[idx]=col;
                        idx++;
                    }
                }

                // g[3] (cover constraints)
                iRow[idx]=10*i+5; jCol[idx]=idx_ua[i]+1;idx++;
                iRow[idx]=10*i+5; jCol[idx]=idx_ua[i]+2;idx++;

                // g[4] (reaching orientation)
                for (Ipopt::Index j=0; j<3; j++)
                {
                    iRow[idx]=10*i+6+j; jCol[idx]=2;
                    idx++;
                }
                for (Ipopt::Index col=idx_ua[i]+1; col<idx_la[i]+3; col++)
                {
                    for (Ipopt::Index j=0; j<3; j++)
                    {
                        iRow[idx]=10*i+6+j; jCol[idx]=col;
                        idx++;
                    }
                }

                // g[5] (objects in front)
                for (Ipopt::Index j=0; j<3; j++)
                {
                    iRow[idx]=10*i+9; jCol[idx]=j;
                    idx++;
                }
            }

            // g[5] (domain boundaries constraints)
            if(domain_constr)
            {
                iRow[idx]=10*nb_targets; jCol[idx]=0;idx++;
                iRow[idx]=10*nb_targets; jCol[idx]=1;
            }
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[n];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw;

            Ipopt::Index idx=0;
            for (size_t i=0; i<nb_targets; i++)
            {
                // g[0,1] (lower_arm)
                double e2=hd2-din2[i].p[2];

                x_dx[idx_la[i]+0]=x[idx_la[i]+0]+drho;
                tripod_fkin(2,x_dx,&d_fw,i);
                values[idx]=-2.0*e2*(d_fw.p[2]-din2[i].p[2])/drho;idx++;
                values[idx]=(d_fw.n[2]-din2[i].n[2])/drho;idx++;
                x_dx[idx_la[i]+0]=x[idx_la[i]+0];

                x_dx[idx_la[i]+1]=x[idx_la[i]+1]+drho;
                tripod_fkin(2,x_dx,&d_fw,i);
                values[idx]=-2.0*e2*(d_fw.p[2]-din2[i].p[2])/drho;idx++;
                values[idx]=(d_fw.n[2]-din2[i].n[2])/drho;idx++;
                x_dx[idx_la[i]+1]=x[idx_la[i]+1];

                x_dx[idx_la[i]+2]=x[idx_la[i]+2]+drho;
                tripod_fkin(2,x_dx,&d_fw,i);
                values[idx]=-2.0*e2*(d_fw.p[2]-din2[i].p[2])/drho;idx++;
                values[idx]=(d_fw.n[2]-din2[i].n[2])/drho;idx++;
                x_dx[idx_la[i]+2]=x[idx_la[i]+2];

                // g[2] (base)
                values[idx]=-s_pos;idx++;
                values[idx]=0.0;idx++;
                values[idx]=0.0;idx++;

                values[idx]=0.0;idx++;
                values[idx]=-s_pos;idx++;
                values[idx]=0.0;idx++;

                Vector o(4,0.0);
                o[2] = 1.0;
                o[3] = M_PI/2.0+x[idx_b+2];
                Matrix Ro=axis2dcm(o);
                Vector v=Ro*Hbt*T[i].getCol(3).subVector(0,3);

                values[idx]=-s_pos*v[0];idx++;
                values[idx]=-s_pos*v[1];idx++;
                values[idx]=0.0;idx++;

                // g[2] (upper_arm)
                Matrix grad=Hb.submatrix(0,2,0,2)*J_[i].submatrix(0,2,0,upper_arm.getDOF()-1);
                for (size_t j=1; j<grad.cols(); j++)
                {
                    for (size_t k=0; k<3; k++)
                    {
                        values[idx]=-s_pos*grad[k][j];
                        idx++;
                    }
                }

                // g[2] (lower_arm)
                Vector xe=Hb*Hbt*T[i].getCol(3).subVector(0,3);
                xe.pop_back();

                Vector e=xd[i]-xe;
                Vector e_fw;
                Matrix M=Hb*Hbt*d1[i].T*H[i];

                x_dx[idx_la[i]+0]=x[idx_la[i]+0]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fw=xd[i]-(M*d_fw.T*TN).getCol(3).subVector(0,2);
                values[idx]=s_pos*(e_fw[0]-e[0])/drho;idx++;
                values[idx]=s_pos*(e_fw[1]-e[1])/drho;idx++;
                values[idx]=s_pos*(e_fw[2]-e[2])/drho;idx++;
                x_dx[idx_la[i]+0]=x[idx_la[i]+0];

                x_dx[idx_la[i]+1]=x[idx_la[i]+1]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fw=xd[i]-(M*d_fw.T*TN).getCol(3).subVector(0,2);
                values[idx]=s_pos*(e_fw[0]-e[0])/drho;idx++;
                values[idx]=s_pos*(e_fw[1]-e[1])/drho;idx++;
                values[idx]=s_pos*(e_fw[2]-e[2])/drho;idx++;
                x_dx[idx_la[i]+1]=x[idx_la[i]+1];

                x_dx[idx_la[i]+2]=x[idx_la[i]+2]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fw=xd[i]-(M*d_fw.T*TN).getCol(3).subVector(0,2);
                values[idx]=s_pos*(e_fw[0]-e[0])/drho;idx++;
                values[idx]=s_pos*(e_fw[1]-e[1])/drho;idx++;
                values[idx]=s_pos*(e_fw[2]-e[2])/drho;idx++;
                x_dx[idx_la[i]+2]=x[idx_la[i]+2];

                // g[3]
                values[idx]=-cover_shoulder_avoidance[0];idx++;
                values[idx]=1.0;idx++;

                // g[4] init
                Vector eo=dcm2axis(Rd[i]*(Rb*T[i]).transposed());
                double theta=eo[3];
                double coef=0.5*theta*cos(theta/2)/sin(theta/2);
                Vector eo_u=eo.subVector(0,2);
                eo*=theta;eo.pop_back();

                // g[4] base
                Vector w(3,0.0);
                w[2]=1.0;
                Vector eo_d=dot(w,eo_u)*eo_u;
                Vector grado=s_ang*(-1*eo_d+coef*(eo_d-w)+0.5*theta*cross(w,eo_u));
                values[idx]=grado[0];idx++;
                values[idx]=grado[1];idx++;
                values[idx]=grado[2];idx++;

                // g[4] (upper_arm)
                Matrix grado_ua=Rb.submatrix(0,2,0,2)*J_[i].submatrix(3,5,0,upper_arm.getDOF()-1);
                for (size_t j=1; j<grado_ua.cols(); j++)
                {
                    w=grado_ua.getCol(j);
                    eo_d=dot(w,eo_u)*eo_u;
                    grado=s_ang*(-1*eo_d+coef*(eo_d-w)+0.5*theta*cross(w,eo_u));
                    values[idx]=grado[0];idx++;
                    values[idx]=grado[1];idx++;
                    values[idx]=grado[2];idx++;
                }

                // g[4] (lower_arm)
                Vector e_fwo;

                x_dx[idx_la[i]+0]=x[idx_la[i]+0]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fwo=dcm2axis(Rd[i]*(M*d_fw.T*TN).transposed()); e_fwo*=e_fwo[3]; e_fwo.pop_back();
                values[idx]=s_ang*(e_fwo[0]-eo[0])/drho;idx++;
                values[idx]=s_ang*(e_fwo[1]-eo[1])/drho;idx++;
                values[idx]=s_ang*(e_fwo[2]-eo[2])/drho;idx++;
                x_dx[idx_la[i]+0]=x[idx_la[i]+0];

                x_dx[idx_la[i]+1]=x[idx_la[i]+1]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fwo=dcm2axis(Rd[i]*(M*d_fw.T*TN).transposed()); e_fwo*=e_fwo[3]; e_fwo.pop_back();
                values[idx]=s_ang*(e_fwo[0]-eo[0])/drho;idx++;
                values[idx]=s_ang*(e_fwo[1]-eo[1])/drho;idx++;
                values[idx]=s_ang*(e_fwo[2]-eo[2])/drho;idx++;
                x_dx[idx_la[i]+1]=x[idx_la[i]+1];

                x_dx[idx_la[i]+2]=x[idx_la[i]+2]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fwo=dcm2axis(Rd[i]*(M*d_fw.T*TN).transposed()); e_fwo*=e_fwo[3]; e_fwo.pop_back();
                values[idx]=s_ang*(e_fwo[0]-eo[0])/drho;idx++;
                values[idx]=s_ang*(e_fwo[1]-eo[1])/drho;idx++;
                values[idx]=s_ang*(e_fwo[2]-eo[2])/drho;idx++;
                x_dx[idx_la[i]+2]=x[idx_la[i]+2];

                // g[5] (object in front)
                double dx=xd[i][0]-x[idx_b+0];
                double dy=xd[i][1]-x[idx_b+1];
                double norm = dx*dx+dy*dy;
                if (norm < std::numeric_limits<double>::epsilon())
                {
                    values[idx]=0;idx++;
                    values[idx]=0;idx++;
                    values[idx]=0;idx++;
                }
                else
                {
                    values[idx]=dy/norm;idx++;
                    values[idx]=-dx/norm;idx++;
                    values[idx]=-1.0;idx++;
                }
            }

            // g[5] (domain boundaries constraints)
            if(domain_constr)
            {
                Vector v(2);
                v[0]=x[idx_b+0]+drho;
                v[1]=x[idx_b+1];
                double d = distanceFromDomain(domain_poly, v);
                values[idx]=(d-domain_dist)/drho;idx++;

                v[0]=x[idx_b+0];
                v[1]=x[idx_b+1]+drho;
                d = distanceFromDomain(domain_poly, v);
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
    MobileArmFullNoTorsoNoHeaveNLP_CentralDiff(MobileArmSolver &slv_, int nb_targets=1) :
        MobileArmFullNoTorsoNoHeaveNLP_ForwardDiff(slv_, nb_targets)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "full_pose+mobile_base+no_torso_no_heave+central_diff";
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        if (values==NULL)
        {
            Ipopt::Index idx=0;
            for (size_t i=0; i<nb_targets; i++)
            {
                // g[0,1] (lower_arm)
                iRow[idx]=10*i+0; jCol[idx]=idx_la[i]+0;idx++;
                iRow[idx]=10*i+1; jCol[idx]=idx_la[i]+0;idx++;
                iRow[idx]=10*i+0; jCol[idx]=idx_la[i]+1;idx++;
                iRow[idx]=10*i+1; jCol[idx]=idx_la[i]+1;idx++;
                iRow[idx]=10*i+0; jCol[idx]=idx_la[i]+2;idx++;
                iRow[idx]=10*i+1; jCol[idx]=idx_la[i]+2;idx++;

                // g[2] (reaching position)
                for (Ipopt::Index col=0; col<3; col++)
                {
                    for (Ipopt::Index j=0; j<3; j++)
                    {
                        iRow[idx]=10*i+2+j; jCol[idx]=col;
                        idx++;
                    }
                }
                for (Ipopt::Index col=idx_ua[i]+1; col<idx_la[i]+3; col++)
                {
                    for (Ipopt::Index j=0; j<3; j++)
                    {
                        iRow[idx]=10*i+2+j; jCol[idx]=col;
                        idx++;
                    }
                }

                // g[3] (cover constraints)
                iRow[idx]=10*i+5; jCol[idx]=idx_ua[i]+1;idx++;
                iRow[idx]=10*i+5; jCol[idx]=idx_ua[i]+2;idx++;

                // g[4] (reaching orientation)
                for (Ipopt::Index j=0; j<3; j++)
                {
                    iRow[idx]=10*i+6+j; jCol[idx]=2;
                    idx++;
                }
                for (Ipopt::Index col=idx_ua[i]+1; col<idx_la[i]+3; col++)
                {
                    for (Ipopt::Index j=0; j<3; j++)
                    {
                        iRow[idx]=10*i+6+j; jCol[idx]=col;
                        idx++;
                    }
                }
            }

            // g[5] (domain boundaries constraints)
            if(domain_constr)
            {
                iRow[idx]=10*nb_targets; jCol[idx]=0;idx++;
                iRow[idx]=10*nb_targets; jCol[idx]=1;
            }
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[n];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw,d_bw;

            Ipopt::Index idx=0;
            for (size_t i=0; i<nb_targets; i++)
            {
                // g[0,1] (lower_arm)
                double e2=hd2-din2[i].p[2];

                x_dx[idx_la[i]+0]=x[idx_la[i]+0]+drho;
                tripod_fkin(2,x_dx,&d_fw,i);
                x_dx[idx_la[i]+0]=x[idx_la[i]+0]-drho;
                tripod_fkin(2,x_dx,&d_bw,i);
                values[idx]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;idx++;
                values[idx]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);idx++;
                x_dx[idx_la[i]+0]=x[idx_la[i]+0];

                x_dx[idx_la[i]+1]=x[idx_la[i]+1]+drho;
                tripod_fkin(2,x_dx,&d_fw,i);
                x_dx[idx_la[i]+1]=x[idx_la[i]+1]-drho;
                tripod_fkin(2,x_dx,&d_bw,i);
                values[idx]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;idx++;
                values[idx]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);idx++;
                x_dx[idx_la[i]+1]=x[idx_la[i]+1];

                x_dx[idx_la[i]+2]=x[idx_la[i]+2]+drho;
                tripod_fkin(2,x_dx,&d_fw,i);
                x_dx[idx_la[i]+2]=x[idx_la[i]+2]-drho;
                tripod_fkin(2,x_dx,&d_bw,i);
                values[idx]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;idx++;
                values[idx]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);idx++;
                x_dx[idx_la[i]+2]=x[idx_la[i]+2];

                // g[2] (base)
                values[idx]=-s_pos;idx++;
                values[idx]=0.0;idx++;
                values[idx]=0.0;idx++;

                values[idx]=0.0;idx++;
                values[idx]=-s_pos;idx++;
                values[idx]=0.0;idx++;

                Vector o(4,0.0);
                o[2] = 1.0;
                o[3] = M_PI/2.0+x[idx_b+2];
                Matrix Ro=axis2dcm(o);
                Vector v=Ro*T[i].getCol(3).subVector(0,3);

                values[idx]=-s_pos*v[0];idx++;
                values[idx]=-s_pos*v[1];idx++;
                values[idx]=0.0;idx++;

                // g[2] (upper_arm)
                Matrix grad=Hb.submatrix(0,2,0,2)*J_[i].submatrix(0,2,0,upper_arm.getDOF()-1);
                for (size_t j=1; j<grad.cols(); j++)
                {
                    for (size_t k=0; k<3; k++)
                    {
                        values[idx]=-s_pos*grad[k][j];
                        idx++;
                    }
                }

                // g[2] (lower_arm)
                Vector xe=Hb*T[i].getCol(3).subVector(0,3);
                xe.pop_back();
                Vector e=xd[i]-xe;
                Vector e_fw,e_bw;
                Matrix M=d1[i].T*H[i];

                x_dx[idx_la[i]+0]=x[idx_la[i]+0]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fw=xd[i]-(Hb*M*d_fw.T*TN).getCol(3).subVector(0,2);
                x_dx[idx_la[i]+0]=x[idx_la[i]+0]-drho;
                d_bw=tripod_fkin(2,x_dx,nullptr,i);
                e_bw=xd[i]-(Hb*M*d_bw.T*TN).getCol(3).subVector(0,2);
                values[idx]=0.5*s_pos*(e_fw[0]-e_bw[0])/drho;idx++;
                values[idx]=0.5*s_pos*(e_fw[1]-e_bw[1])/drho;idx++;
                values[idx]=0.5*s_pos*(e_fw[2]-e_bw[2])/drho;idx++;
                x_dx[idx_la[i]+0]=x[idx_la[i]+0];

                x_dx[idx_la[i]+1]=x[idx_la[i]+1]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fw=xd[i]-(Hb*M*d_fw.T*TN).getCol(3).subVector(0,2);
                x_dx[idx_la[i]+1]=x[idx_la[i]+1]-drho;
                d_bw=tripod_fkin(2,x_dx,nullptr,i);
                e_bw=xd[i]-(Hb*M*d_bw.T*TN).getCol(3).subVector(0,2);
                values[idx]=0.5*s_pos*(e_fw[0]-e_bw[0])/drho;idx++;
                values[idx]=0.5*s_pos*(e_fw[1]-e_bw[1])/drho;idx++;
                values[idx]=0.5*s_pos*(e_fw[2]-e_bw[2])/drho;idx++;
                x_dx[idx_la[i]+1]=x[idx_la[i]+1];

                x_dx[idx_la[i]+2]=x[idx_la[i]+2]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fw=xd[i]-(Hb*M*d_fw.T*TN).getCol(3).subVector(0,2);
                x_dx[idx_la[i]+2]=x[idx_la[i]+2]-drho;
                d_bw=tripod_fkin(2,x_dx,nullptr,i);
                e_bw=xd[i]-(Hb*M*d_bw.T*TN).getCol(3).subVector(0,2);
                values[idx]=0.5*s_pos*(e_fw[0]-e_bw[0])/drho;idx++;
                values[idx]=0.5*s_pos*(e_fw[1]-e_bw[1])/drho;idx++;
                values[idx]=0.5*s_pos*(e_fw[2]-e_bw[2])/drho;idx++;
                x_dx[idx_la[i]+2]=x[idx_la[i]+2];

                // g[3]
                values[idx]=-cover_shoulder_avoidance[0];idx++;
                values[idx]=1.0;idx++;

                // g[4] init
                Vector eo=dcm2axis(Rd[i]*(Rb*T[i]).transposed());
                double theta=eo[3];
                double coef=0.5*theta*cos(theta/2)/sin(theta/2);
                Vector eo_u=eo.subVector(0,2);
                eo*=theta;eo.pop_back();

                // g[4] base
                Vector w(3,0.0);
                w[2]=1.0;
                Vector eo_d=dot(w,eo_u)*eo_u;
                Vector grado=s_ang*(-1*eo_d+coef*(eo_d-w)+0.5*theta*cross(w,eo_u));
                values[idx]=grado[0];idx++;
                values[idx]=grado[1];idx++;
                values[idx]=grado[2];idx++;

                // g[4] (upper_arm)
                Matrix grado_ua=Rb.submatrix(0,2,0,2)*J_[i].submatrix(3,5,0,upper_arm.getDOF()-1);
                for (size_t j=1; j<grado_ua.cols(); j++)
                {
                    w=grado_ua.getCol(j);
                    eo_d=dot(w,eo_u)*eo_u;
                    grado=s_ang*(-1*eo_d+coef*(eo_d-w)+0.5*theta*cross(w,eo_u));
                    values[idx]=grado[0];idx++;
                    values[idx]=grado[1];idx++;
                    values[idx]=grado[2];idx++;
                }

                // g[4] (lower_arm)
                Vector e_fwo, e_bwo;

                x_dx[idx_la[i]+0]=x[idx_la[i]+0]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fwo=dcm2axis(Rd[i]*(Rb*M*d_fw.T*TN).transposed()); e_fwo*=e_fwo[3]; e_fwo.pop_back();
                x_dx[idx_la[i]+0]=x[idx_la[i]+0]-drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_bwo=dcm2axis(Rd[i]*(Rb*M*d_fw.T*TN).transposed()); e_bwo*=e_bwo[3]; e_bwo.pop_back();
                values[idx]=s_ang*0.5*(e_fwo[0]-e_bwo[0])/drho;idx++;
                values[idx]=s_ang*0.5*(e_fwo[1]-e_bwo[1])/drho;idx++;
                values[idx]=s_ang*0.5*(e_fwo[2]-e_bwo[2])/drho;idx++;
                x_dx[idx_la[i]+0]=x[idx_la[i]+0];

                x_dx[idx_la[i]+1]=x[idx_la[i]+1]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fwo=dcm2axis(Rd[i]*(Rb*M*d_fw.T*TN).transposed()); e_fwo*=e_fwo[3]; e_fwo.pop_back();
                x_dx[idx_la[i]+1]=x[idx_la[i]+1]-drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_bwo=dcm2axis(Rd[i]*(Rb*M*d_fw.T*TN).transposed()); e_bwo*=e_bwo[3]; e_bwo.pop_back();
                values[idx]=s_ang*0.5*(e_fwo[0]-e_bwo[0])/drho;idx++;
                values[idx]=s_ang*0.5*(e_fwo[1]-e_bwo[1])/drho;idx++;
                values[idx]=s_ang*0.5*(e_fwo[2]-e_bwo[2])/drho;idx++;
                x_dx[idx_la[i]+1]=x[idx_la[i]+1];

                x_dx[idx_la[i]+2]=x[idx_la[i]+2]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fwo=dcm2axis(Rd[i]*(Rb*M*d_fw.T*TN).transposed()); e_fwo*=e_fwo[3]; e_fwo.pop_back();
                x_dx[idx_la[i]+2]=x[idx_la[i]+2]-drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_bwo=dcm2axis(Rd[i]*(Rb*M*d_fw.T*TN).transposed()); e_bwo*=e_bwo[3]; e_bwo.pop_back();
                values[idx]=s_ang*0.5*(e_fwo[0]-e_bwo[0])/drho;idx++;
                values[idx]=s_ang*0.5*(e_fwo[1]-e_bwo[1])/drho;idx++;
                values[idx]=s_ang*0.5*(e_fwo[2]-e_bwo[2])/drho;idx++;
                x_dx[idx_la[i]+2]=x[idx_la[i]+2];
            }

            // g[5] (domain boundaries constraints)
            if(domain_constr)
            {
                Vector v(2);
                v[0]=x[idx_b+0]+drho;
                v[1]=x[idx_b+1];
                double df = distanceFromDomain(domain_poly, v);
                v[0]=x[idx_b+0]-drho;
                double db = distanceFromDomain(domain_poly, v);
                values[idx]=0.5*(df-db)/drho;idx++;

                v[0]=x[idx_b+0];
                v[1]=x[idx_b+1]+drho;
                df = distanceFromDomain(domain_poly, v);
                v[1]=x[idx_b+1]-drho;
                db = distanceFromDomain(domain_poly, v);
                values[idx]=0.5*(df-db)/drho;
            }
        }

        return true;
    }
};


