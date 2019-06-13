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
class MobileArmCommonNLP : public TripodNLPHelper,
                     public Ipopt::TNLP
{
protected:
    MobileArmSolver &slv;
    const TripodParametersExtended torso;
    const TripodParametersExtended lower_arm;
    iKinLimb &upper_arm;

    bool domain_constr;
    vector<Vector> domain_poly;
    double domain_dist;

    double drho;

    Matrix TN;
    double hd1,hd2;
    double wpostural_torso;
    double wpostural_torso_yaw;
    double wpostural_upper_arm;
    double wpostural_lower_arm;

    int nb_targets;
    int nb_kin_DOF;
    Matrix Hb,Rb,H0,HN;
    vector<Matrix> Hd,Rd;
    vector<Vector> xd;
    Vector x0,x,xref;

    int idx_b;
    vector<int> idx_t,idx_ua,idx_la;

    Vector latch_x;
    vector<int> latch_idx;
    Vector latch_gl,latch_gu;

    Vector zL,zU;
    Vector lambda;

    vector<TripodState> d1,d2;
    vector<TripodState> din1,din2;
    vector<Matrix> H,H_,J_,T;
    vector<Vector> q;

    Vector cover_shoulder_avoidance;

    /****************************************************************/
    TripodState tripod_fkin(const int which, const Ipopt::Number *x,
                            TripodState *internal=NULL, const int target_idx=0)
    {
        const TripodParametersExtended &params=((which==1)?torso:lower_arm);
        int offs=(which==1)?idx_t[target_idx]:idx_la[target_idx];

        return fkinHelper(&x[offs],params,internal);
    }

    /****************************************************************/
    bool verify_alpha(const Ipopt::Number *x, const Ipopt::Number *g)
    {
        if (latch_idx.size()==0)
            return false;

        for (size_t i=0; i<latch_idx.size(); i++)
        {
            int idx=latch_idx[i];
            if ((g[idx]<latch_gl[i]) || (g[idx]>latch_gu[i]))
                return false;
        }

        return true;
    }

    /****************************************************************/
    void latch_x_verifying_alpha(Ipopt::Index n, const Ipopt::Number *x,
                                 const Ipopt::Number *g)
    {
        if (verify_alpha(x,g))
        {
            if (latch_x.length()==0)
                latch_x.resize(n);

            for (Ipopt::Index i=0; i<n; i++)
                latch_x[i]=x[i];
        }        
    }

    /****************************************************************/
    Vector verify_solution_against_alpha(Ipopt::Index n, const Ipopt::Number *x,
                                         Ipopt::Index m)
    {
        Vector solution(n);
        for (Ipopt::Index i=0; i<n; i++)
            solution[i]=x[i];

        if (latch_idx.size()>0)
        {
            Vector g_data(m);
            Ipopt::Number *g=(Ipopt::Number*)g_data.data();

            eval_g(n,x,true,m,g);
            if (!verify_alpha(x,g) && (latch_x.length()>0))
                solution=latch_x;
        }

        return solution;
    }

public:
    /****************************************************************/
    MobileArmCommonNLP(MobileArmSolver &slv_, int nb_targets_=1) :
                 slv(slv_), torso(slv_.armParameters.torso),
                 upper_arm(slv_.armParameters.upper_arm),
                 lower_arm(slv_.armParameters.lower_arm),
                 domain_constr(false),
                 TN(slv_.armParameters.TN),
                 hd1(slv_.slvParameters.torso_heave),
                 hd2(slv_.slvParameters.lower_arm_heave),
                 wpostural_torso(slv_.slvParameters.weight_postural_torso),
                 wpostural_torso_yaw(slv_.slvParameters.weight_postural_torso_yaw),
                 wpostural_upper_arm(slv_.slvParameters.weight_postural_upper_arm),
                 wpostural_lower_arm(slv_.slvParameters.weight_postural_lower_arm),
                 nb_targets(nb_targets_)
    {
        drho=DELTA_RHO;

        yAssert(nb_targets>=1);

        idx_b=0;

        idx_t.resize(nb_targets);
        idx_t[0]=idx_b+3;
        idx_ua.resize(nb_targets);
        idx_ua[0]=idx_t[0]+3;
        idx_la.resize(nb_targets);
        idx_la[0]=idx_ua[0]+upper_arm.getDOF();

        for (size_t i=1; i<nb_targets; i++)
        {
            idx_t[i]=idx_la[i-1]+3;
            idx_ua[i]=idx_t[i]+3;
            idx_la[i]=idx_ua[i]+upper_arm.getDOF();
        }
        nb_kin_DOF=idx_la[0]+3-3;

        Hb=Rb=eye(4,4);
        H0=upper_arm.getH0();
        HN=upper_arm.getHN();

        Hd.resize(nb_targets, eye(4,4));
        Rd=Hd;
        xd.resize(nb_targets, Vector(3,0.0));

        x0.resize(3+nb_kin_DOF,0.0);

        iKinChain *chain=upper_arm.asChain();
        for (size_t i=0; i<upper_arm.getDOF(); i++)
            x0[idx_ua[0]+i]=0.5*((*chain)[i].getMin()+(*chain)[i].getMax());

        xref=x0;
        x.resize(3+nb_targets*nb_kin_DOF);
        x.setSubvector(0, x0.subVector(0,2));
        for (size_t i=0; i<nb_targets; i++)
            x.setSubvector(3+i*nb_kin_DOF, x0.subVector(3,x0.size()-1));

        d1.resize(nb_targets);
        d2.resize(nb_targets);
        din1.resize(nb_targets);
        din2.resize(nb_targets);
        H.resize(nb_targets,eye(4,4));
        H_.resize(nb_targets,eye(4,4));
        J_.resize(nb_targets,eye(6,upper_arm.getDOF()));
        T.resize(nb_targets,eye(4,4));

        q.resize(nb_targets, x0.subVector(idx_ua[0],idx_ua[0]+upper_arm.getDOF()-1));

        // limits to prevent shoulder from hitting torso's cover
        double roll_0=CTRL_DEG2RAD*5.0;  double pitch_0=CTRL_DEG2RAD*0.0;   // [rad]
        double roll_1=CTRL_DEG2RAD*15.0; double pitch_1=CTRL_DEG2RAD*60.0;  // [rad]

        cover_shoulder_avoidance.resize(2);
        cover_shoulder_avoidance[0]=(roll_1-roll_0)/(pitch_1-pitch_0);
        cover_shoulder_avoidance[1]=roll_0;
    }

    /****************************************************************/
    virtual string get_mode() const=0;

    /****************************************************************/
    TripodState tripod_fkin(const int which, const Vector &x,
                            TripodState *internal=NULL)
    {
        return tripod_fkin(which,(Ipopt::Number*)x.data(),internal);
    }

    /****************************************************************/
    virtual Matrix fkin(const Vector &x)
    {
        TripodState d1=tripod_fkin(1,x);
        TripodState d2=tripod_fkin(2,x);

        Vector o(4,0.0);
        o[2] = 1.0;
        o[3] = CTRL_DEG2RAD*x[idx_b+2];
        Matrix M = axis2dcm(o);
        M[0][3] = x[idx_b];
        M[1][3] = x[idx_b+1];

        return M*d1.T*upper_arm.getH(CTRL_DEG2RAD*x.subVector(idx_ua[0],idx_ua[0]+upper_arm.getDOF()-1))*d2.T*TN;
    }

    /****************************************************************/
    virtual Matrix fkin(const Vector &x, const int frame)
    {
        if ((frame<0) || (frame>11))
            return fkin(x);

        TripodState d1=tripod_fkin(1,x);
        TripodState d2=tripod_fkin(2,x);
        upper_arm.setAng(CTRL_DEG2RAD*x.subVector(idx_ua[0],idx_ua[0]+upper_arm.getDOF()-1));

        Matrix T=d1.T;

        if (frame>=3)
            T*=upper_arm.getH(std::min(frame-3,(int)upper_arm.getDOF()-1));
        
        if (frame>=9)
            T*=d2.T;

        Vector o(4,0.0);
        o[2] = 1.0;
        o[3] = CTRL_DEG2RAD*x[idx_b+2];
        Matrix M = axis2dcm(o);
        M[0][3] = x[idx_b];
        M[1][3] = x[idx_b+1];

        return M*T;
    }

    /****************************************************************/
    virtual void set_q0(const Vector &x0)
    {
        for (size_t i=0; i<3; i++)
            this->x0[idx_b+i]=x0[idx_b+i];

        for (size_t i=0; i<3; i++)
            this->x0[idx_t[0]+i]=std::max(torso.l_min,std::min(torso.l_max,x0[idx_t[0]+i]));

        iKinChain *chain=upper_arm.asChain();
        for (size_t i=0; i<upper_arm.getDOF(); i++)
            this->x0[idx_ua[0]+i]=std::max((*chain)[i].getMin(),std::min((*chain)[i].getMax(),CTRL_DEG2RAD*x0[idx_ua[0]+i]));

        for (size_t i=0; i<3; i++)
            this->x0[idx_la[0]+i]=std::max(lower_arm.l_min,std::min(lower_arm.l_max,x0[idx_la[0]+i]));
    }

    /****************************************************************/
    virtual void set_targets(const vector<Matrix> &Hd)
    {
        yAssert(Hd.size()==nb_targets);
        this->Hd=Hd;
        for (size_t i=0; i<nb_targets ; i++)
        {
            xd[i]=Hd[i].getCol(3).subVector(0,2);
            Rd[i]=Hd[i];
            Rd[i](0,3)=Rd[i](1,3)=Rd[i](2,3)=0.0;
        }
    }

    /****************************************************************/
    virtual void set_qref(const Vector &xref)
    {
        for (size_t i=0; i<2; i++)
            this->xref[idx_b+i]=xref[idx_b+i];

        this->xref[idx_b+2]=CTRL_DEG2RAD*xref[idx_b+2];

        for (size_t i=0; i<3; i++)
            this->xref[idx_t[0]+i]=std::max(torso.l_min,std::min(torso.l_max,xref[idx_t[0]+i]));

        iKinChain *chain=upper_arm.asChain();
        for (size_t i=0; i<upper_arm.getDOF(); i++)
            this->xref[idx_ua[0]+i]=std::max((*chain)[i].getMin(),std::min((*chain)[i].getMax(),CTRL_DEG2RAD*xref[idx_ua[0]+i]));

        for (size_t i=0; i<3; i++)
            this->xref[idx_la[0]+i]=std::max(lower_arm.l_min,std::min(lower_arm.l_max,xref[idx_la[0]+i]));
    }

    /****************************************************************/
    virtual void set_domain(const Vector &domain)
    {
        if(domain.size()>5)
        {
            this->lambda.resize(9*nb_targets+1);
            domain_constr=true;
            domain_poly.resize(domain.size()/2, Vector(2));

            for (size_t i=0; i<domain_poly.size(); i++)
            {
                domain_poly[i][0] = domain[2*i];
                domain_poly[i][1] = domain[2*i+1];
            }
        }
        else
        {
            this->lambda.resize(9*nb_targets);
            domain_constr=false;
            domain_poly.clear();
        }
    }

    /****************************************************************/
    virtual Vector get_result() const
    {
        Vector x_=x;

        x_[idx_b+2]*=CTRL_RAD2DEG;
        x_[idx_b+2]=remainder(x_[idx_b+2], 360.0);

        for (size_t i=0; i<nb_targets; i++)
        {
            for (size_t j=0; j<upper_arm.getDOF(); j++)
            {
                x_[idx_ua[i]+j]=remainder(CTRL_RAD2DEG*x_[idx_ua[i]+j], 360.0);
            }
        }

        return x_;
    }

    /****************************************************************/
    virtual void set_warm_start(const Vector &zL, const Vector &zU,
                                const Vector &lambda)
    {
        this->zL=zL;
        this->zL.resize(idx_la.back()+3);
        this->zU=zU;
        this->zU.resize(idx_la.back()+3);
        this->lambda=lambda;
        if(domain_constr)
            this->lambda.resize(9*nb_targets+1);
        else
            this->lambda.resize(9*nb_targets);
    }

    /****************************************************************/
    virtual void get_warm_start(Vector &zL, Vector &zU, Vector &lambda) const
    {
        zL=this->zL;
        zU=this->zU;
        lambda=this->lambda;
    }

    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
    {
        if (init_x)
        {
            for (Ipopt::Index i=0; i<3; i++)
                x[i]=x0[i];

            for (Ipopt::Index i=0; i<nb_targets; i++)
                for (Ipopt::Index j=0; j<nb_kin_DOF; j++)
                    x[3+i*nb_kin_DOF+j]=x0[3+j];
        }

        if (init_z)
        {
            yAssert((zL.length()==n) && (zU.length()==n));
            for (Ipopt::Index i=0; i<n; i++)
            {
                z_L[i]=zL[i];
                z_U[i]=zU[i];
            }
        }

        if (init_lambda)
        {
            yAssert(this->lambda.length()==m);
            for (Ipopt::Index i=0; i<m; i++)
                lambda[i]=this->lambda[i];
        }

        return true;
    }

    /************************************************************************/
    virtual void computeQuantities(const Ipopt::Number *x, const bool new_x)
    {
        if (new_x)
        {
            for (size_t i=0; i<this->x.length(); i++)
                this->x[i]=x[i];

            Vector b(4,0.0);
            b[2]=1.0;
            b[3]=x[idx_b+2];
            Rb=axis2dcm(b);
            Hb=Rb;
            Hb[0][3]=x[idx_b];
            Hb[1][3]=x[idx_b+1];

            for(size_t i=0; i<nb_targets; i++)
            {
                for (size_t j=0; j<q[i].length(); j++)
                    q[i][j]=x[idx_ua[i]+j];

                d1[i]=tripod_fkin(1,x,&(din1[i]),i);
                d2[i]=tripod_fkin(2,x,&(din2[i]),i);

                upper_arm.setH0(H0); upper_arm.setHN(HN);
                H[i]=upper_arm.getH(q[i]);
                T[i]=d1[i].T*H[i]*d2[i].T*TN;

                upper_arm.setH0(d1[i].T*H0); upper_arm.setHN(HN*d2[i].T*TN);
                H_[i]=upper_arm.getH(q[i]);
                J_[i]=upper_arm.GeoJacobian();
            }
            upper_arm.setH0(H0); upper_arm.setHN(HN);

            if(domain_constr)
            {
                Vector v(2);
                v[0]=x[0];
                v[1]=x[1];
                domain_dist = distanceFromDomain(domain_poly, v);
            }
        }
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
    bool intermediate_callback(Ipopt::AlgorithmMode mode, Ipopt::Index iter,
                               Ipopt::Number obj_value, Ipopt::Number inf_pr,
                               Ipopt::Number inf_du, Ipopt::Number mu,
                               Ipopt::Number d_norm, Ipopt::Number regularization_size,
                               Ipopt::Number alpha_du, Ipopt::Number alpha_pr,
                               Ipopt::Index ls_trials, const Ipopt::IpoptData* ip_data,
                               Ipopt::IpoptCalculatedQuantities* ip_cq)
    {
        if (slv.callback!=NULL)
            return true;//slv.callback->exec(iter,Hd,x,T);
        else
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
        this->x=verify_solution_against_alpha(n,x,m);

        zL.resize(n);
        zU.resize(n);
        this->lambda.resize(m);

        for (Ipopt::Index i=0; i<n; i++)
        {
            zL[i]=z_L[i];
            zU[i]=z_U[i];
        }

        for (Ipopt::Index i=0; i<m; i++)
            this->lambda[i]=lambda[i];
    }

    static double distanceFromDomain(const std::vector<Vector> &domain, const Vector &point)
    {
        int nbSides=domain.size();
        int on_right=0;
        double dist=0;
        double min_dist=std::numeric_limits<double>::max();
        Vector p1(2);
        Vector p2=domain[nbSides-1];
        Vector x(2);
        Vector x1(2);
        Vector x2(2);

        for(size_t i=0; i<nbSides; i++)
        {
            p1=p2;
            p2=domain[i];

            x=p2-p1;
            x*=1.0/norm(x);
            x1=point-p1;
            x2=point-p2;

            if( dot(x1,x)<=0 )
                dist=norm(x1);
            else if( dot(x2,x)>=0 )
                dist=norm(x2);
            else
                dist=fabs(x1[1]*x[0]-x1[0]*x[1]);

            if(dist<min_dist)
            {
                min_dist=dist;
                if(min_dist==0)
                    break;
            }

            if( (p1[1]<=point[1] && p2[1]<=point[1]) ||
               (p1[1]>point[1] && p2[1]>point[1]) ||
               (p1[0]<point[0] && p2[0]<point[0]) )
                continue;

            double signed_dist=x1[1]*x[0]-x1[0]*x[1];
            if(x[1]<0)
                signed_dist=-signed_dist;
            if(signed_dist>0)
                on_right++;
        }

        if(on_right%2==0)
            return -min_dist;
        else
            return min_dist;
    }
};


