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

#include <cstdio>
#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <cer_kinematics/utils.h>
#include <cer_kinematics/arm.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace cer_kinematics;


/****************************************************************/
class Target : public RateThread
{
    Mutex mutex;
    Vector c,xd;
    double f,R;
    int cnt;
    double t0;

    /****************************************************************/
    void helperPrint(const double t, const string &tag, const int cnt,
                     const Vector &x)
    {
        printf("%.3f %s %d %.3f %.3f %.3f %.3f %.3f %.3f\n",
               t,tag.c_str(),cnt,x[0],x[1],x[2],x[3],x[4],x[5]);
    }

    /****************************************************************/
    void run()
    {
        LockGuard lg(mutex);        
        const double t=Time::now()-t0;
        const double phi=2.0*M_PI*f*t;

        Vector r(6,0.0);
        r[1]=R*cos(phi);
        r[2]=R*sin(phi);

        xd=c+r;
        helperPrint(t,"xd",++cnt,xd);        
    }

public:
    /****************************************************************/
    Target(const double Ts) : RateThread((int)(Ts*1000.0)),
                              c(6,0.0), xd(6,0.0)
    {
        c[0]=0.35;
        c[4]=M_PI/2.0;
        f=1.0/10.0;
        R=0.1;
        cnt=0;
        t0=Time::now();
    }

    /****************************************************************/
    Vector get_xd(int &cnt)
    {
        LockGuard lg(mutex);
        cnt=this->cnt;
        return xd;
    }

    /****************************************************************/
    void print(const int cnt, const Vector &x)
    {
        LockGuard lg(mutex);
        helperPrint(Time::now()-t0,"x",cnt,x);
    }
};


/****************************************************************/
class IKSolver : public RFModule
{
    Target target;
    ArmSolver solver;
    Vector q;

public:
    /****************************************************************/
    IKSolver() : target(0.05), q(12,0.0) { }

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string arm_type=rf.check("arm-type",Value("left")).asString().c_str();
        string mode=rf.check("mode",Value("full_pose+no_torso+forward_diff")).asString().c_str();
        
        SolverParameters p=solver.getSolverParameters();
        p.setMode(mode);
        p.torso_heave=0.1;
        p.lower_arm_heave=0.05;
        p.warm_start=true;

        solver.setArmParameters(ArmParameters(arm_type));
        solver.setSolverParameters(p);

        target.start();
        return true;
    }

    /****************************************************************/
    bool close()
    {
        target.stop();
        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.01;
    }

    /****************************************************************/
    bool updateModule()
    {
        int cnt;
        Vector xd=target.get_xd(cnt);

        Vector ud=xd.subVector(3,5);
        double n=norm(ud);
        ud=(1.0/n)*ud;
        ud.push_back(n);
        Matrix Hd=axis2dcm(ud);
        Hd(0,3)=xd[0];
        Hd(1,3)=xd[1];
        Hd(2,3)=xd[2];

        solver.setInitialGuess(q);
        solver.ikin(Hd,q);

        Matrix H;
        solver.fkin(q,H);
        Vector x=H.getCol(3).subVector(0,2);
        Vector u=dcm2axis(H);
        u*=u[3];
        u.pop_back();
        x=cat(x,u);
        target.print(cnt,x);

        return true;
    }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.configure(argc,argv);

    IKSolver solver;
    return solver.runModule(rf);
}

