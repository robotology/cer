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
using namespace cer::kinematics;


/****************************************************************/
class Target : public PeriodicThread
{
    mutable Mutex mutex;
    Vector c,xd;
    double f,R;
    int cnt;
    double t0;

    /****************************************************************/
    void helperPrint(const double t, const string &tag, const int cnt,
                     const Vector &x) const
    {
        printf("%.3f %s %d %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
               t,tag.c_str(),cnt,x[0],x[1],x[2],x[3],x[4],x[5],x[6]);
    }

    /****************************************************************/
    void run()
    {
        LockGuard lg(mutex);        
        const double t=Time::now()-t0;
        const double phi=2.0*M_PI*f*t;

        Vector r(7,0.0);
        r[1]=R*cos(phi);
        r[2]=R*sin(phi);

        xd=c+r;
        helperPrint(t,"xd",++cnt,xd);        
    }

public:
    /****************************************************************/
    Target() : PeriodicThread(0.01), c(7,0.0), xd(7,0.0), f(0.1), R(0.1), cnt(0)
    {
        c[0]=0.35;
        c[4]=1.0;
        c[6]=M_PI/2.0;
        t0=Time::now();
    }

    /****************************************************************/
    void setOptions(const Property &options)
    {
        if (options.check("Ts"))
            setPeriod(options.find("Ts").asDouble());

        if (options.check("f"))
            f=options.find("f").asDouble();

        if (options.check("R"))
            R=options.find("R").asDouble();
    }

    /****************************************************************/
    Vector get_xd(int &cnt) const
    {
        LockGuard lg(mutex);
        cnt=this->cnt;
        return xd;
    }

    /****************************************************************/
    void print(const int cnt, const Vector &x) const
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
    double solverTs;
    Vector q;

public:
    /****************************************************************/
    IKSolver() : q(12,0.0) { }

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {        
        string arm_type=rf.check("arm-type",Value("left")).asString();
        string mode=rf.check("mode",Value("full_pose+no_torso_no_heave+forward_diff")).asString();
        solverTs=rf.check("solverTs",Value(0.01)).asDouble();
        double targetTs=rf.check("targetTs",Value(0.05)).asDouble();
        double f=rf.check("f",Value(0.1)).asDouble();
        double R=rf.check("R",Value(0.1)).asDouble();
        
        Property options;
        options.put("Ts",targetTs);
        options.put("f",f);
        options.put("R",R);        
        target.setOptions(options);
        
        SolverParameters p=solver.getSolverParameters();
        p.setMode(mode);
        p.weight_postural_torso=0.001;
        p.weight_postural_torso_yaw=0.001;
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
        return solverTs;
    }

    /****************************************************************/
    bool updateModule()
    {
        int cnt;
        Vector xd=target.get_xd(cnt);

        Matrix Hd=axis2dcm(xd.subVector(3,6));
        Hd(0,3)=xd[0];
        Hd(1,3)=xd[1];
        Hd(2,3)=xd[2];

        solver.setInitialGuess(q);
        solver.ikin(Hd,q);

        Matrix H;
        solver.fkin(q,H);
        Vector x=H.getCol(3).subVector(0,2);
        x=cat(x,dcm2axis(H));
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

