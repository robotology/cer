
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

#include <csignal>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <algorithm>
#include <deque>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <cer_kinematics/arm.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace cer_kinematics;


/****************************************************************/
namespace
{
    volatile std::sig_atomic_t gSignalStatus;
}


/****************************************************************/
void signal_handler(int signal)
{
    gSignalStatus=signal;
}


/****************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.configure(argc,argv);

    // command-line options
    string arm_type=rf.check("arm-type",Value("left")).asString().c_str();
    string grasp_type=rf.check("grasp-type",Value("top")).asString().c_str();
    double table_height=rf.check("table-height",Value(0.7)).asDouble();
    double external_weight=rf.check("external-weight",Value(2.0)).asDouble();
    double floor_z=rf.check("floor-z",Value(-0.63)).asDouble();
    double step=rf.check("step",Value(0.05)).asDouble();

    // define solver and its parameters
    ArmParameters armp(arm_type);
    ArmSolver solver(armp);
    Vector q(12,0.0);

    SolverParameters slvp=solver.getSolverParameters();
    if (table_height>0.7)
        slvp.setMode("full+heave");
    else
        slvp.setMode("full");
    slvp.torso_heave=0.1;
    slvp.lower_arm_heave=0.01;
    solver.setSolverParameters(slvp);

    // init CoMs, weights and support polygon
    ArmCOM armCOM(solver,external_weight,floor_z);

    // targets
    Vector ud(4,0.0);
    if (grasp_type=="top")
        ud[1]=1.0;
    else
        ud[0]=-1.0;
    ud[3]=M_PI/2.0;     

    Matrix Hd=axis2dcm(ud);
    ud*=ud[3]; ud.pop_back();
    Hd(2,3)=table_height+floor_z;
    
    // timing statistics
    double maxT=0.0;
    double minT=std::numeric_limits<double>::max();

    double avgT=0.0;
    double stdT=0.0;
    double N=0.0;

    ofstream fout;
    fout.open("data.log");

    std::signal(SIGINT,signal_handler);
    for (Hd(0,3)=0.3; Hd(0,3)<=1.0; Hd(0,3)+=step)
    {
        for (Hd(1,3)=0.5; Hd(1,3)>=-0.5; Hd(1,3)-=step)
        {
            // inverse kinematics
            solver.setInitialGuess(q);
            double t0=Time::now();
            solver.ikin(Hd,q);
            double dt=1000.0*(Time::now()-t0);

            // forward kinematics
            Matrix H;
            solver.fkin(q,H);

            maxT=std::max(maxT,dt);
            minT=std::min(minT,dt);

            double avgT_n1=(avgT*N+dt)/(N+1.0);
            stdT=sqrt((N*(stdT*stdT+avgT*avgT)+dt*dt)/(N+1.0)-avgT_n1*avgT_n1);
            avgT=avgT_n1;
            N+=1.0;

            deque<Vector> coms;
            armCOM.getCOMs(q,coms);
            Vector &com=coms.back();

            double margin;
            armCOM.getSupportMargin(com,margin);

            Vector xd=Hd.getCol(3).subVector(0,2);
            Vector x=H.getCol(3).subVector(0,2);
            Vector u=dcm2axis(H);
            u*=u[3]; u.pop_back();

            ostringstream stream;
            stream.precision(5);
            stream<<fixed;

            stream<<xd.toString(3,3).c_str();
            stream<<"\t";
            stream<<x.toString(3,3).c_str();
            stream<<"\t";
            stream<<u.toString(3,3).c_str();
            stream<<"\t";
            stream<<com.subVector(0,2).toString(3,3).c_str();
            stream<<"\t";
            stream<<margin;

            fout<<stream.str()<<endl;
            yInfo("%s",stream.str().c_str());
            yInfo("solving time [ms]: min=%d, avg=%d, std=%d, max=%d;",
                  (int)minT,(int)avgT,(int)stdT,(int)maxT);

            if (gSignalStatus==SIGINT)
            {
                yWarning("SIGINT detected: closing ...");
                break;
            }
        }

        if (gSignalStatus==SIGINT)
            break;
    }

    fout.close();
    return 0;
}


