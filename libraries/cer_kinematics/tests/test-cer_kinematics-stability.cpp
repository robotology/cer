
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
int main()
{
    std::signal(SIGINT,signal_handler);    

    ofstream fout;
    fout.open("data.log");

    ArmSolver solver;
    Vector q(12,0.0);

    SolverParameters p=solver.getSolverParameters();
    p.setMode("full");
    p.torso_heave=0.1;
    p.lower_arm_heave=0.01;
    solver.setSolverParameters(p);    

    Vector ud(4,0.0);
    ud[1]=1.0; ud[3]=M_PI/2.0;
    Matrix Hd=axis2dcm(ud);
    ud*=ud[3]; ud.pop_back();
    Hd(2,3)=0.7-0.63;   // table height - root frame's height
    
    double maxT=0.0;
    double minT=std::numeric_limits<double>::max();

    double avgT=0.0;
    double avgN=0.0;

    for (Hd(0,3)=0.3; Hd(0,3)<1.0; Hd(0,3)+=0.01)
    {
        for (Hd(1,3)=0.3; Hd(1,3)>0.0; Hd(1,3)-=0.01)
        {
            solver.setInitialGuess(q);

            double t0=Time::now();
            solver.ikin(Hd,q);
            double dt=1000.0*(Time::now()-t0);

            maxT=std::max(maxT,dt);
            minT=std::min(minT,dt);

            avgT=(avgT*avgN+dt)/(avgN+1.0);
            avgN+=1.0;

            Matrix T;
            solver.fkin(q,T);

            Vector xd=Hd.getCol(3).subVector(0,2);
            Vector x=T.getCol(3).subVector(0,2);
            Vector u=dcm2axis(T);
            u*=u[3]; u.pop_back();

            ostringstream stream;
            stream.precision(5);
            stream<<fixed;

            stream<<xd.toString(3,3).c_str();
            stream<<"\t";
            stream<<q.toString(3,3).c_str();
            stream<<"\t";
            stream<<norm(xd-x);
            stream<<"\t";
            stream<<norm(ud-u);
            
            fout<<stream.str()<<endl;
            yInfo("%s",stream.str().c_str());
            yInfo("solving time [ms]: min=%d, avg=%d, max=%d;",
                  (int)minT,(int)avgT,(int)maxT);

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


