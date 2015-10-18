
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

    // define solver and set parameters
    ArmParameters armp(arm_type);
    ArmSolver solver(armp);
    Vector q(12,0.0);

    SolverParameters slvp=solver.getSolverParameters();
    slvp.setMode("full");
    slvp.torso_heave=0.0;
    slvp.lower_arm_heave=0.01;
    solver.setSolverParameters(slvp);

    // CoM positions relative to frames
    Vector com_mobilebase_lowertorso(4,1.0);
    com_mobilebase_lowertorso[0]=0.0264;
    com_mobilebase_lowertorso[1]=0.0;
    com_mobilebase_lowertorso[2]=0.215;
    
    Vector com_l0(4,1.0);
    com_l0[0]=0.074;
    com_l0[1]=0.0;
    com_l0[2]=0.79;

    Vector com_l3(4,1.0);
    com_l3[0]=-0.00945;
    com_l3[1]=0.117;
    com_l3[2]=0.96;
    
    Vector com_l5(4,1.0);
    com_l5[0]=-0.01;
    com_l5[1]=0.191;
    com_l5[2]=0.541;
    
    Vector com_hand(4,1.0);
    com_hand[0]=0.0;
    com_hand[1]=0.0;
    com_hand[2]=0.0;
    
    // weights
    double weight_mobilebase_lowertorso=27.5;
    double weight_l0=5.67;
    double weight_l3=2.83;
    double weight_l5=0.6;
    double weight_hand=0.6+2.0;
    double weight_tot=weight_mobilebase_lowertorso+
                      weight_l0+weight_l3+weight_l5+
                      weight_hand;

    // targets
    Vector ud(4,0.0);
    if (grasp_type=="top")
        ud[1]=1.0;
    else
        ud[0]=-1.0;
    ud[3]=M_PI/2.0;     

    Matrix Hd=axis2dcm(ud);
    ud*=ud[3]; ud.pop_back();
    Hd(2,3)=table_height-0.63;
    
    // timing statistics
    double maxT=0.0;
    double minT=std::numeric_limits<double>::max();

    double avgT=0.0;
    double stdT=0.0;
    double N=0.0;

    ofstream fout;
    fout.open("data.log");

    std::signal(SIGINT,signal_handler);
    for (Hd(0,3)=0.3; Hd(0,3)<=1.0; Hd(0,3)+=0.02)
    {
        for (Hd(1,3)=0.3; Hd(1,3)>=-0.3; Hd(1,3)-=0.02)
        {
            solver.setInitialGuess(q);

            double t0=Time::now();
            solver.ikin(Hd,q);
            double dt=1000.0*(Time::now()-t0);

            maxT=std::max(maxT,dt);
            minT=std::min(minT,dt);

            double avgT_n1=(avgT*N+dt)/(N+1.0);
            stdT=sqrt((N*(stdT*stdT+avgT*avgT)+dt*dt)/(N+1.0)-avgT_n1*avgT_n1);
            avgT=avgT_n1;
            N+=1.0;

            Matrix frame_l0;
            solver.fkin(q,frame_l0,3+0);

            Matrix frame_l3;
            solver.fkin(q,frame_l3,3+3);

            Matrix frame_l5;
            solver.fkin(q,frame_l5,3+5);

            Matrix frame_hand;
            solver.fkin(q,frame_hand);

            Vector com=weight_mobilebase_lowertorso*com_mobilebase_lowertorso+
                       weight_l0*(frame_l0*com_l0)+weight_l3*(frame_l3*com_l3)+
                       weight_l5*(frame_l5*com_l5)+weight_hand*(frame_hand*com_hand);
            com.pop_back();
            com/=weight_tot;            

            Vector xd=Hd.getCol(3).subVector(0,2);
            Vector x=frame_hand.getCol(3).subVector(0,2);
            Vector u=dcm2axis(frame_hand);
            u*=u[3]; u.pop_back();

            ostringstream stream;
            stream.precision(5);
            stream<<fixed;

            stream<<xd.toString(3,3).c_str();
            stream<<"\t";
            stream<<x.toString(3,3).c_str();
            stream<<"\t";
            stream<<norm(xd-x);
            stream<<"\t";
            stream<<norm(ud-u);
            stream<<"\t";
            stream<<com.toString(3,3).c_str();

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


