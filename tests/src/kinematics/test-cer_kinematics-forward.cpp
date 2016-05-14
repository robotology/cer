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

#include <iostream>
#include <iomanip>

#include <yarp/sig/all.h>
#include <cer_kinematics/arm.h>

using namespace std;
using namespace yarp::sig;
using namespace cer::kinematics;


/****************************************************************/
int main()
{
    // define solver and its parameters
    ArmParameters armp("left");
    ArmSolver solver(armp);

    Vector q(12);
    Matrix H;

    // pose #0
    q=0.0;
    solver.fkin(q,H);
    cout<<"q=("<<q.toString(3,3)<<")"<<endl;
    cout<<"H="<<H.toString(3,3)<<endl;
    cout<<endl;

    // pose #1
    q[3]=-18.0;
    q[4]=+44.0;
    q[5]=+74.0;
    q[6]=-89.0;
    q[7]=+95.0;
    q[8]=+34.0;
    solver.fkin(q,H);
    cout<<"q=("<<q.toString(3,3)<<")"<<endl;
    cout<<"H="<<H.toString(3,3)<<endl;
    cout<<endl;

    // pose #2
    q[0] =0.10;
    q[1] =0.11;
    q[2] =0.09;
    q[9] =0.05;
    q[10]=0.06;
    q[11]=0.04;
    solver.fkin(q,H);
    cout<<"q=("<<q.toString(3,3)<<")"<<endl;
    cout<<"H="<<H.toString(3,3)<<endl;
    cout<<endl;

    return 0;
}


