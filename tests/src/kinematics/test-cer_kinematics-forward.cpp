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

#include <string>
#include <iostream>
#include <iomanip>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <cer_kinematics/arm.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace cer::kinematics;


/****************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout<<"Options:"<<endl;
        cout<<"--arm-type left|right"<<endl;
        cout<<"--q \"(0.0 1.0 ... 11.0)\""<<endl;
        return 0;
    }

    string arm_type=rf.check("arm-type",Value("left")).asString();
    ArmParameters armp(arm_type);
    ArmSolver solver(armp);
    
    Vector q(12,0.0);
    if (Bottle *b=rf.find("q").asList())
    {
        size_t len=std::min(q.length(),(size_t)b->size());
        for (size_t i=0; i<len; i++)
            q[i]=b->get(i).asDouble();
    }

    Matrix H;
    solver.fkin(q,H);
    cout<<"q=("<<q.toString(3,3)<<")"<<endl;
    cout<<"H="<<H.toString(3,3)<<endl;
    cout<<endl;

    return 0;
}


