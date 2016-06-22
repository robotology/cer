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
#include <cer_kinematics/head.h>

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
        cout<<"--type left|center|right"<<endl;
        cout<<"--verbosity <int>"<<endl;
        cout<<"--xd \"(0.0 1.0 2.0)\""<<endl;        
        return 0;
    }

    string type=rf.check("type",Value("center")).asString();
    int verbosity=rf.check("verbosity",Value(0)).asInt();

    if ((type!="left") && (type!="center") && (type!="right"))
    {
        cerr<<"unrecognized type \""<<type<<"\""<<endl;
        return 1;
    }

    Vector xd(3,0.0);
    if (Bottle *b=rf.find("xd").asList())
    {
        size_t len=std::min(xd.length(),(size_t)b->size());
        for (size_t i=0; i<len; i++)
            xd[i]=b->get(i).asDouble();
    }

    HeadParameters headp(type);
    HeadSolver solver(headp);
    solver.setVerbosity(verbosity);

    Vector q;
    solver.ikin(xd,q);
    cout<<"head="<<type<<endl;
    cout<<"xd=("<<xd.toString(3,3)<<")"<<endl;
    cout<<"q=("<<q.toString(3,3)<<")"<<endl;
    cout<<endl;

    return 0;
}


