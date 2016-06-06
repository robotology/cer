/*
* Copyright (C)2015  iCub Facility - Istituto Italiano di Tecnologia
* Author: Marco Randazzo
* email:  marco.randazzo@iit.it
* website: www.robotcub.org
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
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Log.h>
//#include <vld.h> //<<<<<<<for who uses vld: add include and lib/win32 directory and uncomment  


#include "tfModule.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::math;
using namespace yarp::sig;

int main(int argc, char* argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("tfPublisher");
    rf.setDefaultConfigFile("tfPublisher.ini");
    rf.configure(argc,argv);

    if ( rf.check("help") )
    {
        yInfo( "Possible options: " );
        yInfo( "'rate <r>' sets the threads rate (default 20ms)." );
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    tfModule mod;

    return mod.runModule(rf);
}
