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
#include <cmath>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/minJerkCtrl.h>

#include <cer_kinematics/tripod.h>

#define MODE_HPR    "hpr"
#define MODE_ZD_UD  "zd+ud"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace cer::kinematics;


/****************************************************************/
class IKSolver : public RFModule
{
    BufferedPort<Bottle> iPort,oPort;
    TripodSolver solver;
    minJerkTrajGen *gen;
    Vector rho;

public:
    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        Vector rot(4,0.0);
        rot[2]=1.0; rot[3]=M_PI;
        Matrix R=(rf.check("add-base-rotation")?axis2dcm(rot):eye(4,4));
        solver.setParameters(TripodParameters(0.09,0.0,0.2,30.0,R));

        rho.resize(3,0.0);
        solver.setVerbosity(rf.check("verbosity",Value(1)).asInt());
        gen=new minJerkTrajGen(rho,getPeriod(),2.0);

        iPort.open("/solver:i");
        oPort.open("/solver:o");

        return true;
    }

    /****************************************************************/
    bool close()
    {
        iPort.close();
        oPort.close();
        delete gen;

        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /****************************************************************/
    bool updateModule()
    {
        Bottle *ibData=iPort.read(false);
        if (ibData!=NULL)
        {
            if (ibData->size()<5)
            {
                yError()<<"Wrong input size!";
                return true;
            }

            Vector ud(4);
            double zd=ibData->get(0).asDouble();
            ud[0]=ibData->get(1).asDouble();
            ud[1]=ibData->get(2).asDouble();
            ud[2]=ibData->get(3).asDouble();
            ud[3]=ibData->get(4).asDouble();
            solver.setInitialGuess(rho);
            solver.ikin(zd,ud,rho);

            yInfo()<<"zd="<<zd<<"; ud=("<<ud.toString(5,5)
                   <<"); rho=("<<rho.toString(5,5)<<");";
        }
        else
        {
            gen->computeNextValues(rho);
            Vector ref=gen->getPos();
            oPort.prepare().read(ref);
            oPort.write();
        }
        
        return true;
    }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP server not available!";
        return 1;
    }
    
    ResourceFinder rf;
    rf.configure(argc,argv);

    IKSolver solver;
    return solver.runModule(rf);
}

