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

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/minJerkCtrl.h>

#include <tripod/tripod.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace tripod;


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
        iPort.open("/solver:i");
        oPort.open("/solver:o");

        rho.resize(3,0.0);
        gen=new minJerkTrajGen(rho,getPeriod(),2.0);

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
            if (ibData->size()<4)
            {
                yError("wrong input!");
                return true;
            }

            Vector ud(3);
            double zd=ibData->get(0).asDouble();        
            ud[0]=ibData->get(1).asDouble();
            ud[1]=ibData->get(2).asDouble();
            ud[2]=ibData->get(3).asDouble();

            solver.setInitialGuess(rho);
            solver.ikin(zd,ud,rho);
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
int main()
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    IKSolver solver;
    ResourceFinder rf;
    return solver.runModule(rf);
}

