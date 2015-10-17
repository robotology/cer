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

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/minJerkCtrl.h>

#include <cer_kinematics/tripod.h>

#define MODE_HPR    "hpr"
#define MODE_ZD_UD  "zd+ud"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace cer_kinematics;


/****************************************************************/
class IKSolver : public RFModule
{
    BufferedPort<Bottle> iPort,oPort;
    TripodSolver solver;
    minJerkTrajGen *gen;
    Vector rho;

    string mode;
    bool use_hpr;

public:
    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        mode=rf.check("mode",Value(MODE_ZD_UD)).asString().c_str();
        int verbosity=rf.check("verbosity",Value(1)).asInt();        

        if ((mode!=MODE_ZD_UD) && (mode!=MODE_HPR))
        {
            yWarning("Unrecognized input mode!");
            mode=MODE_ZD_UD;
        }
        use_hpr=(mode==MODE_HPR); 

        iPort.open("/solver:i");
        oPort.open("/solver:o");

        rho.resize(3,0.0);
        solver.setVerbosity(verbosity);
        gen=new minJerkTrajGen(rho,getPeriod(),2.0);

        yInfo("Using \"%s\" input mode",mode.c_str());
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
            if (ibData->size()!=(use_hpr?3:4))
            {
                yError("Wrong \"%s\" input size!",mode.c_str());
                return true;
            }

            if (use_hpr)
            {
                Vector hpr(3);
                hpr[0]=ibData->get(0).asDouble();
                hpr[1]=ibData->get(1).asDouble();
                hpr[2]=ibData->get(2).asDouble();
                solver.setInitialGuess(rho);
                solver.ikin(hpr,rho);

                yInfo("hpr=(%s); rho=(%s);",
                      hpr.toString(5,5).c_str(),
                      rho.toString(5,5).c_str());
            }
            else
            {
                Vector ud(3);
                double zd=ibData->get(0).asDouble();
                ud[0]=ibData->get(1).asDouble();
                ud[1]=ibData->get(2).asDouble();
                ud[2]=ibData->get(3).asDouble();
                solver.setInitialGuess(rho);
                solver.ikin(zd,ud,rho);

                yInfo("zd=%g; ud=(%s); rho=(%s);",
                      zd,ud.toString(5,5).c_str(),
                      rho.toString(5,5).c_str());
            }
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
        yError("YARP server not available!");
        return 1;
    }
    
    ResourceFinder rf;
    rf.configure(argc,argv);

    IKSolver solver;
    return solver.runModule(rf);
}

