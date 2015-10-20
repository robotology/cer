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
#include <yarp/math/Math.h>

#include <cer_kinematics/arm.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace cer_kinematics;


/****************************************************************/
class IKSolver : public RFModule
{
    BufferedPort<Bottle> portTarget;
    BufferedPort<Bottle> portSolution;

    ArmSolver solver;
    Vector q;

    ArmCOM *armCOM;

public:
    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string arm_type=rf.check("arm-type",Value("left")).asString().c_str();
        double external_weight=rf.check("external-weight",Value(2.0)).asDouble();
        double floor_z=rf.check("floor-z",Value(-0.63)).asDouble();
        int verbosity=rf.check("verbosity",Value(1)).asInt();

        q.resize(12,0.0);
        portTarget.open("/solver/target:i");
        portSolution.open("/solver/solution:o");

        SolverParameters p=solver.getSolverParameters();
        p.setMode("full");

        solver.setArmParameters(ArmParameters(arm_type));
        solver.setSolverParameters(p);
        solver.setVerbosity(verbosity);

        armCOM=new ArmCOM(solver,external_weight,floor_z);
        return true;
    }

    /****************************************************************/
    bool close()
    {
        portTarget.close();
        portSolution.close();
        delete armCOM;

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
        Bottle *target=portTarget.read(false);
        if (target!=NULL)
        {
            if (target->size()<9)
            {
                yError("wrong target size!");
                return true;
            }

            string mode;
            double zd1,zd2;
            Vector xd(3),ud(3);

            mode=target->get(0).asString().c_str();
            zd1=target->get(1).asDouble();
            zd2=target->get(2).asDouble();
            xd[0]=target->get(3).asDouble();
            xd[1]=target->get(4).asDouble();
            xd[2]=target->get(5).asDouble();
            ud[0]=target->get(6).asDouble();
            ud[1]=target->get(7).asDouble();
            ud[2]=target->get(8).asDouble();

            double n=norm(ud);
            Vector ud_=(1.0/n)*ud;
            ud_.push_back(n);
            Matrix Hd=axis2dcm(ud_);
            Hd(0,3)=xd[0];
            Hd(1,3)=xd[1];
            Hd(2,3)=xd[2];

            SolverParameters p=solver.getSolverParameters();
            p.setMode(mode);
            p.torso_heave=zd1;
            p.lower_arm_heave=zd2;

            solver.setSolverParameters(p);
            solver.setInitialGuess(q);
            solver.ikin(Hd,q);

            deque<Vector> coms;
            armCOM->getCOMs(q,coms);
            Vector &com=coms.back();

            double margin;
            armCOM->getSupportMargin(com,margin);

            Vector solution=cat(cat(xd,ud),q);
            for (size_t i=0; i<coms.size(); i++)
                solution=cat(solution,coms[i].subVector(0,2));
            solution.push_back(margin);

            portSolution.prepare().read(solution);
            portSolution.writeStrict();
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

