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

#include <cer_kinematics/utils.h>
#include <cer_kinematics/arm.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace cer::kinematics;


/****************************************************************/
class Callback : public SolverIterateCallback
{
public:
    /****************************************************************/
    bool exec(const int iter, const Matrix &Hd, const Vector &q,
              const Matrix &Hee)
    {
        yDebug()<<"q["<<iter<<"]=("<<q.toString(3,3)<<")";
        return true;
    }
};


/****************************************************************/
class IKSolver : public RFModule
{
    BufferedPort<Bottle> portTarget;
    BufferedPort<Bottle> portSolution;

    ArmSolver solver;
    Callback callback;
    Vector q;

    ArmCOM *armCOM;

public:
    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string arm_type=rf.check("arm-type",Value("left")).asString();
        double external_weight=rf.check("external-weight",Value(2.0)).asFloat64();
        double floor_z=rf.check("floor-z",Value(-0.16)).asFloat64();
        int verbosity=rf.check("verbosity",Value(1)).asInt32();
        bool enable_callback=rf.check("enable-callback");

        q.resize(12,0.0);
        portTarget.open("/solver/target:i");
        portSolution.open("/solver/solution:o");

        SolverParameters p=solver.getSolverParameters();
        p.setMode("full_pose");
        p.weight_postural_torso=0.001;
        p.weight_postural_torso_yaw=0.001;

        solver.setArmParameters(ArmParameters(arm_type));
        solver.setSolverParameters(p);
        solver.setVerbosity(verbosity);
        if (enable_callback)
            solver.enableIterateCallback(callback); 

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
            if (target->size()<10)
            {
                yError()<<"wrong target size!";
                return true;
            }

            string mode;
            double hd1,hd2;
            Vector xd(3),ud(4);

            mode=target->get(0).asString();
            hd1=target->get(1).asFloat64();
            hd2=target->get(2).asFloat64();
            xd[0]=target->get(3).asFloat64();
            xd[1]=target->get(4).asFloat64();
            xd[2]=target->get(5).asFloat64();
            ud[0]=target->get(6).asFloat64();
            ud[1]=target->get(7).asFloat64();
            ud[2]=target->get(8).asFloat64();
            ud[3]=target->get(9).asFloat64();

            bool warm_start=false;
            if (target->size()>=11)
                warm_start=(target->get(10).asString()=="warm_start");

            Matrix Hd=axis2dcm(ud);
            Hd.setSubcol(xd,0,3);

            SolverParameters p=solver.getSolverParameters();
            p.setMode(mode);
            p.torso_heave=hd1;
            p.lower_arm_heave=hd2;
            p.warm_start=warm_start;

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
        yError()<<"YARP server not available!";
        return 1;
    }
    
    ResourceFinder rf;
    rf.configure(argc,argv);

    IKSolver solver;
    return solver.runModule(rf);
}

