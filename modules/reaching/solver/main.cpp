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

#define _USE_MATH_DEFINES
#include <cmath>
#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>
#include <cer_kinematics/arm.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace cer::kinematics;


/****************************************************************/
class IKSolver : public RFModule
{    
    ArmSolver solver;
    RpcServer rpcPort;
    Vector q;

    /****************************************************************/
    bool getLimits(const string &remote, const string &local,
                   Matrix &lim) const
    {        
        Property option;
        option.put("device","remote_controlboard");
        option.put("remote",remote.c_str());
        option.put("local",local.c_str());

        PolyDriver driver;
        if (driver.open(option))
        {
            IEncoders      *ienc;
            IControlLimits *ilim;

            driver.view(ienc);
            driver.view(ilim);

            int nAxes;
            ienc->getAxes(&nAxes);

            lim.resize(nAxes,2);
            for (int i=0; i<nAxes; i++)
                ilim->getLimits(i,&lim(i,0),&lim(i,1));

            driver.close();
            return true;
        }
        else
        {
            yError("Unable to connect to %s!",remote.c_str());
            return false;
        }
    }

    /****************************************************************/
    bool alignJointsBounds(const string &robot, const string &arm_type)
    {
        ArmParameters p=solver.getArmParameters();
        Matrix lim;

        if (getLimits("/"+robot+"/torso","/cer_solver/torso",lim))
        {
            p.torso.l_min=lim(0,0);
            p.torso.l_max=lim(0,1);
            p.torso.alpha_max=fabs(lim(1,1));

            yInfo("limits of %s part: heave=[%g,%g] [m], alpha_max=%g [deg]",
                  ("/"+robot+"/torso").c_str(),p.torso.l_min,
                  p.torso.l_max,p.torso.alpha_max);
        }
        else
            return false;

        if (getLimits("/"+robot+"/torso_yaw","/cer_solver/torso_yaw",lim))
        {
            iKinChain *chain=p.upper_arm.asChain();
            (*chain)[0].setMin((M_PI/180.0)*lim(0,0));
            (*chain)[0].setMax((M_PI/180.0)*lim(0,1));

            yInfo("limits of %s part: joint %d=[%g,%g] [deg]",
                  ("/"+robot+"/torso_yaw").c_str(),0,
                  (180.0/M_PI)*(*chain)[0].getMin(),
                  (180.0/M_PI)*(*chain)[0].getMax());
        }
        else
            return false;

        if (getLimits("/"+robot+"/upper_"+arm_type+"_arm","/cer_solver/upper_"+arm_type+"_arm",lim))
        {
            iKinChain *chain=p.upper_arm.asChain();
            for (int i=0; i<lim.rows(); i++)
            {
                (*chain)[1+i].setMin((M_PI/180.0)*lim(i,0)); 
                (*chain)[1+i].setMax((M_PI/180.0)*lim(i,1));

                yInfo("limits of %s part: joint %d=[%g,%g] [deg]",
                      ("/"+robot+"/upper_"+arm_type+"_arm").c_str(),i,
                      (180.0/M_PI)*(*chain)[1+i].getMin(),
                      (180.0/M_PI)*(*chain)[1+i].getMax());
            }
        }
        else
            return false;

        if (getLimits("/"+robot+"/"+arm_type+"_wrist","/cer_solver/"+arm_type+"_wrist",lim))
        {
            p.lower_arm.l_min=lim(0,0);
            p.lower_arm.l_max=lim(0,1);
            p.lower_arm.alpha_max=fabs(lim(1,1));

            yInfo("limits of %s part: heave=[%g,%g] [m], alpha_max=%g [deg]",
                  ("/"+robot+"/"+arm_type+"_wrist").c_str(),p.lower_arm.l_min,
                  p.lower_arm.l_max,p.lower_arm.alpha_max);
        }
        else
            return false;

        return true;
    }

public:
    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("cer")).asString().c_str();
        string arm_type=rf.check("arm-type",Value("left")).asString().c_str();
        int verbosity=rf.check("verbosity",Value(0)).asInt();

        if (!alignJointsBounds(robot,arm_type))
            return false;

        SolverParameters p=solver.getSolverParameters();
        p.setMode("full_pose");
        p.torso_heave=0.0;
        p.lower_arm_heave=0.02;
        p.warm_start=true;

        solver.setArmParameters(ArmParameters(arm_type));
        solver.setSolverParameters(p);
        solver.setVerbosity(verbosity);        

        rpcPort.open(("/cer_solver/"+arm_type+"/rpc").c_str());
        attach(rpcPort);
        q.resize(12,0.0);

        return true;
    }

    /****************************************************************/
    bool close()
    {
        rpcPort.close();
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
        return true;
    }

    /****************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        SolverParameters p=solver.getSolverParameters();
        Bottle cmd=command;

        if (cmd.get(0).isString())
        {
            p.setMode(cmd.get(0).asString().c_str());
            cmd=cmd.tail();
        }

        if (cmd.size()<8)
        {
            yError("wrong target size!");
            reply.addVocab(Vocab::encode("nack"));
            return true;
        }
        
        p.torso_heave=cmd.get(0).asDouble();
        p.lower_arm_heave=cmd.get(1).asDouble();

        Vector xd(3),ud(3);
        xd[0]=cmd.get(2).asDouble();
        xd[1]=cmd.get(3).asDouble();
        xd[2]=cmd.get(4).asDouble();
        ud[0]=cmd.get(5).asDouble();
        ud[1]=cmd.get(6).asDouble();
        ud[2]=cmd.get(7).asDouble();

        double n=norm(ud);
        Vector ud_=(1.0/n)*ud;
        ud_.push_back(n);
        Matrix Hd=axis2dcm(ud_);
        Hd(0,3)=xd[0];
        Hd(1,3)=xd[1];
        Hd(2,3)=xd[2];

        solver.setSolverParameters(p);
        solver.setInitialGuess(q);
        solver.ikin(Hd,q);

        reply.addVocab(Vocab::encode("ack"));
        reply.addList().read(q);

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

