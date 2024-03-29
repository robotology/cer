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
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>
#include <cer_kinematics/arm.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace cer::kinematics;


/****************************************************************/
class IKSolver : public RFModule
{    
    ArmSolver solver;
    RpcServer rpcPort;
    Vector q;

    /****************************************************************/
    bool getBounds(const string &remote, const string &local,
                   Matrix &lim) const
    {
        Property option;
        option.put("device","remote_controlboard");
        option.put("remote",remote);
        option.put("local",local);

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

        if (getBounds("/"+robot+"/torso","/cer_reaching-solver/"+arm_type+"/torso",lim))
        {
            p.torso.l_min=lim(0,0);
            p.torso.l_max=lim(0,1);
            p.torso.alpha_max=fabs(lim(1,1));

            iKinChain &chain=*p.upper_arm.asChain();
            chain[0].setMin(CTRL_DEG2RAD*lim(3,0));
            chain[0].setMax(CTRL_DEG2RAD*lim(3,1));

            yInfo("limits of %s part: heave=[%g,%g] [m], [pitch,roll]=[%g,%g] [deg], yaw=[%g,%g] [deg]",
                  ("/"+robot+"/torso").c_str(),p.torso.l_min,p.torso.l_max,
                  -p.torso.alpha_max,p.torso.alpha_max,lim(3,0),lim(3,1));
        }
        else
            return false;

        if (getBounds("/"+robot+"/"+arm_type+"_arm","/cer_reaching-solver/"+arm_type+"/"+arm_type+"_arm",lim))
        {
            iKinChain &chain=*p.upper_arm.asChain(); 
            for (int i=0; i<5; i++)
            {
                chain[1+i].setMin(CTRL_DEG2RAD*lim(i,0)); 
                chain[1+i].setMax(CTRL_DEG2RAD*lim(i,1));

                yInfo("limits of %s part: joint %d=[%g,%g] [deg]",
                      ("/"+robot+"/"+arm_type+"_arm").c_str(),i,
                      CTRL_RAD2DEG*chain[1+i].getMin(),
                      CTRL_RAD2DEG*chain[1+i].getMax());
            }

            p.lower_arm.l_min=lim(5,0);
            p.lower_arm.l_max=lim(5,1);
            p.lower_arm.alpha_max=fabs(lim(6,1));

            yInfo("limits of %s part: heave=[%g,%g] [m], [pitch,roll]=[%g,%g] [deg]",
                  ("/"+robot+"/"+arm_type+"_wrist").c_str(),p.lower_arm.l_min,p.lower_arm.l_max,
                  -p.lower_arm.alpha_max,p.lower_arm.alpha_max);
        }
        else
            return false;

        solver.setArmParameters(p);
        return true;
    }

public:
    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("cer")).asString();
        string arm_type=rf.check("arm-type",Value("left")).asString();
        bool get_bounds=(rf.check("get-bounds",Value("on")).asString()=="on");
        int verbosity=rf.check("verbosity",Value(0)).asInt32();

        SolverParameters p=solver.getSolverParameters();
        p.setMode("full_pose");
        p.torso_heave=0.0;
        p.lower_arm_heave=0.02;
        p.warm_start=true;

        solver.setArmParameters(ArmParameters(arm_type));
        solver.setSolverParameters(p);
        solver.setVerbosity(verbosity);

        if (get_bounds)
            if (!alignJointsBounds(robot,arm_type))
                return false;

        q.resize(3+solver.getArmParameters().upper_arm.getDOF()+3,0.0);
        rpcPort.open("/cer_reaching-solver/"+arm_type+"/rpc");
        attach(rpcPort);

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
    bool respond(const Bottle &cmd, Bottle &reply)
    {
        SolverParameters p=solver.getSolverParameters();
        reply.addVocab32(Vocab32::encode("nack"));

        if (cmd.check("get"))
        {
            reply.clear();
            reply.addVocab32(Vocab32::encode("ack"));

            Bottle &payLoad1=reply.addList();
            payLoad1.addString("parameters");

            Bottle &payLoad2=payLoad1.addList();

            Bottle &mode=payLoad2.addList();
            mode.addString("mode");
            mode.addString(p.getMode());

            Bottle &torso_heave=payLoad2.addList();
            torso_heave.addString("torso_heave");
            torso_heave.addFloat64(p.torso_heave);

            Bottle &lower_arm_heave=payLoad2.addList();
            lower_arm_heave.addString("lower_arm_heave");
            lower_arm_heave.addFloat64(p.lower_arm_heave);

            Bottle &tol=payLoad2.addList();
            tol.addString("tol");
            tol.addFloat64(p.tol);

            Bottle &constr_tol=payLoad2.addList();
            constr_tol.addString("constr_tol");
            constr_tol.addFloat64(p.constr_tol);

            return true;
        }
        
        if (cmd.check("parameters"))
        {
            if (Bottle *parameters=cmd.find("parameters").asList())
            {
                bool ack=false;
                if (parameters->check("mode"))
                {
                    string mode=parameters->find("mode").asString();
                    p.setMode(mode);
                    ack=true;
                }

                if (parameters->check("torso_heave"))
                {
                    p.torso_heave=parameters->find("torso_heave").asFloat64();
                    ack=true;
                }

                if (parameters->check("lower_arm_heave"))
                {
                    p.lower_arm_heave=parameters->find("lower_arm_heave").asFloat64();
                    ack=true;
                }

                if (parameters->check("tol"))
                {
                    p.tol=parameters->find("tol").asFloat64();
                    ack=true;
                }

                if (parameters->check("constr_tol"))
                {
                    p.constr_tol=parameters->find("constr_tol").asFloat64();
                    ack=true;
                }

                if (ack)
                {
                    solver.setSolverParameters(p);
                    reply.clear();
                    reply.addVocab32(Vocab32::encode("ack"));
                }
            }
        }

        if (Bottle *payLoad=cmd.find("q").asList())
        {
            int len=std::min(payLoad->size(),q.length());
            for (int i=0; i<len; i++)
                q[i]=payLoad->get(i).asFloat64();

            reply.clear();
            reply.addVocab32(Vocab32::encode("ack"));
        }

        if (Bottle *payLoad=cmd.find("target").asList())
        {
            if (payLoad->size()<7)
            {
                yError("wrong target size!");
                reply.clear();
                reply.addVocab32(Vocab32::encode("nack"));
                return true;
            }
            
            Vector xd(3),ud(4);
            xd[0]=payLoad->get(0).asFloat64();
            xd[1]=payLoad->get(1).asFloat64();
            xd[2]=payLoad->get(2).asFloat64();
            ud[0]=payLoad->get(3).asFloat64();
            ud[1]=payLoad->get(4).asFloat64();
            ud[2]=payLoad->get(5).asFloat64();
            ud[3]=payLoad->get(6).asFloat64();

            Matrix Hd=axis2dcm(ud);
            Hd.setSubcol(xd,0,3);

            solver.setSolverParameters(p);
            solver.setInitialGuess(q);
            solver.ikin(Hd,q);

            Matrix H; solver.fkin(q,H);
            Vector x=H.getCol(3).subVector(0,2);
            x=cat(x,dcm2axis(H));

            reply.clear();
            reply.addVocab32(Vocab32::encode("ack"));

            Bottle &payLoadJoints=reply.addList();
            payLoadJoints.addString("q");
            payLoadJoints.addList().read(q);

            Bottle &payLoadPose=reply.addList();
            payLoadPose.addString("x");
            payLoadPose.addList().read(x);
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

