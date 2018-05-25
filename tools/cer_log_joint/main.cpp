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
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/minJerkCtrl.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/****************************************************************/
class Controller : public RFModule
{
    PolyDriver driver;
    IControlMode     *imod;
    IControlLimits   *ilim;
    IEncodersTimed   *ienc;
    IPositionControl *ipos;
    IPositionDirect  *iposd;

    minJerkTrajGen *gen;    
    BufferedPort<Vector> outPort;

    string type;
    int joint,target;
    Vector bounds,qd;
    double Ts,T,t0;

public:
    /****************************************************************/
    Controller() : gen(NULL)
    {
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("cer")).asString();
        string part=rf.check("part",Value("left_upper_arm")).asString();
        type=rf.check("type",Value("stairs")).asString();
        joint=rf.check("joint",Value(0)).asInt();
        Ts=rf.check("Ts",Value(0.01)).asDouble();
        T=rf.check("T",Value(4.0)).asDouble();

        Property option;
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/"+part);
        option.put("local","/cer_log_joint/"+part); 
        if (!driver.open(option))
        {
            yError("Unable to connect to %s",("/"+robot+"/"+part).c_str());
            close();
            return false;
        }

        driver.view(imod);
        driver.view(ilim);
        driver.view(ienc);
        driver.view(ipos);
        driver.view(iposd);

        bounds.resize(2);
        ilim->getLimits(joint,&bounds[0],&bounds[1]);
        double tmp=0.1*(bounds[1]-bounds[0]);

        bounds[0]+=tmp;
        bounds[1]-=tmp;
        
        qd.resize(1);
        ienc->getEncoder(joint,&qd[0]);
        imod->setControlMode(joint,VOCAB_CM_POSITION_DIRECT);

        gen=new minJerkTrajGen(qd,Ts,T);
        outPort.open("/cer_log_joint:o");

        qd=bounds[target=0];
        t0=Time::now();

        return true;
    }

    /****************************************************************/
    bool close()
    {        
        if (!outPort.isClosed())
            outPort.close();

        if (driver.isValid())
        {
            ipos->stop(joint);
            driver.close();
        }

        delete gen;
        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return Ts;
    }

    /****************************************************************/
    bool updateModule()
    {
        double t=Time::now();
        if (t-t0>=T)
        {
            qd=bounds[target=!target];
            t0=t;
        }

        Vector ref;
        if (type=="stairs")
            ref=qd;
        else
        {
            gen->computeNextValues(qd); 
            ref=gen->getPos();
        }

        iposd->setPosition(joint,ref[0]);

        Vector &info=outPort.prepare();
        info.resize(2);
        info[0]=ref[0];
        ienc->getEncoder(joint,&info[1]);
        outPort.writeStrict();        

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

    Controller controller;
    return controller.runModule(rf);
}

