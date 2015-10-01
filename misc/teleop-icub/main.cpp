// Copyright: (C)2015 iCub Facility-Istituto Italiano di Tecnologia
// Authors:Ugo Pattacini
// CopyPolicy:Released under the terms of the GNU GPL v2.0.

#include <string>
#include <cmath>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#define DEG2RAD     (M_PI/180.0)
#define RAD2DEG     (180.0/M_PI)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/**********************************************************/
class TeleOp: public RFModule
{
protected:
    PolyDriver         driver;
    ICartesianControl *iarm;
    
    BufferedPort<Bottle> inPort;
    RpcClient simPort;

    string type;
    int startup_context;

    enum {
        idle,
        trigger,
        running
    };

    int state;
    int triggerCnt;
    bool simulator;

    Matrix T,Tsim,H0;
    Bottle data;
    Vector pos0,rpy0;

public:
    /**********************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("teleop-icub")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();
        type=rf.check("type",Value("right")).asString().c_str();
        simulator=rf.check("simualtor");

        if (simulator)
        {
            simPort.open(("/"+name+"/simulator:rpc").c_str());
            if (Network::connect(simPort.getName().c_str(),"/icubSim/world"))
            {
                Bottle cmd,reply;
                cmd.addString("world");
                cmd.addString("mk");
                cmd.addString("sph");
                
                // radius
                cmd.addDouble(0.01);

                // position
                cmd.addDouble(0.0);
                cmd.addDouble(0.0);
                cmd.addDouble(0.0);
                
                // color
                cmd.addInt(1);
                cmd.addInt(0);
                cmd.addInt(0);

                // collision
                cmd.addString("false");

                simPort.write(cmd,reply);
            }
            else
            {
                yError("iCub simulator is not running!");
                simPort.close();
                return false;
            }
        }

        Property option("(device cartesiancontrollerclient)");
        option.put("remote",("/"+robot+"/cartesianController/"+type+"_arm").c_str());
        option.put("local",("/"+name+"/cartesianController/"+type+"_arm").c_str());

        if (!driver.open(option))
            return false;
        driver.view(iarm);

        iarm->storeContext(&startup_context);
        iarm->restoreContext(0);

        Vector dof(10,1.0);
        iarm->setDOF(dof,dof);

        inPort.open(("/"+name+"/geomagic:i").c_str());
        state=idle;
        triggerCnt=0;

        T=eye(4,4);
        Tsim=zeros(4,4);
        Tsim(0,1)=-1.0;
        Tsim(1,2)=1.0;  Tsim(1,3)=0.5976;
        Tsim(2,0)=-1.0; Tsim(2,3)=-0.026;
        Tsim(3,3)=1.0;

        H0=zeros(4,4);
        pos0.resize(3,0.0);
        rpy0.resize(3,0.0);

        for (int i=0; i<8; i++)
            data.addDouble(0.0);

        return true;
    }

    /**********************************************************/
    bool close()
    {
        iarm->stopControl();
        iarm->restoreContext(startup_context);
        driver.close();

        inPort.close();
        if (simulator)
            simPort.close();

        return true;
    }

    /**********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /**********************************************************/
    bool updateModule()
    {
        if (Bottle *data=inPort.read(false))
        {
            if (data->size()!=this->data.size())
                yWarning("wrong incoming data");
            else
                this->data=*data;
        }

        bool buttonsPressed=(data.get(6).asInt()!=0)||(data.get(7).asInt()!=0);
        if (buttonsPressed)
        {
            if (state==idle)
                state=trigger;
            else if (state==trigger)
            {
                if (++triggerCnt>10)
                {
                    pos0[0]=data.get(0).asDouble();
                    pos0[1]=data.get(1).asDouble();
                    pos0[2]=data.get(2).asDouble();

                    rpy0[0]=data.get(3).asDouble();
                    rpy0[1]=data.get(4).asDouble();
                    rpy0[2]=data.get(5).asDouble();

                    Vector x0,o0;
                    iarm->getPose(x0,o0);

                    H0=axis2dcm(o0);
                    H0(0,3)=x0[0];
                    H0(1,3)=x0[1];
                    H0(2,3)=x0[2];
                    
                    state=running;
                }
            }
            else
            {
                Vector rpy(3);
                rpy[0]=DEG2RAD*(data.get(3).asDouble()-rpy0[0]);
                rpy[1]=DEG2RAD*(data.get(4).asDouble()-rpy0[1]);
                rpy[2]=DEG2RAD*(data.get(5).asDouble()-rpy0[2]);

                Matrix H=rpy2dcm(rpy);
                
                H(0,3)=0.001*(data.get(0).asDouble()-pos0[0]);
                H(1,3)=0.001*(data.get(1).asDouble()-pos0[1]);
                H(2,3)=0.001*(data.get(2).asDouble()-pos0[2]);

                H=H0*T*H;
                iarm->goToPose(H.getCol(3),dcm2axis(H));

                if (simulator)
                {
                    H=Tsim*H;

                    Bottle cmd,reply;
                    cmd.addString("world");
                    cmd.addString("set");
                    cmd.addString("sph");

                    // obj #
                    cmd.addInt(1);

                    // position
                    cmd.addDouble(H(0,3));
                    cmd.addDouble(H(1,3));
                    cmd.addDouble(H(2,3));

                    simPort.write(cmd,reply);
                }
            }
        }
        else
        {
            state=idle;
            if (triggerCnt!=0)
                iarm->stopControl();
            triggerCnt=0;
        }

        return true;
    }
};


/**********************************************************/
int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not found!");
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    TeleOp teleop;
    return teleop.runModule(rf);
}



