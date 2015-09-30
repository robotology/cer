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

    string type;
    int startup_context;

    enum {
        idle,
        trigger,
        running
    };

    int state;
    int triggerCnt;

    Matrix T,H0;
    Bottle data;
    Vector pos0,rpy0;

public:
    /**********************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("teleop-icub")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();
        type=rf.check("type",Value("right")).asString().c_str();

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

        bool buttonsPressed=(data.get(6).asInt()!=0)&&(data.get(7).asInt()!=0);
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
        return 1;

    ResourceFinder rf;
    rf.configure(argc,argv);

    TeleOp teleop;
    return teleop.runModule(rf);
}



