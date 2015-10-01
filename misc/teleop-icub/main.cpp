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

    string part;
    int startup_context;

    enum {
        idle,
        trigger,
        running
    };

    int state;
    int triggerCnt;
    bool simulator;
    bool onlyXYZ;

    Matrix T,Tsim;
    Bottle data;
    Vector pos0,rpy0;
	Vector x0,o0;

public:
    /**********************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("teleop-icub")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();
		double Tp2p=rf.check("Tp2p",Value(1.0)).asDouble();
        part=rf.check("part",Value("right_arm")).asString().c_str();
        simulator=rf.check("simulator");
        onlyXYZ=rf.check("onlyXYZ");

        if (simulator)
        {
            simPort.open(("/"+name+"/simulator:rpc").c_str());
            if (Network::connect(simPort.getName().c_str(),"/icubSim/world"))
            {
                Bottle cmd,reply;
                cmd.addString("world");
                cmd.addString("mk");
                cmd.addString("ssph");
                
                // radius
                cmd.addDouble(0.02);

                // position
                cmd.addDouble(0.0);
                cmd.addDouble(0.0);
                cmd.addDouble(0.0);
                
                // color
                cmd.addInt(1);
                cmd.addInt(0);
                cmd.addInt(0);

                // collision
                cmd.addString("FALSE");

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
        option.put("remote",("/"+robot+"/cartesianController/"+part).c_str());
        option.put("local",("/"+name+"/cartesianController/"+part).c_str());

        if (!driver.open(option))
            return false;
        driver.view(iarm);

        iarm->storeContext(&startup_context);
        iarm->restoreContext(0);

        Vector dof(10,1.0);
        iarm->setDOF(dof,dof);
		iarm->setTrajTime(Tp2p);

        inPort.open(("/"+name+"/geomagic:i").c_str());
        state=idle;
        triggerCnt=0;

        T=zeros(4,4);
		T(0,1)=1.0;
		T(1,2)=1.0;
		T(2,0)=1.0;
		T(3,3)=1.0;
		T=SE3inv(T);
        
		Tsim=zeros(4,4);
        Tsim(0,1)=-1.0;
        Tsim(1,2)=1.0;  Tsim(1,3)=0.5976;
        Tsim(2,0)=-1.0; Tsim(2,3)=-0.026;
        Tsim(3,3)=1.0;

        pos0.resize(3,0.0);
        rpy0.resize(3,0.0);

		x0.resize(3,0.0);
        o0.resize(4,0.0);

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
		{
			Bottle cmd,reply;
			cmd.addString("world");
			cmd.addString("del");
			cmd.addString("all");
			simPort.write(cmd,reply);

            simPort.close();
		}

        return true;
    }

    /**********************************************************/
    void updateSim(const Vector &c_)
    {
        if ((c_.length()!=3) && (c_.length()!=4))
            return;

        Vector c=c_;        
        if (c.length()==3)
            c.push_back(1.0);
        c[3]=1.0;

        c=Tsim*c;

        Bottle cmd,reply;
        cmd.addString("world");
        cmd.addString("set");
        cmd.addString("ssph");

        // obj #
        cmd.addInt(1);

        // position
        cmd.addDouble(c[0]);
        cmd.addDouble(c[1]);
        cmd.addDouble(c[2]);

        simPort.write(cmd,reply);
    }

    /**********************************************************/
    double getPeriod()
    {
        return 0.01;
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
                    
                    iarm->getPose(x0,o0);
                    state=running;
                }
            }
            else
            {
                Vector xd(4,0.0);
                xd[0]=0.001*(data.get(0).asDouble()-pos0[0]);
                xd[1]=0.001*(data.get(1).asDouble()-pos0[1]);
                xd[2]=0.001*(data.get(2).asDouble()-pos0[2]);
                xd[3]=1.0;

				Matrix H0=eye(4,4);
                H0(0,3)=x0[0];
                H0(1,3)=x0[1];
                H0(2,3)=x0[2];

                xd=H0*(T*xd);
				
                Matrix Rd=zeros(3,3);
                if (onlyXYZ)
                    Rd(0,0)=Rd(2,1)=Rd(1,2)=-1.0;
                else
                {
                    Vector rpy(3);
                    rpy[0]=data.get(3).asDouble()-rpy0[0];
                    rpy[1]=data.get(4).asDouble()-rpy0[1];
                    rpy[2]=data.get(5).asDouble()-rpy0[2];

                    Vector ax(4,0.0),ay(4,0.0),az(4,0.0);
                    ax[0]=1.0; ax[3]=rpy[2];
                    ay[1]=1.0; ay[3]=rpy[1]*((part=="left_arm")?1.0:-1.0);
                    az[2]=1.0; az[3]=rpy[0]*((part=="left_arm")?1.0:-1.0);

                    Rd=axis2dcm(o0)*axis2dcm(ax)*axis2dcm(ay)*axis2dcm(az);
                }

                Vector od=dcm2axis(Rd);
				iarm->goToPose(xd,od);
				
				yInfo("going to (%s) (%s)",
					  xd.toString(3,3).c_str(),od.toString(3,3).c_str());

                if (simulator)
					updateSim(xd);
            }
        }
        else
        {
            state=idle;
            if (triggerCnt!=0)
			{
                iarm->stopControl();
				if (simulator)
				{
					Vector x,o;
					iarm->getPose(x,o);
					updateSim(x);
				}
			}
            triggerCnt=0;
        }

		yInfo("state=%d;",state);

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

