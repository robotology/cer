// Copyright: (C)2015 iCub Facility-Istituto Italiano di Tecnologia
// Authors:Ugo Pattacini
// CopyPolicy:Released under the terms of the GNU GPL v2.0.

#include <string>
#include <cmath>
#include <map>

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
    PolyDriver         drvCart;
    PolyDriver         drvHand;
    PolyDriver         drvGaze;
    ICartesianControl *iarm;
	IControlMode2     *imod;
    IPositionControl2 *ipos;
    IVelocityControl2 *ivel;
    IGazeControl      *igaze;
    
    BufferedPort<Bottle> inPort;
    RpcClient            simPort;

    string part;
    int startup_context;

    enum {
        idle,
        triggered,
        running
    };

    int s0,s1;
    int c0,c1;
    bool simulator;
    bool gaze;
    bool onlyXYZ;
    map<int,string> stateStr;

    Matrix T,Tsim;
    Bottle data;
    Vector pos0,rpy0;
	Vector x0,o0;

    VectorOf<int> joints,modes;
    Vector vels;

public:
    /**********************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("teleop-icub")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();
		double Tp2p=rf.check("Tp2p",Value(1.0)).asDouble();
        part=rf.check("part",Value("right_arm")).asString().c_str();
        simulator=rf.check("simulator",Value("off")).asString()=="on";
        gaze=rf.check("gaze",Value("off")).asString()=="on";

        if (simulator)
        {
            simPort.open(("/"+name+"/simulator:rpc").c_str());
            if (!Network::connect(simPort.getName().c_str(),"/icubSim/world"))
            {
                yError("iCub simulator is not running!");
                simPort.close();
                return false;
            }
        }

        if (gaze)
        {
            Property optGaze("(device gazecontrollerclient)");
            optGaze.put("remote","/iKinGazeCtrl");
            optGaze.put("local",("/"+name+"/gaze").c_str());
            if (!drvGaze.open(optGaze))
            {
                simPort.close();
                return false;
            }
            drvGaze.view(igaze);
        }

        Property optCart("(device cartesiancontrollerclient)");
        optCart.put("remote",("/"+robot+"/cartesianController/"+part).c_str());
        optCart.put("local",("/"+name+"/cartesianController/"+part).c_str());
        if (!drvCart.open(optCart))
		{
            if (simulator)
			    simPort.close();
            if (gaze)
                drvGaze.close();
            return false;
		}
        drvCart.view(iarm);

        Property optHand("(device remote_controlboard)");
        optHand.put("remote",("/"+robot+"/"+part).c_str());
        optHand.put("local",("/"+name+"/"+part).c_str());
        if (!drvHand.open(optHand))
        {
            if (simulator)
                simPort.close();
            if (gaze)
                drvGaze.close();
            drvCart.close();
            return false;
        }
		drvHand.view(imod);
        drvHand.view(ipos);
        drvHand.view(ivel);

        iarm->storeContext(&startup_context);
        iarm->restoreContext(0);

        Vector dof(10,1.0);
        iarm->setDOF(dof,dof);
		iarm->setTrajTime(Tp2p);
        
        Vector accs,poss;
        for (int i=0; i<9; i++)
        {
            joints.push_back(7+i);
            modes.push_back(VOCAB_CM_POSITION);
            accs.push_back(1e9);
            vels.push_back(100.0);
            poss.push_back(0.0);
        }
        poss[0]=40.0;
		poss[1]=90.0;
        
        imod->setControlModes(joints.size(),joints.getFirst(),modes.getFirst());
        ipos->setRefAccelerations(joints.size(),joints.getFirst(),accs.data());
        ipos->setRefSpeeds(joints.size(),joints.getFirst(),vels.data());
        ipos->positionMove(joints.size(),joints.getFirst(),poss.data());

        joints.clear();
        modes.clear();
        vels.clear();
        for (int i=2; i<9; i++)
        {
            joints.push_back(7+i);
            modes.push_back(VOCAB_CM_VELOCITY);
            vels.push_back(40.0);
        }

        inPort.open(("/"+name+"/geomagic:i").c_str());
        s0=s1=idle;
        c0=c1=0;
        onlyXYZ=true;
        
        stateStr[idle]="idle";
        stateStr[triggered]="triggered";
        stateStr[running]="running";

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

		if (simulator)
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

        return true;
    }

    /**********************************************************/
    bool close()
    {
        iarm->stopControl();
        iarm->restoreContext(startup_context);
        drvCart.close();

        ivel->stop(joints.size(),joints.getFirst());
        for (size_t i=0; i<modes.size(); i++)
            modes[i]=VOCAB_CM_POSITION;
        imod->setControlModes(joints.size(),joints.getFirst(),modes.getFirst());
        drvHand.close();

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

        if (gaze)
        {
            igaze->stopControl();
            drvGaze.close();
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
    void reachingHandler(const bool b)
    {
        if (b)
        {
            if (s0==idle)
                s0=triggered;
            else if (s0==triggered)
            {
                if (++c0*getPeriod()>0.1)
                {
                    pos0[0]=data.get(0).asDouble();
                    pos0[1]=data.get(1).asDouble();
                    pos0[2]=data.get(2).asDouble();

                    rpy0[0]=data.get(3).asDouble();
                    rpy0[1]=data.get(4).asDouble();
                    rpy0[2]=data.get(5).asDouble();

                    iarm->getPose(x0,o0);
                    s0=running;
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

                Matrix Rd;
                if (onlyXYZ)
                    Rd=axis2dcm(o0);
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

                if (gaze)
                    igaze->lookAtFixationPoint(xd);

                yInfo("going to (%s) (%s)",
                      xd.toString(3,3).c_str(),od.toString(3,3).c_str());

                if (simulator)
                    updateSim(xd);
            }
        }
        else
        {
            if (s0==triggered)
                onlyXYZ=!onlyXYZ;

            if (c0!=0)
            {
                iarm->stopControl();
                if (simulator)
                {
                    Vector x,o;
                    iarm->getPose(x,o);
                    updateSim(x);
                }
                if (gaze)
                    igaze->stopControl();
            }

            s0=idle;
            c0=0;
        }
    }

    /**********************************************************/
    void handHandler(const bool b)
    {
        if (b)
        {
            if (s1==idle)
                s1=triggered;
            else if (s1==triggered)
            {
                if (++c1*getPeriod()>0.1)
				{
					imod->setControlModes(joints.size(),joints.getFirst(),modes.getFirst());
                    s1=running;
				}
            }
            else
                ivel->velocityMove(joints.size(),joints.getFirst(),vels.data());
        }
        else
        {
            if (s1==triggered)
                vels=-1.0*vels;

            if (c1!=0)
                ivel->stop(joints.size(),joints.getFirst());

            s1=idle;
            c1=0;
        }
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

        bool b0=(data.get(6).asInt()!=0);
        bool b1=(data.get(7).asInt()!=0);

        reachingHandler(b0);
        handHandler(b1);

        yInfo("[reaching=%s; pose=%s;] [hand=%s; movement=%s;]",
              stateStr[s0].c_str(),onlyXYZ?"xyz":"full",
              stateStr[s1].c_str(),vels[0]>0.0?"closing":"opening");

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

