
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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <deque>

#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <cer_kinematics/arm.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace cer_kinematics;


/****************************************************************/
class ComData
{
    ArmSolver &solver;
    deque<Vector> supPolygon;
    deque<Vector> relComs;
    Vector weights;
    double weight_tot;
    
    ComData();  // not implemented

public:
    /****************************************************************/
    ComData(ArmSolver &solver_, const double external_weight,
            const double floor_z) : solver(solver_)
    {
        string arm_type=solver.getArmParameters().upper_arm.getType();

        // CoM absolute positions
        // mobilebase_lowertorso
        Vector tmp(4,1.0);
        tmp[0]=0.026;
        tmp[1]=0.0;
        tmp[2]=-0.415;
        relComs.push_back(tmp);
        
        // head
        tmp[0]=0.046;
        tmp[1]=0.0;
        tmp[2]=0.521;
        relComs.push_back(tmp);

        // l0
        tmp[0]=0.06;
        tmp[1]=0.0;
        tmp[2]=0.18;
        relComs.push_back(tmp);
       
        // l3
        tmp[0]=-0.01;
        tmp[1]=0.191*(arm_type=="left")?1.0:-1.0;
        tmp[2]=0.215;
        relComs.push_back(tmp);
       
        // l5
        tmp[0]=-0.01;
        tmp[1]=0.191*(arm_type=="left")?1.0:-1.0;
        tmp[2]=-0.089;
        relComs.push_back(tmp);
        
        // hand
        tmp[0]=0.0;
        tmp[1]=0.0;
        tmp[2]=0.0;
        relComs.push_back(tmp);

        // same order as per relComs
        weights.push_back(25.0);
        weights.push_back(2.5);
        weights.push_back(7.5);
        weights.push_back(1.5);
        weights.push_back(1.0);
        weights.push_back(0.6+external_weight);
        weight_tot=dot(weights,Vector(weights.length(),1.0));

        // compute CoMs relative positions wrt q0
        Vector q0(12,0.0);

        Matrix frame_l0;
        solver.fkin(q0,frame_l0,3+0);
        relComs[1]=SE3inv(frame_l0)*relComs[1];
        relComs[2]=SE3inv(frame_l0)*relComs[2];

        Matrix frame_l3;
        solver.fkin(q0,frame_l3,3+3);
        relComs[3]=SE3inv(frame_l3)*relComs[3];

        Matrix frame_l5;
        solver.fkin(q0,frame_l5,3+5);
        relComs[4]=SE3inv(frame_l5)*relComs[4];

        // support polygon    
        Vector c1(4,1.0);
        c1[0]=0.152; c1[1]=0.09;
        Vector c2(4,1.0);
        c2[0]=-0.17; c2[1]=0.0;
        Vector c3(4,1.0);
        c3[0]=c1[0]; c3[1]=-c1[1];
        Vector w1(4,1.0);
        w1[0]=0.0; w1[1]=0.17;
        Vector w2(4,1.0);
        w2[0]=w1[1]; w2[1]=-w1[1];

        c1[2]=c2[2]=c3[2]=w1[2]=w2[2]=floor_z;

        // apply caster reduction
        double caster_red=0.023;
        double r,theta;

        r=norm(c1)-caster_red;
        theta=atan2(c1[1],c1[0]);
        c1[0]=r*cos(theta);
        c1[1]=r*sin(theta);

        r=norm(c2)-caster_red;
        theta=atan2(c2[1],c2[0]);
        c2[0]=r*cos(theta);
        c2[1]=r*sin(theta);

        r=norm(c3)-caster_red;
        theta=atan2(c3[1],c3[0]);
        c3[0]=r*cos(theta);
        c3[1]=r*sin(theta);

        supPolygon.push_back(c1);
        supPolygon.push_back(c3);
        supPolygon.push_back(w2);
        supPolygon.push_back(c2);
        supPolygon.push_back(w1);
    }


    /****************************************************************/
    deque<Vector> getCOMs(const Vector &q) const
    {
        Matrix frame_l0;
        solver.fkin(q,frame_l0,3+0);

        Matrix frame_l3;
        solver.fkin(q,frame_l3,3+3);

        Matrix frame_l5;
        solver.fkin(q,frame_l5,3+5);

        Matrix frame_hand;
        solver.fkin(q,frame_hand);

        deque<Vector> coms;
        coms.push_back(relComs[0]);
        coms.push_back(frame_l0*relComs[1]);
        coms.push_back(frame_l0*relComs[2]);
        coms.push_back(frame_l3*relComs[3]);
        coms.push_back(frame_l5*relComs[4]);
        coms.push_back(frame_hand*relComs[5]);

        Vector com_tot(4,0.0);
        for (size_t i=0; i<coms.size(); i++)
            com_tot+=weights[i]*coms[i];
        
        com_tot/=weight_tot;
        com_tot[3]=1.0; // reinforce

        coms.push_back(com_tot);
        return coms;
    }

    /****************************************************************/
    double getSupportMargin(const Vector &com) const
    {
        double margin=std::numeric_limits<double>::max();
        for (size_t i=0; i<supPolygon.size(); i++)
        {
            const Vector &p0=supPolygon[i];
            const Vector &p1=supPolygon[(i+1)%supPolygon.size()];

            Vector d=p0-p1;
            Vector rot(4,0.0);
            rot[2]=1.0;
            rot[3]=(d[0]!=0.0)?atan2(d[1],d[0]):M_PI/2.0;
            Matrix R=SE3inv(axis2dcm(rot));

            Vector p0_rot=R*p0;
            if (p0_rot[1]<0.0)
            {
                rot[3]+=M_PI;
                R=SE3inv(axis2dcm(rot));
                p0_rot=R*p0;
            }

            Vector com_rot=R*com;
            margin=std::min(p0_rot[1]-com_rot[1],margin);
        }

        return (margin<0.0?0.0:margin);
    }

};

#endif

