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

#include <cmath>

#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>

#include <cer_kinematics/private/helpers.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace cer::kinematics;

/****************************************************************/
TripodParametersExtended::TripodParametersExtended(const TripodParameters &parameters) :
                                                   TripodParameters(parameters)
{
    cos_alpha_max=cos(CTRL_DEG2RAD*alpha_max);

    z.resize(3,0.0);
    z[2]=1.0;

    Vector v(3,0.0);
    double theta=0.0;
    for (int i=0; i<3; i++)
    {            
        v[0]=r*cos(theta);
        v[1]=r*sin(theta);
        s.push_back(v);

        theta+=CTRL_DEG2RAD*120.0;
    }
}


/****************************************************************/
TripodState TripodNLPHelper::fkinHelper(const Ipopt::Number *x,
                                        const TripodParametersExtended &params,
                                        TripodState *internal)
{
    double q33=sqrt(27.0)*params.r/sqrt(12.0*(x[2]*x[2]-(x[0]+x[1])*x[2]+
                                        x[1]*x[1]-x[0]*x[1]+x[0]*x[0]+
                                        (27.0/12.0)*params.r*params.r));

    TripodState d;
    if (q33>=1.0)
    {
        d.n=params.z;
        d.u=0.0;
        d.p[0]=d.p[1]=0.0;
        d.p[2]=d.T(2,3)=x[0];
    }
    else
    {
        Vector v1=params.s[0]+x[0]*params.z;
        Vector v2=params.s[1]+x[1]*params.z;
        Vector v3=params.s[2]+x[2]*params.z;
        d.n=cross(v2-v1,v3-v1);
        d.n/=norm(d.n);

        double sin_theta=sqrt(1.0-q33*q33);
        d.u[0]=-d.n[1]/sin_theta;
        d.u[1]=d.n[0]/sin_theta;
        d.u[2]=0.0;
        d.u[3]=acos(q33);
        double tmp=(1.0-q33);
        double q11=tmp*d.u[0]*d.u[0]+q33;
        double q22=tmp*d.u[1]*d.u[1]+q33;
        double q21=tmp*d.u[0]*d.u[1];
        double q31=-sin_theta*d.u[1];
        double q32=sin_theta*d.u[0];
        double m1=params.r/q33*(-0.5*q11+1.5*q22);
        d.p[0]=params.r-m1*q11;
        d.p[1]=-m1*q21;
        d.p[2]=x[0]-m1*q31;            

        // transformation matrix
        d.T(0,0)=q11; d.T(0,1)=q21; d.T(0,2)=-q31; d.T(0,3)=d.p[0];
        d.T(1,0)=q21; d.T(1,1)=q22; d.T(1,2)=-q32; d.T(1,3)=d.p[1];
        d.T(2,0)=q31; d.T(2,1)=q32; d.T(2,2)=q33;  d.T(2,3)=d.p[2];
    }

    if (internal!=NULL)
        *internal=d;

    d.T=params.T0*d.T;
    d.n[0]=d.T(0,2); d.n[1]=d.T(1,2); d.n[2]=d.T(2,2);
    d.p[0]=d.T(0,3); d.p[1]=d.T(1,3); d.p[2]=d.T(2,3);
    d.u=dcm2axis(d.T);

    return d;
}

