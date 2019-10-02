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

#ifndef __CER_KINEMATICS_MOBILE_ARM_H__
#define __CER_KINEMATICS_MOBILE_ARM_H__

#include <deque>
#include <vector>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <cer_mobile_kinematics/mobile_utils.h>

namespace cer {
namespace mobile_kinematics {

/**
 * Class to handle direct and inverse kinematics of the robot 
 * arm and mobile base.
 * 
 * @author Ugo Pattacini
 */
class MobileArmSolver : public cer::kinematics::Solver
{
protected:
    cer::kinematics::ArmParameters armParameters;
    MobileSolverParameters slvParameters;
    bool domain_constr;
    yarp::sig::Vector domainPoly;
    yarp::sig::Vector q0;
    yarp::sig::Vector zL;
    yarp::sig::Vector zU;
    yarp::sig::Vector lambda;
    int curMode;

    friend class MobileArmCommonNLP;

    int computeMode() const;

public:
    /**
     * Constructor.
     * 
     * @param armParams arm parameters. 
     * @param slvParams solver parameters. 
     * @param verb      integers greater than 0 enable successive 
     *                  levels of verbosity (default=0).
     */
    MobileArmSolver(const cer::kinematics::ArmParameters &armParams=cer::kinematics::ArmParameters(),
              const MobileSolverParameters &slvParams=MobileSolverParameters(),
              const yarp::sig::Vector &domain=yarp::sig::Vector(),
              const int verb=0);

    /**
     * Define parameters of the arm.
     * 
     * @param params arm parameters.
     */
    virtual void setArmParameters(const cer::kinematics::ArmParameters &params)
    {
        armParameters=params;
    }

    /**
     * Retrieve parameters of the arm.
     * 
     * @return arm parameters.
     */
    virtual const cer::kinematics::ArmParameters& getArmParameters() const
    {
        return armParameters;
    }

    /**
     * Define parameters of the solver.
     * 
     * @param params solver parameters.
     */
    virtual void setSolverParameters(const MobileSolverParameters &params)
    {
        slvParameters=params;
    }

    /**
     * Retrieve parameters of the solver.
     *
     * @return solver parameters.
     */
    virtual const MobileSolverParameters& getSolverParameters() const
    {
        return slvParameters;
    }

    /**
     * Define the mobile base domain polygon.
     *
     * @param poly mobile base domain polygon.
     */
    virtual void setDomain(const yarp::sig::Vector &domain)
    {
        if (domain.size()>5)
        {
            domainPoly=domain;
            domain_constr=true;
            if(domainPoly.size()%2!=0)
                domainPoly.pop_back();
        }
        else
        {
            domainPoly.clear();
            domain_constr=false;
        }
    }

    /**
     * Retrieve the mobile base domain polygon.
     * 
     * @return mobile base domain polygon.
     */
    virtual const yarp::sig::Vector& getDomain() const
    {
        return domainPoly;
    }

    /**
     * Specify the initial DOFs values.
     * 
     * @param q0   initial DOFs values ([m]-[deg]-[m]). 
     * @return true/false on success/failure. 
     */
    virtual bool setInitialGuess(const yarp::sig::Vector &q0);

    /**
     * Retrieve the initial guess used for DOFs.
     * 
     * @return the initial DOFs values ([m]-[deg]-[m]).
     */
    virtual yarp::sig::Vector getInitialGuess() const
    {
        return q0;
    }

    /**
     * Forward Kinematics Law.
     * 
     * @param q      the DOFs values ([m]-[deg]-[m]).
     * @param H      the 4-by-4 homogeneous matrix of the specified 
     *               frame ([m]).
     * @param frame  specify the DOF number whose frame is returned. 
     *               Thus, frame is in [0...nDOF-1]; negative
     *               numbers account for the end-effector frame.
     * @return true/false on success/failure.
     */
    virtual bool fkin(const yarp::sig::Vector &q, yarp::sig::Matrix &H,
                      const int frame=-1);

    /**
     * Inverse Kinematics Law.
     * 
     * @param Hd        the list of desired 4-by-4 homogeneous matrices
     *                  representing the end-effector frame ([m]).
     * @param q         the solved DOFs ([m]-[deg]-[m]). 
     * @param exit_code pointer to solver's exit codes. 
     * @return true/false on success/failure.
     */
    virtual bool ikin(const std::vector<yarp::sig::Matrix> &Hd, yarp::sig::Vector &q,
                      int *exit_code=NULL);

    virtual bool ikin(const yarp::sig::Matrix &Hd, yarp::sig::Vector &q,
                      int *exit_code=NULL);

    virtual double getManip(const yarp::sig::Vector &q, bool add_joint_limit);

    /**
     * Destructor.
     */
    virtual ~MobileArmSolver() { }
};

}

}

#endif

