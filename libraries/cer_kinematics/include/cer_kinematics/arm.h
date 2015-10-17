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

#ifndef __CER_KINEMATICS_ARM_H__
#define __CER_KINEMATICS_ARM_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <cer_kinematics/utils.h>

namespace cer_kinematics
{

/**
 * Class to handle direct and inverse kinematics of the robot 
 * arm. 
 * 
 * @author Ugo Pattacini
 */
class ArmSolver
{
protected:
    ArmParameters armParameters;
    SolverParameters slvParameters;
    yarp::sig::Vector q0;
    yarp::sig::Vector z_L;
    yarp::sig::Vector z_U;
    int verbosity;

public:
    /**
     * Constructor.
     * 
     * @param armParams arm parameters. 
     * @param slvParams solver parameters. 
     * @param verb      integers greater than 0 enable successive 
     *                  levels of verbosity (default=0).
     */
    ArmSolver(const ArmParameters &armParams=ArmParameters(),
              const SolverParameters &slvParams=SolverParameters(),
              const int verb=0);

    /**
     * Define parameters of the arm.
     * 
     * @param params arm parameters.
     */
    virtual void setArmParameters(const ArmParameters &params)
    {
        armParameters=params;
    }

    /**
     * Retrieve parameters of the arm.
     * 
     * @return arm parameters.
     */
    virtual ArmParameters getArmParameters() const
    {
        return armParameters;
    }

    /**
     * Define parameters of the solver.
     * 
     * @param params solver parameters.
     */
    virtual void setSolverParameters(const SolverParameters &params)
    {
        slvParameters=params;
    }

    /**
     * Retrieve parameters of the solver.
     * 
     * @return solver parameters.
     */
    virtual SolverParameters getSolverParameters() const
    {
        return slvParameters;
    }

    /**
     * Specify new verbosity level.
     * 
     * @param verb   the verbosity level.
     */
    virtual void setVerbosity(const int verb)
    {
        verbosity=verb;
    }

    /**
     * Retrieve verbosity level.
     * 
     * @return the current verbosity level.
     */
    virtual int getVerbosity() const
    {
        return verbosity;
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
     * @param H      the 4-by-4 homogeneous matrix of the 
     *               end-effector frame ([m]).
     * @return true/false on success/failure.
     */
    virtual bool fkin(const yarp::sig::Vector &q, yarp::sig::Matrix &H);

    /**
     * Inverse Kinematics Law.
     * 
     * @param Hd     the desired 4-by-4 homogeneous matrix 
     *               representing the end-effector frame ([m]).
     * @param q      the solved DOFs ([m]-[deg]-[m]).
     * @return true/false on success/failure.
     */
    virtual bool ikin(const yarp::sig::Matrix &Hd, yarp::sig::Vector &q,
                      int *exit_code=NULL);

    /**
     * Destructor.
     */
    virtual ~ArmSolver() { }
};

}

#endif

