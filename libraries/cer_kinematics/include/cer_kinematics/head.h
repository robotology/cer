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

#ifndef __CER_KINEMATICS_HEAD_H__
#define __CER_KINEMATICS_HEAD_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <cer_kinematics/utils.h>
#include <cer_kinematics/tripod.h>

namespace cer {
namespace kinematics {

/**
 * Class to handle direct and inverse kinematics of the robot 
 * head. 
 * 
 * @author Ugo Pattacini
 */
class HeadSolver : public Solver
{
protected:
    HeadParameters headParameters;
    SolverParameters slvParameters;
    TripodSolver torso;
    yarp::sig::Vector q0;

    friend class HeadNLP;

public:
    /**
     * Constructor.
     * 
     * @param headParams head parameters.
     * @param slvParams  solver parameters. 
     * @param verb       integers greater than 0 enable successive 
     *                   levels of verbosity (default=0).
     */
    HeadSolver(const HeadParameters &headParams=HeadParameters(),
               const SolverParameters &slvParams=SolverParameters(),
               const int verb=0);

    /**
     * Specify new verbosity level.
     * 
     * @param verb   the verbosity level.
     */
    virtual void setVerbosity(const int verb);

    /**
     * Define parameters of the head.
     * 
     * @param params arm parameters.
     */
    virtual void setHeadParameters(const HeadParameters &params);

    /**
     * Retrieve parameters of the head.
     * 
     * @return head parameters.
     */
    virtual const HeadParameters& getHeadParameters() const
    {
        return headParameters;
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
    virtual const SolverParameters& getSolverParameters() const
    {
        return slvParameters;
    }

    /**
     * Specify the initial DOFs values.
     * 
     * @param q0   initial DOFs values ([m]-[deg]). 
     * @return true/false on success/failure. 
     */
    virtual bool setInitialGuess(const yarp::sig::Vector &q0);

    /**
     * Retrieve the initial guess used for DOFs.
     * 
     * @return the initial DOFs values ([m]-[deg]).
     */
    virtual yarp::sig::Vector getInitialGuess() const
    {
        return q0;
    }

    /**
     * Forward Kinematics Law.
     * 
     * @param q      the DOFs values ([m]-[deg]).
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
     * @param xd        the desired 3D fixation point ([m]).
     * @param q         the solved head DOFs ([deg]). 
     * @param exit_code pointer to solver's exit codes.  
     * @return true/false on success/failure.
     */
    virtual bool ikin(const yarp::sig::Vector &xd, yarp::sig::Vector &q,
                      int *exit_code=NULL);

    /**
     * Destructor.
     */
    virtual ~HeadSolver() { }
};

}

}

#endif

