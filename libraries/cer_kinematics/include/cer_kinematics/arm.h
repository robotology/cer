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

#include <deque>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <cer_kinematics/utils.h>

namespace cer {
namespace kinematics {

/**
 * Class to handle direct and inverse kinematics of the robot 
 * arm. 
 * 
 * @author Ugo Pattacini
 */
class ArmSolver : public Solver
{
protected:
    ArmParameters armParameters;
    SolverParameters slvParameters;
    yarp::sig::Vector q0;
    yarp::sig::Vector zL;
    yarp::sig::Vector zU;
    yarp::sig::Vector lambda;
    int curMode;

    friend class ArmCommonNLP;

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
    virtual const ArmParameters& getArmParameters() const
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
    virtual const SolverParameters& getSolverParameters() const
    {
        return slvParameters;
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


/**
 * Class to compute useful information of the arm, such as CoMs,
 * and stability margin.
 * 
 * @author Ugo Pattacini
 */
class ArmCOM
{
    ArmSolver &solver;
    std::deque<yarp::sig::Vector> supPolygon;
    std::deque<yarp::sig::Vector> relComs;
    yarp::sig::Vector weights;
    double weight_tot;
    
    ArmCOM();  // not implemented

public:
    /**
     * Constructor.
     * 
     * @param solver_           the arm solver.
     * @param external_weight   the weight [Kg] attached to the 
     *                          hand.
     * @param floor_z           the z-coordinate of the floor in the 
     *                          root frame.
     */
    ArmCOM(ArmSolver &solver_, const double external_weight=0.0,
           const double floor_z=-0.16);

    /**
     * CoM computation.
     * 
     * @param q     the DOFs configuration ([m]-[deg]-[m]).
     * @param coms  a deque of vectors, accounting each for the 
     *              single CoM of the links; the last vector is the
     *              CoM of the whole structure.
     * @return true/false on success/failure.  
     */
    bool getCOMs(const yarp::sig::Vector &q, std::deque<yarp::sig::Vector> &coms) const;

    /**
     * CoM computation.
     * 
     * @param com       the homogeneous vector representing the CoM 
     *                  of the structure ([m]).
     * @param margin    the stability margin given as the distance 
     *                  from the border of the support polygon;
     *                  positive values mean we are inside the
     *                  polygon.
     * @return true/false on success/failure. 
     */
    bool getSupportMargin(const yarp::sig::Vector &com, double &margin) const;
};

}

}

#endif

