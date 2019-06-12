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

#ifndef __CER_KINEMATICS_TRIPOD_H__
#define __CER_KINEMATICS_TRIPOD_H__

#include <yarp/sig/Vector.h>

#include <cer_kinematics/utils.h>

namespace cer {
namespace kinematics {

/**
 * Class to handle direct and inverse kinematics of the tripod 
 * mechanism. 
 * 
 * @author Ugo Pattacini
 */
class TripodSolver : public Solver
{
protected:
    TripodParameters parameters;
    yarp::sig::Vector lll0;

    friend class TripodNLP;

public:
    /**
     * Constructor.
     * 
     * @param params tripod parameters.
     * @param verb   integers greater than 0 enable successive 
     *               levels of verbosity (default=0).
     */
    TripodSolver(const TripodParameters &params=TripodParameters(),
                 const int verb=0);

    /**
     * Define parameters of tripod mechanism.
     * 
     * @param params tripod parameters.
     */
    virtual void setParameters(const TripodParameters &params)
    {
        parameters=params;
    }

    /**
     * Retrieve parameters of tripod mechanism.
     * 
     * @return tripod parameters.
     */
    virtual const TripodParameters& getParameters() const
    {
        return parameters;
    }

    /**
     * Specify the initial values of elongations.
     * 
     * @param lll0   initial elongations values ([m]). 
     * @return true/false on success/failure. 
     */
    virtual bool setInitialGuess(const yarp::sig::Vector &lll0);

    /**
     * Retrieve the initial guess used for the elongations.
     * 
     * @return the initial elongations values ([m]).
     */
    virtual yarp::sig::Vector getInitialGuess() const
    {
        return lll0;
    }

    /**
     * Forward Kinematics Law.
     * 
     * @param lll    the three elongations ([m]).
     * @param p      the computed 3d position of the platform center
     *               ([m]).
     * @param u      the 4 components orientation (axis,angle) given
     *               in [rad].
     * @return true/false on success/failure.
     */
    virtual bool fkin(const yarp::sig::Vector &lll,
                      yarp::sig::Vector &p, yarp::sig::Vector &u);

    /**
     * Forward Kinematics Law.
     * 
     * @param q      the three elongations ([m]).
     * @param H      the 4-by-4 homogeneous matrix of the platform 
     *               ([m]).
     * @param frame  not used.
     * @return true/false on success/failure.
     */
    virtual bool fkin(const yarp::sig::Vector &q, yarp::sig::Matrix &H,
                      const int frame=-1);

    /**
     * Inverse Kinematics Law with desired heave and orientation 
     * (axis*angles). 
     * 
     * @param zd     desired heave ([m]).
     * @param ud     the desired orientation given as 4 components 
     *               vector (axis,angle) expressed in [rad].
     * @param lll    the solved three elongations ([m]).
     * @return true/false on success/failure.
     */
    virtual bool ikin(const double zd, const yarp::sig::Vector &ud,
                      yarp::sig::Vector &lll, int *exit_code=NULL);

    /**
     * Inverse Kinematics Law.
     * 
     * @param Hd        the desired 4-by-4 homogeneous matrix 
     *                  representing the platform frame ([m]).
     * @param q         the solved elongations ([m]). 
     * @param exit_code pointer to solver's exit codes. 
     * @return true/false on success/failure.
     */
    virtual bool ikin(const yarp::sig::Matrix &Hd,
                      yarp::sig::Vector &q,
                      int *exit_code=NULL);

    /**
     * Destructor.
     */
    virtual ~TripodSolver() { }
};

}

}

#endif

