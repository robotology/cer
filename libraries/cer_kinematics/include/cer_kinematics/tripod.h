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

namespace cer_kinematics
{

/**
 * Class to handle direct and inverse kinematics of the tripod 
 * mechanism. 
 * 
 * @author Ugo Pattacini
 */
class TripodSolver
{
protected:
    TripodParameters parameters;
    yarp::sig::Vector lll0;
    int verbosity;

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
     * @param u      the 3 components orientation (axis*angle) given
     *               in [rad].
     * @return true/false on success/failure.
     */
    virtual bool fkin(const yarp::sig::Vector &lll,
                      yarp::sig::Vector &p, yarp::sig::Vector &u);

    /**
     * Forward Kinematics Law.
     * 
     * @param lll    the three elongations ([m]).
     * @param hpr    the computed heave ([m]), pitch ([deg]) and 
     *               roll ([deg]).
     * @return true/false on success/failure.
     */
    virtual bool fkin(const yarp::sig::Vector &lll,
                      yarp::sig::Vector &hpr);

    /**
     * Inverse Kinematics Law with desired heave and orientation 
     * (axis*angles). 
     * 
     * @param zd     desired heave ([m]).
     * @param ud     the desired orientation given as 3 components 
     *               vector (axis*angle) expressed in [rad].
     * @param lll    the solved three elongations ([m]).
     * @return true/false on success/failure.
     */
    virtual bool ikin(const double zd, const yarp::sig::Vector &ud,
                      yarp::sig::Vector &lll, int *exit_code=NULL);

    /**
     * Inverse Kinematics Law with heave-pitch-roll.
     * 
     * @param hpr    the desired heave ([m]), pitch ([deg]) and roll
     *               ([deg]) to solve for.
     * @param lll    the solved three elongations ([m]).
     * @return true/false on success/failure.
     */
    virtual bool ikin(const yarp::sig::Vector &hpr,
                      yarp::sig::Vector &lll,
                      int *exit_code=NULL);

    /**
     * Destructor.
     */
    virtual ~TripodSolver() { }
};

}

#endif

