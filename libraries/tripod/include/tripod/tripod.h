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

#include <yarp/sig/Vector.h>

namespace tripod
{

/**
 * Structure used to initialize a tripod mechanism.
 * 
 * @author Ugo Pattacini
 */
struct TripodParameters
{
    /**
     * the radius ([m]).
     */
    double r;

    /**
     * the minimum elongation ([m]).
     */
    double l_min;

    /**
     * the minimum elongation ([m]).
     */
    double l_max;

    /**
     * the maximum permitted bending angle ([deg]).
     */
    double alpha_max;

    /**
     * Constructor.
     * 
     * @param r_        the radius ([m]).
     * @param l_min_    the minimum elongation ([m]).
     * @param l_max_    the maximum elongation ([m]).
     * @param alpha_max the maximum permitted bending angle ([deg]).
     */
    TripodParameters(const double r_=0.09, const double l_min_=0.0,
                     const double l_max_=0.2, const double alpha_max=30.0);
};


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
    yarp::sig Vector lll0;
    int verbosity;

public:
    /**
     * Constructor.
     * 
     * @param params tripod parameters.
     * @param verb   integers greater than 0 enable successive levels
     *               ofverbosity (default=0).
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
    virtual TripodParameters getParameters() const
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
    bool setInitialGuess(const yarp::sig::Vector lll0);

    /**
     * Retrieve the initial guess used for the elongations.
     * 
     * @return the initial elongations values ([m]).
     */
    yarp::sig::Vector getInitialGuess() const
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
                      yarp::sig::Vector &p, yarp::sig::Vector &u) const;

    /**
     * Forward Kinematics Law.
     * 
     * @param lll    the three elongations ([m]).
     * @param hpr    the computed height ([m]), pitch ([deg]) and 
     *               roll ([deg]).
     * @return true/false on success/failure.
     */
    virtual bool fkin(const yarp::sig::Vector &lll,
                      yarp::sig::Vector &hpr) const;

    /**
     * Inverse Kinematics Law with desired height and orientation (axis*angles).
     * 
     * @param zd     desired height ([m]).
     * @param ud     the desired orientation given as 3 components 
     *               vector (axis*angle) expressed in [rad].
     * @param lll    the solved three elongations ([m]).
     * @return true/false on success/failure.
     */
    virtual bool ikin(const double zd, const yarp::sig::Vector &ud,
                      yarp::sig::Vector &lll) const;

    /**
     * Inverse Kinematics Law with height-pitch-roll.
     * 
     * @param hpr    the desired height ([m]), pitch ([deg]) and 
     *               roll ([deg]) to solve for.
     * @param lll    the solved three elongations ([m]).
     * @return true/false on success/failure.
     */
    virtual bool ikin(const yarp::sig::Vector &hpr,
                      yarp::sig::Vector &lll) const;

    /**
     * Destructor.
     */
    virtual ~TripodSolver() { }
};

}


