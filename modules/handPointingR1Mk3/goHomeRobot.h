/*
 * SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef GO_HOME_ROBOT_H
#define GO_HOME_ROBOT_H

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/ControlBoardInterfaces.h>

class GoHomeRobot : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
private:
    //Polydriver
    yarp::dev::PolyDriver           m_drivers[3];
    yarp::dev::IRemoteCalibrator*   m_iremcal[3];       //to command HomingWholePart to arms and head
    yarp::dev::IControlMode*        m_ictrlmode[3];     //to set the Position control mode
    yarp::dev::IPositionControl*    m_iposctrl[3];      //to retrieve the number of joints of each part

public:
    //Constructor/Distructor
    GoHomeRobot(){}
    ~GoHomeRobot(){}

    //Internal methods
    bool configure(yarp::os::ResourceFinder &rf);
    void backToHome();
    void backToHomePart(const std::string& part);
    void close();

    //Port inherited from TypedReaderCallback
    using TypedReaderCallback<yarp::os::Bottle>::onRead;
    void onRead(yarp::os::Bottle& b) override;
};

#endif //GO_HOME_ROBOT_H
