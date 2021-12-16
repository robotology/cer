/*
 * Copyright (C) 2013-2020 Fondazione Istituto Italiano di Tecnologia
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef CER_DOUBLE_LIDAR_H
#define CER_DOUBLE_LIDAR_H

#include <yarp/os/Property.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/Lidar2DDeviceBase.h>
#include <yarp/dev/Wrapper.h>

#include <string>
#include <functional>
#include <unordered_map>
#include <vector>
#include <mutex>

namespace cer {
    namespace dev {
        class cerDoubleLidar;
    }
}

class cer::dev::cerDoubleLidar :
    public yarp::dev::DeviceDriver,
    public yarp::dev::Lidar2DDeviceBase,
    public yarp::dev::IMultipleWrapper
{
    struct LaserPose_t
    {
       double x;
       double y;
       double z;
       double theta;
    };

    class LaserCfg_t
    {
       public:
       enum Laser{front=0, back=1};
       Laser laser;
       LaserPose_t pose;
       std::string sensorName;
       std::string typeOfDevice; //currently not used
       LaserCfg_t(Laser l){laser=l;}
       bool loadConfig(yarp::os::Searchable& config);
    };


public:
    cerDoubleLidar();
    virtual ~cerDoubleLidar();


    /**
     * Yarp interfaces start here
     */

    bool open(yarp::os::Searchable& config);
    bool close();
    
    //IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList &p) override;
    bool detachAll() override;
    
    //IRangefinder2D interface
    virtual bool getRawData          (yarp::sig::Vector& data, double* timestamp = nullptr) override;
    virtual bool getLaserMeasurement (std::vector<yarp::dev::LaserMeasurementData>& data, double* timestamp = nullptr) override;
    virtual bool setDistanceRange    (double min, double max) override;
    virtual bool setScanLimits        (double min, double max) override;
    virtual bool setHorizontalResolution      (double step) override;
    virtual bool setScanRate         (double rate) override;

public:
    //Lidar2DDeviceBase
    bool acquireDataFromHW() override final;

private:

    void calculate(int sensNum, double distance, double x_off, double y_off, double t_off);
    bool verifyLasersConfigurations(void);
    bool getLasersInterfaces(void);

    yarp::dev::PolyDriver * m_driver_laserFront = nullptr;
    yarp::dev::IRangefinder2D* m_ILaserFrontData = nullptr;

    yarp::dev::PolyDriver * m_driver_laserBack = nullptr;
    yarp::dev::IRangefinder2D* m_ILaserBackData = nullptr;

    bool m_inited;

    LaserCfg_t m_lFrontCfg;
    LaserCfg_t m_lBackCfg;
};

#endif //CER_DOUBLE_LIDAR_H
