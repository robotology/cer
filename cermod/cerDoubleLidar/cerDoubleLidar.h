/*
 * Copyright (C) 2013-2020 Fondazione Istituto Italiano di Tecnologia
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef DOUBLELASERDEVICE_HH
#define DOUBLELASERDEVICE_HH

#include <yarp/os/Property.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/Lidar2DDeviceBase.h>
#include <yarp/dev/Wrapper.h>

#include <boost/shared_ptr.hpp>

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
       std::string fileCfgName;
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
    virtual bool getRawData(yarp::sig::Vector &data) override;
    virtual bool getLaserMeasurement(std::vector<yarp::dev::LaserMeasurementData> &data) override;
    virtual bool setDistanceRange    (double min, double max) override;
    virtual bool setScanLimits        (double min, double max) override;
    virtual bool setHorizontalResolution      (double step) override;
    virtual bool setScanRate         (double rate) override;

private:

    void calculate(int sensNum, double distance, double x_off, double y_off, double t_off);
    bool verifyLasersConfigurations(void);
    bool getLasersInterfaces(void);
    bool createLasersDevices(void);
    
    yarp::dev::PolyDriver * m_driver_laserFront = nullptr;
    yarp::dev::IRangefinder2D* m_dev_laserFront = nullptr;

    yarp::dev::PolyDriver * m_driver_laserBack = nullptr;
    yarp::dev::IRangefinder2D* m_dev_laserBack = nullptr;
    
    bool m_inited;
    bool m_onSimulator; //if true the device looks for front and back laser devices in gazebo, else creates them

    LaserCfg_t m_lFrontCfg;
    LaserCfg_t m_lBackCfg;
};

#endif //GAZEBOYARP_CONTROLBOARDDRIVER_HH
