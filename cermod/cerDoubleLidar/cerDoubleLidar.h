/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
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
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/Wrapper.h>

#include <boost/shared_ptr.hpp>



#include <string>
#include <functional>
#include <unordered_map>
#include <vector>



namespace cer {
    namespace dev {
        class cerDoubleLidar;
    }
}

struct Point3d_t
{
    double x;
    double y;
    double z;
};

class LaserCfg_t
{
public:
    enum Laser{front=0, back=1};
    Laser laser;
    Point3d_t pose;
    std::string fileCfgName;
    std::string sensorName;
    std::string typeOfDevice; //currntly not used
    LaserCfg_t(Laser l){laser=l;}
    bool loadConfig(yarp::os::Searchable& config);

};

class cer::dev::cerDoubleLidar :
    public yarp::dev::DeviceDriver,
    public yarp::dev::IRangefinder2D,
    public yarp::dev::IMultipleWrapper
{
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
    virtual bool getDeviceStatus     (Device_status &status) override;
    virtual bool getDeviceInfo       (std::string &device_info) override; //
    virtual bool getDistanceRange    (double& min, double& max) override;
    virtual bool setDistanceRange    (double min, double max) override;
    virtual bool getScanLimits        (double& min, double& max) override;
    virtual bool setScanLimits        (double min, double max) override;
    virtual bool getHorizontalResolution      (double& step) override;
    virtual bool setHorizontalResolution      (double step) override;
    virtual bool getScanRate         (double& rate) override;
    virtual bool setScanRate         (double rate) override;

private:

    void calculate(int sensNum, double distance, bool front, int &newSensNum, double &newdistance);
    bool verifyLasersConfigurations(void);
    bool getLasersInterfaces(void);
    bool createLasersDevices(void);
    
    yarp::dev::PolyDriver * m_driver_laserFront;
    yarp::dev::IRangefinder2D* m_dev_laserFront;

    yarp::dev::PolyDriver * m_driver_laserBack;
    yarp::dev::IRangefinder2D* m_dev_laserBack;
    
    int m_samples;
    double m_resolution;
    bool m_inited;
    bool m_onSimulator; //if true the device looks for front and back laser devices in gazebo, else creates them

    
    yarp::os::Mutex       m_mutex; //MI SERVE??????

    LaserCfg_t m_lFrontCfg;
    LaserCfg_t m_lBackCfg;

};

#endif //GAZEBOYARP_CONTROLBOARDDRIVER_HH
