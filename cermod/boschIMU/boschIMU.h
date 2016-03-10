// Copyright: (C) 2016 iCub Facility
// Authors: Alberto Cardellino <alberto.cardellino@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef BOSCH_IMU_DEVICE
#define BOSCH_IMU_DEVICE

#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/SerialInterfaces.h>
#include <yarp/dev/GenericSensorInterfaces.h>


namespace cer {
    namespace dev {
        class BoschIMU;
    }
}

/**
*  @ingroup dev_impl_wrapper
*
* \section BoschIMU Description of input parameters
*
*  This device will connect to the proper analogServer and read the data broadcasted making them available to use for the user application. It also made available some function to check and control the state of the remote sensor.
*
* Parameters accepted in the config argument of the open method:
* | Parameter name | Type   | Units | Default Value | Required  | Description   | Notes |
* |:--------------:|:------:|:-----:|:-------------:|:--------: |:-------------:|:-----:|
* | comport        | string |       |               | Yes       | full name of device file  | ex '/dev/ttyUSB0' |
* | baudrate       | int    | Hz    |               | Yes       | baudrate setting of COM port | ex 115200 |
*
**/

class cer::dev::BoschIMU:   public yarp::dev::DeviceDriver,
                            public yarp::os::RateThread,
                            public yarp::dev::IGenericSensor
{
protected:
    yarp::os::Semaphore mutex;

    bool               verbose;
    short              status;
    yarp::sig::Vector  data;
    double             timeStamp;
    double             battery_charge;
    double             battery_voltage;
    double             battery_current;


    FILE                        *logFile;
    yarp::os::ResourceFinder    rf;
    yarp::dev::PolyDriver       driver;
    yarp::dev::ISerialDevice    *pSerial;
    char                        serial_buff[255];
    char                        log_buffer[255];

public:
    BoschIMU(int period = 20) : RateThread(period), mutex(1)  {}

    ~BoschIMU() { }

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    /*
     * Read a vector from the sensor.
     * @param out a vector containing the sensor's last readings.
     * @return true/false success/failure
     */
    virtual bool read(yarp::sig::Vector &out);

    /**
     * Get the number of channels of the sensor.
     * @param nc pointer to storage, return value
     * @return true/false success/failure
     */
    virtual bool getChannels(int *nc);

    /**
     * Calibrate the sensor, single channel.
     * @param ch channel number
     * @param v reset valure
     * @return true/false success/failure
     */
    virtual bool calibrate(int ch, double v);

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();
};


#endif  // BOSCH_IMU_DEVICE
