/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef IMAGE_PATTERN_RECOGNITION_H
#define IMAGE_PATTERN_RECOGNITION_H

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>

#include <math.h>
#include <opencv2/opencv.hpp>

//Defaults RGBD sensor
#define RGBDClient            "RGBDSensorClient"
#define RGBDLocalImagePort    "/imagePatternRecognition/clientRgbPort:i"
#define RGBDLocalDepthPort    "/imagePatternRecognition/clientDepthPort:i"
#define RGBDLocalRpcPort      "/imagePatternRecognition/clientRpcPort"
#define RGBDRemoteImagePort   "/SIM_CER_ROBOT/depthCamera/rgbImage:o"
#define RGBDRemoteDepthPort   "/SIM_CER_ROBOT/depthCamera/depthImage:o"
#define RGBDRemoteRpcPort     "/SIM_CER_ROBOT/depthCamera/rpc:i"
#define RGBDImageCarrier      "mjpeg"
#define RGBDDepthCarrier      "fast_tcp"

class ImagePatternRecognition : public yarp::os::RFModule, public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
    enum
    {
        CV_TM_SQDIFF        =0,
        CV_TM_SQDIFF_NORMED =1,
        CV_TM_CCORR         =2,
        CV_TM_CCORR_NORMED  =3,
        CV_TM_CCOEFF        =4,
        CV_TM_CCOEFF_NORMED =5
    };

    protected:
        yarp::dev::PolyDriver            m_rgbdPoly;
        yarp::dev::IRGBDSensor*          m_iRgbd{nullptr};
        yarp::sig::FlexImage             m_rgbImage;

        //Ports
        yarp::os::BufferedPort<yarp::os::Bottle> m_inputPatternPort;  // input: string with the path to the pattern image
        yarp::os::BufferedPort<yarp::os::Bottle> m_OutPort;           // output: coordinates in the rgb camera frame of the point where the pattern matches the base image
        double    m_period;
        int       m_matchingMethod;

    public:
        ImagePatternRecognition();
        virtual bool configure(yarp::os::ResourceFinder &rf);
        virtual bool close();
        virtual double getPeriod();
        virtual bool updateModule();

        // inherited from TypedReaderCallback
        using TypedReaderCallback<yarp::os::Bottle>::onRead;
        void onRead(yarp::os::Bottle& btl) override;

    private:
        cv::Point MatchImage( cv::Mat img, cv::Mat templ, cv::Mat result, int match_method );
};

#endif
