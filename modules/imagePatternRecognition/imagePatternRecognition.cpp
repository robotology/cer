#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

#include "imagePatternRecognition.h"

YARP_LOG_COMPONENT(IMAGE_PATTERN_RECOGNITION, "cer.imagePatternRecognition")

ImagePatternRecognition::ImagePatternRecognition() :
    m_period(1.0)
{
}

bool ImagePatternRecognition::configure(yarp::os::ResourceFinder &rf)
{
    if(rf.check("period")){m_period = rf.find("period").asFloat32();}

    // --------- RGBDSensor config --------- //
    yarp::os::Property rgbdProp;
    // Prepare default prop object
    rgbdProp.put("device", RGBDClient);
    rgbdProp.put("localImagePort", RGBDLocalImagePort);
    rgbdProp.put("localDepthPort", RGBDLocalDepthPort);
    rgbdProp.put("localRpcPort", RGBDLocalRpcPort);
    rgbdProp.put("remoteImagePort", RGBDRemoteImagePort);
    rgbdProp.put("remoteDepthPort", RGBDRemoteDepthPort);
    rgbdProp.put("remoteRpcPort", RGBDRemoteRpcPort);
    rgbdProp.put("ImageCarrier", RGBDImageCarrier);
    rgbdProp.put("DepthCarrier", RGBDDepthCarrier);
    bool okRgbdRf = rf.check("RGBD_SENSOR_CLIENT");
    if(!okRgbdRf)
    {
        yCWarning(IMAGE_PATTERN_RECOGNITION,"RGBD_SENSOR_CLIENT section missing in ini file. Using default values");
    }
    else
    {
        yarp::os::Searchable& rgbd_config = rf.findGroup("RGBD_SENSOR_CLIENT");
        if(rgbd_config.check("device")) {rgbdProp.put("device", rgbd_config.find("device").asString());}
        if(rgbd_config.check("localImagePort")) {rgbdProp.put("localImagePort", rgbd_config.find("localImagePort").asString());}
        if(rgbd_config.check("localDepthPort")) {rgbdProp.put("localDepthPort", rgbd_config.find("localDepthPort").asString());}
        if(rgbd_config.check("localRpcPort")) {rgbdProp.put("localRpcPort", rgbd_config.find("localRpcPort").asString());}
        if(rgbd_config.check("remoteImagePort")) {rgbdProp.put("remoteImagePort", rgbd_config.find("remoteImagePort").asString());}
        if(rgbd_config.check("remoteDepthPort")) {rgbdProp.put("remoteDepthPort", rgbd_config.find("remoteDepthPort").asString());}
        if(rgbd_config.check("remoteRpcPort")) {rgbdProp.put("remoteRpcPort", rgbd_config.find("remoteRpcPort").asString());}
        if(rgbd_config.check("ImageCarrier")) {rgbdProp.put("ImageCarrier", rgbd_config.find("ImageCarrier").asString());}
        if(rgbd_config.check("DepthCarrier")) {rgbdProp.put("DepthCarrier", rgbd_config.find("DepthCarrier").asString());}
    }

    m_rgbdPoly.open(rgbdProp);
    if(!m_rgbdPoly.isValid())
    {
        yCError(IMAGE_PATTERN_RECOGNITION,"Error opening PolyDriver check parameters");
        return false;
    }
    m_rgbdPoly.view(m_iRgbd);
    if(!m_iRgbd)
    {
        yCError(IMAGE_PATTERN_RECOGNITION,"Error opening iRGBD interface. Device not available");
        return false;
    }

    // open Ports
    std::string patternPortName = "/imagePatternRecognition/inputPatternImage:i";
    if(rf.check("pattern_matching_in_port")){patternPortName = rf.find("pattern_matching_in_port").asString();}
    m_inputPatternPort.useCallback(*this);
    if (!m_inputPatternPort.open(patternPortName))
    {
        yCError(IMAGE_PATTERN_RECOGNITION) << "Cannot open port" << patternPortName;
        return false;
    }
    else {
        yCDebug(IMAGE_PATTERN_RECOGNITION) << "opened port" << patternPortName;
    }
    

    std::string OutPortName = "/imagePatternRecognition/outputMatch:o";
    if (rf.check("pattern_matching_out_port")) {OutPortName = rf.find("pattern_matching_out_port").asString();}
    if(!m_OutPort.open(OutPortName))
    {
        yCError(IMAGE_PATTERN_RECOGNITION) << "Cannot open port" << OutPortName;
        return false;
    }
    else {
        yCDebug(IMAGE_PATTERN_RECOGNITION) << "opened port" << OutPortName;
    }

    return true;
}


double ImagePatternRecognition::getPeriod()
{
    return m_period;
}

bool ImagePatternRecognition::updateModule()
{
    return true;
}



bool ImagePatternRecognition::close()
{
    
    m_inputPatternPort.close();
    m_OutPort.close();

    if(m_rgbdPoly.isValid())
        m_rgbdPoly.close();

    return true;
}



void ImagePatternRecognition::onRead(yarp::os::Bottle& btl) 
{
    
    yCDebug(IMAGE_PATTERN_RECOGNITION,"Received:  %s",btl.toString().c_str());

    /// Declare Variables
    cv::Mat img; cv::Mat templ; cv::Mat result;
    std::string str = btl.get(0).asString();

    yCDebug(IMAGE_PATTERN_RECOGNITION,"Loading images...");
    /// Load image and template
    img = cv::imread( "/home/rcolombo/Where_is_Waldo.jpg", 1 );
    templ = cv::imread( "/home/rcolombo/Waldo.jpg", 1 );

    yCDebug(IMAGE_PATTERN_RECOGNITION,"Images Loaded. Starting matching algorithm...");
    cv::Point match { MatchImage( img, templ, result, 1 ) };

    yCDebug(IMAGE_PATTERN_RECOGNITION,"Match! Sending Output...");
    //output to goHome part
    yarp::os::Bottle&  toSend = m_OutPort.prepare();
    toSend.clear();
    toSend.addFloat32(match.x);
    toSend.addFloat32(match.y);

    yCDebug(IMAGE_PATTERN_RECOGNITION,"Output to send: %s",toSend.toString().c_str());
    m_OutPort.write();

}
