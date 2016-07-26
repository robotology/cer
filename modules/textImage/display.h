/*
* Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
* Author: Marco Randazzo
* email:   marco.randazzo@iit.it
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#ifndef DISPLAY_H
#define DISPLAY_H

#include <yarp/os/Os.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IBattery.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Image.h>
#include <QPixmap>
#include <QGraphicsScene>

class MainModule :public yarp::os::RFModule
{
    enum renderer_enum_type
    {
        none = 0,
        bitmap =1,
        qtfont =2
    };

public:
    yarp::os::Network                yarp;
    QGraphicsScene*                  scene;
    QGraphicsTextItem*               text;
    QPen*                            outlinePen;
    QBrush*                          brush;
    renderer_enum_type               text_renderer_type;

    MainModule();
    ~MainModule();
    bool updateModule();
    double getPeriod();
    int offset;
    bool configure(yarp::os::ResourceFinder &rf);

private:

    QPixmap                          img_numbers;

    std::string                                 current_string;
    yarp::os::BufferedPort<yarp::os::Bottle>  inputPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    outputPort;
};

#endif
