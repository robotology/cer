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
#include <yarp/os/Port.h>
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

    enum status_type
    {
        status_idle = 0,
        status_running = 1,
        status_complete = 2
    };


public:
    yarp::os::Network                yarp;
    QGraphicsScene*                  scene;
    QGraphicsTextItem*               text;
    QPen*                            outlinePen;
    QBrush*                          brush;
    renderer_enum_type               text_renderer_type;
    double                           scroll_speed;
    status_type                      status;
    std::string                      qtfontname;
    int                              qtfontsize;

    MainModule();
    ~MainModule();
    bool updateModule();
    double getPeriod();
    int offset;
    bool configure(yarp::os::ResourceFinder &rf);
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

private:

    QPixmap                                  img_numbers;

    std::string                              current_string;
    yarp::os::BufferedPort<yarp::os::Bottle> inputPort;
    yarp::os::Port rpcPort;
    yarp::os::Port cmdPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outputPort;
};

#endif
