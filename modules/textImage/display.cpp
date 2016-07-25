/*
* Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
* Author: Marco Randazzo
* email:   marco.randazzo@iit.it
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#ifdef MSVC
    #define _USE_MATH_DEFINES
#endif
#include <cmath>

#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include "display.h"
#include <QGraphicsPixmapItem>
#include <QBitmap>
#include <QGraphicsView>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

MainModule::~MainModule()
{
    if (scene)
    {
        delete scene;
        scene = 0;
    }
    if (outlinePen)
    {
        delete outlinePen;
        outlinePen = 0;
    }
    if (brush)
    {
        delete brush;
        brush = 0;
    }
    outputPort.interrupt();
    outputPort.close();
}

double MainModule::getPeriod()
{
    return 0.050;
}

bool MainModule::updateModule()
{
    Bottle* b = inputPort.read(false);
    if (b != NULL)
    {
        current_string = b->get(0).asString();
        offset = 81;
    }

    scene->clear();
    scene->setSceneRect(0, 0, 80, 32);
    //the voltage
    {
        if (offset > -int(strlen(current_string.c_str()) * 29))
        {
            offset--;
        }

        int len = strlen(current_string.c_str());
        for (int i = 0; i < len; i++)
        {
            if (current_string[i] >= '0' && current_string[i] <= '9')
            {
                QRect rect((current_string[i] - '0') * 29, 0, 29, 52);
                QPixmap  qpm = img_numbers.copy(rect);
                scene->addRect(QRect(0, 0, 80, 32), *outlinePen, *brush);
                QGraphicsPixmapItem *p1 = scene->addPixmap(qpm);
                p1->setScale(1);
                p1->setFlag(QGraphicsItem::ItemIsMovable, true);
                p1->setPos(i * 29+offset, 0);
            }
        }
    }
    QImage image(scene->sceneRect().size().toSize(), QImage::Format_RGB32);
    QPainter painter(&image);
    scene->render(&painter);

    yarp::sig::ImageOf<yarp::sig::PixelRgb> &i = outputPort.prepare();
    int width = 80;
    int height = 32;
    i.resize(width, height);
    for (int x = 0; x < width; x++)
        for (int y = 0; y < height; y++)
        {         
            i.pixel(x, y).r = qRed(image.pixel(x, y));
            i.pixel(x, y).g = qGreen(image.pixel(x, y));
            i.pixel(x, y).b = qBlue(image.pixel(x, y));
        }
    outputPort.write();

    return true;
}

MainModule::MainModule()
{
    offset = 0;
    bool ret_load = true;
    ret_load &= img_numbers.load(":/images/numbers.bmp");
    if (ret_load == false)
    {
        yError("Failed loading graphics");
    }

    img_numbers.setMask(img_numbers.createMaskFromColor(QColor(255, 0, 255)));

    scene = new QGraphicsScene;
    outlinePen = new QPen (Qt::black);
    brush = new QBrush (Qt::black);

    inputPort.open("/textimage/txt:i");
    outputPort.open("/textimage/img:o");
    yarp::os::Network::connect("/textimage/img:o", "/robot/faceDisplay/image:i");
}
