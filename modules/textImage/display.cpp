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
    if (text)
    {
        delete text;
        text = 0;
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
    int rect_w=32;

    Bottle* b = inputPort.read(false);
    if (b != NULL)
    {
        current_string = b->get(0).asString();
        offset = 81;
    }

    scene->clear();
    scene->setSceneRect(0, 0, 80, 32);

    {
        if (offset > -int(strlen(current_string.c_str()) * rect_w))
        {
            offset--;
        }

        scene->addRect(QRect(0, 0, 80, 32), *outlinePen, *brush);

        int len = strlen(current_string.c_str());
        for (int i = 0; i < len; i++)
        {
            //if (current_string[i] >= '0' && current_string[i] <= '9')
         /*   if ((current_string[i] = ':' &&
                (current_string[i+1] = '-' &&
                (current_string[i+2] = ')')*/
            {
                QRect rect((current_string[i]) * rect_w, 0, rect_w, 32);
                //QRect rect((current_string[i] - '0') * rect_w, 0, rect_w, 32);
                QPixmap  qpm = img_numbers.copy(rect);
                if (text_renderer_type == bitmap)
                {
                    QGraphicsPixmapItem *p1 = scene->addPixmap(qpm);
                    p1->setScale(1);
                    p1->setFlag(QGraphicsItem::ItemIsMovable, true);
                    p1->setPos(i * (rect_w - 16) + offset, 0);
                }
            }
        }
    }
    if (text_renderer_type == qtfont)
    {
        text = new QGraphicsTextItem;
        text->setDefaultTextColor(Qt::white);
        text->setPos(offset, 0);
        text->setFont(QFont("Newyork", 12));
        text->setPlainText(current_string.c_str());
        scene->addItem(text);
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

    if (text_renderer_type == qtfont)
    {
        delete text;
    }
    return true;
}

bool MainModule::configure(yarp::os::ResourceFinder &rf)
{
    bool ret_load = true;
    std::string fontname = "default.bmp";

    if (rf.check("font"))
    {
        fontname = rf.find("font").asString();
    }

    if (rf.check("renderer"))
    {
        string tt = rf.find("renderer").asString();
        if (tt == "bitmap_renderer")
        {
            yInfo("Using bitmap renderer");
            text_renderer_type = bitmap;
        }
        else if (tt == "qtfont_renderer")
        {
            yInfo("Using qtfont renderer");
            text_renderer_type = qtfont;
        }
        else
        {
            text_renderer_type = none;
            yError() << "Invalid renderer";
            return false;
        }
    }

    ret_load &= img_numbers.load(QString(":/images/") + QString(fontname.c_str()));
    if (ret_load == false)
    {
        yError("Failed loading graphics");
        return false;
    }

    img_numbers.setMask(img_numbers.createMaskFromColor(QColor(255, 0, 255)));

    inputPort.open("/textimage/txt:i");
    outputPort.open("/textimage/img:o");
    bool bc= yarp::os::Network::connect("/textimage/img:o", "/robot/faceDisplay/image:i");
    if (bc == false)
    {
        yError() << "Unable to connect to the robot";
    }

    return ret_load;
}

MainModule::MainModule()
{
    offset = 0;
    scene = new QGraphicsScene;
    outlinePen = new QPen (Qt::black);
    brush = new QBrush (Qt::black);
    text_renderer_type = qtfont;
}
