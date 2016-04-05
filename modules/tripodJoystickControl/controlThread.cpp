/*
* Copyright (C)2015  iCub Facility - Istituto Italiano di Tecnologia
* Author: Marco Randazzo
* email:  marco.randazzo@iit.it
* website: www.robotcub.org
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#include "controlThread.h"
#define MIN_ELONG 0.0
#define MAX_ELONG 0.2
#define MAX_ALPHA 25.0

void ControlThread::run()
{
    double pidout_linear_speed  = 0;
    double pidout_angular_speed = 0;
    double pidout_direction     = 0;

    Bottle *b = this->port_joystick_control.read(false);
    if (b)
    {
        double val0 = b->get(1).asDouble();
        double val1 = b->get(2).asDouble();
        double val2 = b->get(3).asDouble();
        if (val1 > 100) val1 = 100;
        if (val1 < -100) val1 = -100;
        if (val2 > 100) val2 = 100;
        if (val2 < -100) val2 = -100;
        pitch = val1/100.0*25.0 * sin(val0 / 180.0 * 3.14);
        roll  = val1/100.0*25.0 * cos(val0 / 180.0 * 3.14);
        elong = elong + val2 / 200000.0;
        if (elong > MAX_ELONG) elong = MAX_ELONG;
        if (elong < MIN_ELONG) elong = MIN_ELONG;
    }

    char buff [255];
    sprintf(buff, "elong: %+3.3f  pitch:%+3.3f  roll:%+3.3f ", elong, pitch, roll);
    yDebug() << buff;
    
    double enc_elong = 0;
    double enc_roll = 0;
    double enc_pitch = 0;

    iEnc->getEncoder(0, &enc_elong);
    iEnc->getEncoder(0, &enc_roll);
    iEnc->getEncoder(0, &enc_pitch);

    //-----------

    if (elong < MIN_ELONG)
    {
        elong = MIN_ELONG;
        yDebug() << "Out of limits elong" << elong;
    }
    if (elong > MAX_ELONG)
    {
        elong = MAX_ELONG;
        yDebug() << "Out of limits elong" << elong;
    }
    if (motors_enabled) iDir->setPosition(0, elong);
    
    //-----------

    if (pitch < -MAX_ALPHA)
    {
        pitch = -MAX_ALPHA;
        yDebug() << "Out of limits elong" << pitch;
    }
    if (pitch > MAX_ALPHA)
    {
        pitch = MAX_ALPHA;
        yDebug() << "Out of limits elong" << pitch;
    }
    if (motors_enabled) iDir->setPosition(1, pitch);

    //-----------

    if (roll < -MAX_ALPHA)
    {
        roll = -MAX_ALPHA;
        yDebug() << "Out of limits elong" << roll;
    }
    if (roll > MAX_ALPHA)
    {
        roll = MAX_ALPHA;
        yDebug() << "Out of limits elong" << roll;
    }
    if (motors_enabled) iDir->setPosition(2, roll);

}

void ControlThread::printStats()
{
}

