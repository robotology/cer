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

#ifndef GESTICULATE_H
#define GESTICULATE_H

#include <yarp/os/all.h>
#include "gesticulateThread.h"

using namespace yarp::os;
using namespace std;

class Gesticulate : public RFModule
{
private:
    double m_period; 
    GesticulateThread* m_thread;
    
public:
    //Constructor/Distructor
    Gesticulate(){ m_period = 0.5;};
    ~Gesticulate() = default;

    //Internal methods
    virtual bool configure(ResourceFinder &rf);
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

};

#endif