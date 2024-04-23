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

#include "gesticulate.h"

YARP_LOG_COMPONENT(GESTICULATE, "r1_obr.gesticulate")


bool Gesticulate::configure(ResourceFinder &rf)
{
    if(rf.check("period")) {m_period = rf.find("period").asFloat32();}

    m_thread = new GesticulateThread(rf);
    m_thread->start();

    return true;

}


bool Gesticulate::close()
{

    return true;
}


bool Gesticulate::updateModule()
{    
   
    return true;
}


double Gesticulate::getPeriod()
{
    return m_period;
}
