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

#include "gesticulateThread.h"

YARP_LOG_COMPONENT(GESTICULATE_THREAD, "r1_obr.gesticulate")

GesticulateThread::GesticulateThread(ResourceFinder &_rf, double _period) :
    PeriodicThread(_period) {}


bool GesticulateThread::threadInit()
{
    m_rpc_left.open("/gesticulate/larm:rpc");
    m_rpc_right.open("/gesticulate/rarm:rpc");
    m_audioPlayPort.open("/gesticulate/audio:i");

    return true;
}


void GesticulateThread::threadRelease()
{
    if (m_rpc_left.asPort().isOpen())
        m_rpc_left.close();
    
    if (m_rpc_right.asPort().isOpen())
        m_rpc_right.close();
    
    if (!m_audioPlayPort.isClosed())
        m_audioPlayPort.close();

}


void GesticulateThread::run()
{    
    bool audio_is_playing{false};
    Bottle* player_status = m_audioPlayPort.read(false);
    if (player_status)
    {
        audio_is_playing = (unsigned int)player_status->get(1).asInt64() > 0;
    }
    
    if(audio_is_playing)
    {
        Bottle motion1{"ctpq time 3.0 off 0 pos (29.44 11.60 -10.11 73.74 62.99 0 -0.02 0.23)"};
        Bottle motion2{"ctpq time 3.0 off 0 pos (29.36 9.05 8.53 71.02 61.04 0 -0.06 -0.29)"};
        
        if(rand() % 2) m_rpc_right.write(motion1);
        if(rand() % 2) m_rpc_left.write(motion2);
        Time::delay(4.0);

        audio_is_playing = (unsigned int)player_status->get(1).asInt64() > 0;
        if(audio_is_playing)
        {
            Bottle motion3{"ctpq time 3.0 off 0 pos (29.44 11.60 25.40 70.14 63.08 0 -0.11 0.07)"};
            Bottle motion4{"ctpq time 3.0 off 0 pos (29.36 9.05 -10.02 50.27 32.8217 0 -0.02 -0.36)"};
            if(rand() % 2) m_rpc_right.write(motion3);
            if(rand() % 2) m_rpc_left.write(motion4);
            Time::delay(3.0);
        }

        Bottle armHome{"ctpq time 3.0 off 0 pos (-9.0 15.0 -10.0 50.5 0.0 0.0 -0.0 -0.0)"};
        m_rpc_right.write(armHome);
        m_rpc_left.write(armHome);
        Time::delay(3.0);

    }
    
}
