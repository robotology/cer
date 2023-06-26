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



#define _USE_MATH_DEFINES

#include "goAndPointThread.h"
#include <chrono>
#include <cmath>
#include <list>

YARP_LOG_COMPONENT(GO_AND_POINT_THREAD, "cer.goAndPoint.GoAndPointThread")
bool print{true};


GoAndPointThread::GoAndPointThread(double _period, yarp::os::ResourceFinder &rf):
    PeriodicThread(_period),
    TypedReaderCallback(),
    m_rf(rf)
{
    m_targetOutPortName = "/goAndPoint/target:o";

}

bool GoAndPointThread::threadInit()
{
#ifdef GOANDPOINT_DEBUG
    yCDebug(GO_AND_POINT_THREAD, "thread initialising...\n");
#endif
    // --------- Generic config --------- //
    if (m_rf.check("target_point_port")) {m_targetOutPortName = m_rf.find("target_point_port").asString();}
  
    //open target ports
    if(!m_targetOutPort.open(m_targetOutPortName)){
        yCError(GO_AND_POINT_THREAD) << "Cannot open targetOut port with name" << m_targetOutPortName;
        return false;
    }

#ifdef GOANDPOINT_DEBUG
    yCDebug(GO_AND_POINT_THREAD, "... done!\n");
#endif

    return true;
}

void GoAndPointThread::run()
{
    

}

void GoAndPointThread::onRead(yarp::os::Bottle &b)
{

    yCInfo(GO_AND_POINT_THREAD,"Received: %s",b.toString().c_str());

    if(b.size() == 2)
    {

        
    }
    else{
        yCError(GO_AND_POINT_THREAD,"The input bottle has the wrong number of elements");
    }
    

}

void GoAndPointThread::threadRelease()
{
#ifdef GOANDPOINT_DEBUG
    yCDebug(GO_AND_POINT_THREAD, "Thread releasing...");
#endif

    if(m_poly.isValid())
        m_poly.close();
    
    if(!m_targetOutPort.isClosed()){
        m_targetOutPort.close();
    }
    
    yCInfo(GO_AND_POINT_THREAD, "Thread released");

#ifdef GOANDPOINT_DEBUG
    yCDebug(GO_AND_POINT_THREAD, "... done.");
#endif

    return;
}
