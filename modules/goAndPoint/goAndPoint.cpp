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


#include "goAndPoint.h"

YARP_LOG_COMPONENT(GO_AND_POINT, "cer.goAndPoint")

GoAndPoint::GoAndPoint() :
    m_period(1.0)
{  
}

bool GoAndPoint::configure(yarp::os::ResourceFinder &rf) 
{   

    if(rf.check("period")){m_period = rf.find("period").asFloat32();}

    double threadPeriod = 0.02;
    if(rf.check("thread_period")){threadPeriod = rf.find("thread_period").asFloat32();}
  

    std::string portName = "/goAndPoint/inputQuestion:i";
    if(rf.check("input_question_port")){portName = rf.find("input_question_port").asString();}
    bool ret = m_inputPort.open(portName);
    if (!ret)
    {
        yCError(GO_AND_POINT, "Unable to open input port");
        return false;
    }
   
    // --------- Thread initialization --------- //
    m_innerThread = new GoAndPointThread(threadPeriod,rf);
    bool threadOk = m_innerThread->start();
    if (!threadOk){
        return false;
    }
    
    m_inputPort.useCallback(*m_innerThread);

    return true;
}

bool GoAndPoint::close()
{
    m_innerThread->stop();
    delete m_innerThread;
    m_innerThread =NULL;
    m_inputPort.close();

    return true;
}

double GoAndPoint::getPeriod()
{
    return m_period;
}

bool GoAndPoint::updateModule()
{
    if (isStopping())
    {
        if (m_innerThread) m_innerThread->stop();
        return false;
    }

    return true;
}
