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

#ifndef IDLE_MANAGER_H
#define IDLE_MANAGER_H

#include <yarp/os/all.h>
#include <r1OrchestratorRPC.h>
#include "idleMotions.h"


using namespace yarp::os;
using namespace std;

class IdleManager : public RFModule
{
private:
    double              m_period; 
    double              m_last_idle_time = 0;

    IdleMotions*        m_motions;
    r1OrchestratorRPC   m_r1OrchestratorRPC;

    //Ports
    RpcServer           m_rpc_port;
    string              m_rpc_port_name;
    Port                m_r1Orchestrator_client_port;
    string              m_r1Orchestrator_client_port_name;
    string              m_r1Orchestrator_server_port_name;
    
public:
    //Constructor/Distructor
    IdleManager();
    ~IdleManager() = default;

    //Internal methods
    virtual bool configure(ResourceFinder &rf);
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

    bool respond(const Bottle &request, Bottle &reply);

};

#endif