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

#include "idleManager.h"

YARP_LOG_COMPONENT(IDLE_MANAGER, "r1_obr.idleManager")

IdleManager::IdleManager() 
{
    m_period = 0.5;
    m_rpc_port_name = "/idleManager/rpc"; 
    m_r1Orchestrator_server_port_name = "/r1Obr-orchestrator/thrift:s";
    m_r1Orchestrator_client_port_name = "/idleManager/r1Orchestrator/thrift:c";
}

bool IdleManager::configure(ResourceFinder &rf)
{
    if(rf.check("period")) {m_period = rf.find("period").asFloat32();}
    if(rf.check("rpc_port")) {m_rpc_port_name = rf.find("rpc_port").asString();}
    if(rf.check("r1Orchestrator_client_port")) {m_r1Orchestrator_client_port_name = rf.find("r1Orchestrator_client_port").asString();}

    // ---------Open RPC Server Port --------- //
    if (!m_rpc_port.open(m_rpc_port_name))
    {
        yCError(IDLE_MANAGER, "open() error could not open rpc port %s, check network", m_rpc_port_name.c_str());
        return false;
    }
    if (!attach(m_rpc_port))
    {
        yCError(IDLE_MANAGER, "attach() error with rpc port %s", m_rpc_port_name.c_str());
        return false;
    }

    // --------- Open thrift client port to orchestrator --------- //
    if(!m_r1Orchestrator_client_port.open(m_r1Orchestrator_client_port_name))
    {
        yCError(IDLE_MANAGER, "Unable to open thrift client port to orchestrator");
        return false;
    }
    if(!m_r1OrchestratorRPC.yarp().attachAsClient(m_r1Orchestrator_client_port))
        yCWarning(IDLE_MANAGER, "Error attaching as client to Orchestrator thrift server");

    // --------- IdleMotions config --------- //
    m_motions = new IdleMotions(rf);
    if(!m_motions->start())
    {
        yCError(IDLE_MANAGER,"IdleMotions configuration failed");
        return false;
    }

    m_last_idle_time = Time::now();

    return true;

}


bool IdleManager::close()
{
    if (m_rpc_port.asPort().isOpen())
        m_rpc_port.close();
    
    if (m_r1Orchestrator_client_port.isOpen())
        m_r1Orchestrator_client_port.close(); 

    return true;
}


bool IdleManager::updateModule()
{    
    if (Network::exists(m_r1Orchestrator_server_port_name)) 
    {
        if (!Network::isConnected(m_r1Orchestrator_client_port.getName(), m_r1Orchestrator_server_port_name))
        {
            if (!Network::connect(m_r1Orchestrator_client_port.getName(), m_r1Orchestrator_server_port_name)) 
            {
                m_motions->dontMove();
                return true;
            }
        }

        if (m_r1OrchestratorRPC.status() != "idle")
        {
            m_motions->dontMove();
            m_last_idle_time = Time::now();
        }
        else if (Time::now()-m_last_idle_time >= 15) //waiting 15 seconds after the orchestrator became 'idle'
        {
            m_motions->nowYouCanMove();
        }
            
    }
    
    return true;
}


double IdleManager::getPeriod()
{
    return m_period;
}

bool IdleManager::respond(const Bottle &request, Bottle &reply)
{
    yCInfo(IDLE_MANAGER,"Received: %s",request.toString().c_str());
    
    reply.clear();
    string cmd=request.get(0).asString();
    if (cmd=="help")
    {
        reply.addVocab32("many");
        reply.addString("help : gets this list");
        reply.addString("move : executes a random motion");
        reply.addString("move <string>: executes the motion defined");
        reply.addString("dontMove : the robot cannot execute motions autonomously");
        reply.addString("okMove : the robot can execute motions autonomously");
    }
    else if (cmd=="dontMove")
    {
        m_motions->dontMove();
        m_motions->setUserStop(true);
    }
    else if (cmd=="okMove")
    {
        m_motions->nowYouCanMove();
        m_motions->setUserStop(false);
    }
    else if (cmd=="move")
    {
        string mot="rand";
        if(request.size()>1)
            mot = request.get(1).asString();
        
        if(!m_motions->doMotion(mot))
        {
            reply.addString("Error");
            return false;
        }
    }
    else
    {
        reply.addVocab32(Vocab32::encode("nack")); 
        yCWarning(IDLE_MANAGER,"Wrong RPC command. Type 'help'");
    }

    if (reply.size()==0)
        reply.addVocab32(Vocab32::encode("ack")); 

    return true;
}