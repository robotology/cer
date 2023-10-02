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

#include "idleMotions.h"

YARP_LOG_COMPONENT(IDLE_MOTIONS, "r1_obr.IdleMotions")

IdleMotions::IdleMotions() 
{
    m_robot = "cer";
    m_period = 1.0;
    m_script_name = "/home/user1/robotology/cer/app/idleMotions/r1_idleMotions.sh";
    m_rpc_port_name = "/idleMotions/rpc"; 
    m_use_ctpservice = true;
};

bool IdleMotions::configure(ResourceFinder &rf)
{
    if(rf.check("robot")) {m_robot = rf.find("robot").asString();}
    if(rf.check("period")) {m_period = rf.find("period").asFloat32();}
    if(rf.check("rpc_port")) {m_rpc_port_name = rf.find("rpc_port").asString();}
    
    if(rf.check("use_ctpservice_or_setpos")) 
    {
        m_use_ctpservice = !(rf.find("use_ctpservice_or_setpos").asString() != "ctpservice");
    }

    if(m_use_ctpservice)
    {
        Searchable& ctps_config = rf.findGroup("USE_CTPSERVICES");
        if(rf.check("ctps_script_name")) {m_script_name = ctps_config.find("ctps_script_name").asString();}
    }

    //Open RPC Server Port
    if (!m_rpc_port.open(m_rpc_port_name))
    {
        yCError(IDLE_MOTIONS, "open() error could not open rpc port %s, check network", m_rpc_port_name.c_str());
        return false;
    }
    if (!attach(m_rpc_port))
    {
        yCError(IDLE_MOTIONS, "attach() error with rpc port %s", m_rpc_port_name.c_str());
        return false;
    }
    
    // Polydriver config
    Property prop;

    prop.put("device","remote_controlboard");
    prop.put("local","/idleMotions/right_arm");
    prop.put("remote","/"+m_robot+"/right_arm");
    if (!m_drivers[0].open(prop))
    {
        yCError(IDLE_MOTIONS,"Unable to connect to %s",("/"+m_robot+"/right_arm").c_str());
        close();
        return false;
    }
    prop.clear();
    prop.put("device","remote_controlboard");
    prop.put("local","/idleMotions/left_arm");
    prop.put("remote","/"+m_robot+"/left_arm");
    if (!m_drivers[1].open(prop))
    {
        yCError(IDLE_MOTIONS,"Unable to connect to %s",("/"+m_robot+"/left_arm").c_str());
        close();
        return false;
    }
    prop.clear();
    prop.put("device","remote_controlboard");
    prop.put("local","/idleMotions/head");
    prop.put("remote","/"+m_robot+"/head");
    if (!m_drivers[2].open(prop))
    {
        yCError(IDLE_MOTIONS,"Unable to connect to %s",("/"+m_robot+"/head").c_str());
        close();
        return false;
    }
    prop.clear();
    prop.put("device","remote_controlboard");
    prop.put("local","/idleMotions/torso");
    prop.put("remote","/"+m_robot+"/torso");
    if (!m_drivers[3].open(prop))
    {
        yCError(IDLE_MOTIONS,"Unable to connect to %s",("/"+m_robot+"/torso").c_str());
        close();
        return false;
    }

    m_drivers[0].view(m_iposctrl[0]);
    m_drivers[1].view(m_iposctrl[1]);
    m_drivers[2].view(m_iposctrl[2]);
    m_drivers[3].view(m_iposctrl[3]);
    if(!m_iposctrl[0] || !m_iposctrl[1] || !m_iposctrl[2] || !m_iposctrl[3] )
    {
        yCError(IDLE_MOTIONS,"Error opening iPositionControl interfaces. Devices not available");
        return false;
    }

    m_drivers[0].view(m_ictrlmode[0]);
    m_drivers[1].view(m_ictrlmode[1]);
    m_drivers[2].view(m_ictrlmode[2]);
    m_drivers[3].view(m_ictrlmode[3]);
    if(!m_ictrlmode[0] || !m_ictrlmode[1] || !m_ictrlmode[2] || !m_ictrlmode[3])
    {
        yCError(IDLE_MOTIONS,"Error opening iControlMode interfaces. Devices not available");
        return false;
    }

    if (m_use_ctpservice)
    {
        //Motions names definition
        if(!rf.check("MOTIONS"))
        {
            yCError(IDLE_MOTIONS,"MOTIONS section missing in ini file. Please list motions name in ini file.");
            return false;
        }
        else
        {
            Searchable& config = rf.findGroup("MOTIONS");
            int idx {0};
            while (true) 
            {
                if (!config.check(to_string(idx)))
                    break;

                string motion_name = config.find(to_string(idx)).asString();           

                m_motions.insert({idx, motion_name });
                idx++;
            }
            if (idx==0)
            {
                yCError(IDLE_MOTIONS,"No motion name listed in MOTIONS section. Please list motions names in ini file.");
                return false;
            }
            else
                m_number_of_possible_motions = idx;
        }
    }

    return true;
}


bool IdleMotions::updateModule()
{
    return true;
}


double IdleMotions::getPeriod()
{
    return m_period;
}


void IdleMotions::setCtrlMode(const int part, int ctrlMode)
{
    int NUMBER_OF_JOINTS;
    m_iposctrl[part]->getAxes(&NUMBER_OF_JOINTS);
    for (int i_joint=0; i_joint < NUMBER_OF_JOINTS; i_joint++){ m_ictrlmode[part]->setControlMode(i_joint, ctrlMode); } 
}


bool IdleMotions::respond(const Bottle &cmd, Bottle &reply)
{    
    reply.clear();
    
    for (int i = 0 ; i<=3 ; i++)
    {
        setCtrlMode(i, VOCAB_CM_POSITION);
    }
    
    if(m_use_ctpservice)
    {
        int motion_number = rand() % m_number_of_possible_motions;
        string motion_name = m_motions.at(motion_number);
        string command = m_script_name + " " + motion_name + " &";
        yCInfo(IDLE_MOTIONS, "Executing: %s", command.c_str() );

        int result = system(command.c_str());

        if(result != 0)
        {
            reply.addVocab32(Vocab32::encode("nack"));
            return false;
        }
    }
    else
    {
        string motion_name = move();
        yCInfo(IDLE_MOTIONS, "Motion: %s", motion_name.c_str() );
    }

    
    // for (int i = 0 ; i<=3 ; i++)
    // {
    //     setCtrlMode(i, VOCAB_CM_POSITION_DIRECT);
    // }

    if (reply.size()==0)
        reply.addVocab32(Vocab32::encode("ack")); 

    return true;
}


bool IdleMotions::close()
{
    if(m_drivers[0].isValid())
    {
        m_drivers[0].close();
    }

    if(m_drivers[1].isValid())
    {
        m_drivers[1].close();
    }

    if(m_drivers[2].isValid())
    {
        m_drivers[2].close();
    }

    if(m_drivers[3].isValid())
    {
        m_drivers[3].close();
    }

    if (m_rpc_port.asPort().isOpen())
        m_rpc_port.close();

    return true;
}

