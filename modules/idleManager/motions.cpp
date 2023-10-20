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

YARP_LOG_COMPONENT(IDLE_MOTIONS_MOTIONS, "r1_obr.idleManager.idleMotions.motions")

string IdleMotions::move(int motion_number) 
{
    if (motion_number == -1)
        motion_number = rand() % 6;
    else if (motion_number > 5)
    {
        yCError(IDLE_MOTIONS_MOTIONS) << "Motion number out of bounds";
        return "OutOfBounds";
    }
    
    string mot;

    switch (motion_number) {
        case 0: stretch_right_arm();
                mot = "stretch_right_arm";
                break;
        case 1: stretch_left_arm();
                mot = "stretch_left_arm";
                break;
        case 2: stretch_shoulders();
                mot = "stretch_shoulders";
                break;
        case 3: stretch_back();
                mot = "stretch_back";
                break;
        case 4: look_gripper();
                mot = "look_gripper";
                break;
        case 5: look_watch();
                mot = "look_watch";
                break;
        }
    
    return mot;
}


void IdleMotions::arms_home() 
{
    double home[8] = {-9.0, 15.0, -10.0, 50.5, 0.0, 0.0, 0.0, 0.0};
    m_iposctrl[0]->positionMove(home); //right arm
    m_iposctrl[1]->positionMove(home); //left arm
}

void IdleMotions::head_home() 
{
    double home[2] = {0.0, 0.0};
    m_iposctrl[2]->positionMove(home); //head
}

void IdleMotions::torso_home() 
{
    double home[4] = {0.012, 0.0, 0.0, 0.0};
    m_iposctrl[3]->positionMove(home); //torso
}

void IdleMotions::go_home() 
{
    torso_home();
    head_home();
    arms_home();
}


void IdleMotions::stretch_right_arm()
{
    double left_arm[8] = {-15.0, 15.2, -10.2, 61.2, 0.0, 0.0, 0.0, 0.0}; m_iposctrl[1]->positionMove(left_arm); //left arm
    double right_arm[8] = {13.8, 26.4, -10.2, 13.5, 50.0, 0.0, 0.0, 0.0}; m_iposctrl[0]->positionMove(right_arm); //right arm
    double head[2] = {4.2, -15.3};  m_iposctrl[2]->positionMove(head); //head
    Time::delay(3.5);
    go_home();
};

void IdleMotions::stretch_left_arm()
{
    double left_arm[8] = {13.8, 26.4, -10.2, 13.5, 50.0, 0.0, 0.0, 0.0}; m_iposctrl[1]->positionMove(left_arm); //left arm
    double right_arm[8] = {-15.0, 15.2, -10.2, 61.2, 0.0, 0.0, 0.0, 0.0}; m_iposctrl[0]->positionMove(right_arm); //right arm
    double head[2] = {4.2, 15.3};  m_iposctrl[2]->positionMove(head); //head
    Time::delay(3.5);
    go_home();
};

void IdleMotions::stretch_shoulders()
{
    double torso[4] = {0.03, 0.0, 0.0, 0.0}; m_iposctrl[3]->positionMove(torso); //torso
    double right_arm[8] = {-9.2, 4.8, 40.8, 16.2, 0.0, 0.0, 0.0, 0.0}; m_iposctrl[0]->positionMove(right_arm); //right arm
    double left_arm[8] = {-9.2, 4.8, 40.8, 16.2, 0.0, 0.0, 0.0, 0.0}; m_iposctrl[1]->positionMove(left_arm); //left arm
    double head[2] = {12.6, 0.0};  m_iposctrl[2]->positionMove(head); //head
    Time::delay(3.0);
    double torso1[4] = {0.012, 0.0, 0.0, 0.0}; m_iposctrl[3]->positionMove(torso1); //torso
    go_home();
};

void IdleMotions::stretch_back()
{
    double head[2] = {9.0, -16.0};  m_iposctrl[2]->positionMove(head); //head
    double right_arm[8] = {-20.0, 35.2, -10.2, 93.6, 0.0, 0.0, 0.0, 0.0}; m_iposctrl[0]->positionMove(right_arm); //right arm
    double left_arm[8] = {21.8, 12.0, -17.0, 56.7, 0.0, 0.0, 0.0, 0.0}; m_iposctrl[1]->positionMove(left_arm); //left arm
    Time::delay(0.5);
    double torso[4] = {0.03, 0.0, 0.0, -20.0}; m_iposctrl[3]->positionMove(torso); //torso
    Time::delay(2.0);
    double head1[2] = {9.0, 16.0};  m_iposctrl[2]->positionMove(head1); //head
    double left_arm1[8] = {-20.0, 35.2, -10.2, 93.6, 0.0, 0.0, 0.0, 0.0}; m_iposctrl[1]->positionMove(left_arm1); //left arm
    double right_arm1[8] = {21.8, 12.0, -17.0, 56.7, 0.0, 0.0, 0.0, 0.0}; m_iposctrl[0]->positionMove(right_arm1); //right arm
    Time::delay(0.5);
    double torso1[4] = {0.03, 0.0, 0.0, 20.0}; m_iposctrl[3]->positionMove(torso1); //torso
    Time::delay(3.5);
    double torso2[4] = {0.012, 0.0, 0.0, 0.0}; m_iposctrl[3]->positionMove(torso2); //torso
    go_home();
};

void IdleMotions::look_gripper()
{
    double right_arm[8] = {41.7, 14.9, -40.6, 74.6, 78.8, 0.0, 6.1, 10.0}; m_iposctrl[0]->positionMove(right_arm); //right arm
    double head[2] = {20.0, 5.0};  m_iposctrl[2]->positionMove(head); //head
    Time::delay(3.0);
    double right_arm1[8] = {41.7, 14.9, -40.6, 74.6, -33.8, 0.0, 6.1, 10.0}; m_iposctrl[0]->positionMove(right_arm1); //right arm
    Time::delay(2.0);
    double right_arm2[8] = {41.7, 14.9, -40.6, 74.6, 78.8, 0.0, 6.1, 10.0}; m_iposctrl[0]->positionMove(right_arm2); //right arm
    Time::delay(3.0);
    go_home();
};

void IdleMotions::look_watch()
{
    double left_arm[8] = {41.7, 14.9, -40.6, 74.6, -40.0, 0.0, 0.0, 0.0}; m_iposctrl[1]->positionMove(left_arm); //left arm
    double head[2] = {20.0, 5.0};  m_iposctrl[2]->positionMove(head); //head
    Time::delay(2.0);
    double left_arm1[8] = {41.7, 14.9, -40.6, 74.6, -54.5, 0.0, 0.0, 0.0}; m_iposctrl[1]->positionMove(left_arm1); //left arm
    Time::delay(3.0);
    double left_arm2[8] = {57.5, 30.0, -40.8, 85.0, -73.6, 0.0, 0.0, 0.0}; m_iposctrl[1]->positionMove(left_arm2); //left arm
    double head1[2] = {15.6, 5.6};  m_iposctrl[2]->positionMove(head1); //head
    Time::delay(2.5);
    go_home();
};