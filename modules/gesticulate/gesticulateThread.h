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

#ifndef GESTICULATE_THREAD_H
#define GESTICULATE_THREAD_H

#include <yarp/os/all.h>

using namespace yarp::os;
using namespace std;

class GesticulateThread : public PeriodicThread
{
private:

    //Ports
    BufferedPort<Bottle> m_audioPlayPort;
    RpcClient m_rpc_left;
    RpcClient m_rpc_right;
    
public:

    //Constructor/Distructor
    GesticulateThread(ResourceFinder &_rf, double _period = 0.1);
    ~GesticulateThread() = default;

    //Internal methods
    bool threadInit() override;
    void threadRelease() override;
    void run() override;

};

#endif