/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Alessandro Scalzo
 * email:  alessandro.scalzo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <RobotController.h>

// pseudoinverse as Lagrange multipliers problem
// minimize 1/2*qd.t()*W*qd under the constraint v = J*qd
// G(qd,L) = 1/2*qd.t()*W*qd+L.t()*(v-J*qd)
// dG/dqd = qd.t()*W-L.t()*J = 0  ==> qd.t()*W = L.t()*J  ==>  W*qd = J.t()*L  ==>  qd = W.inv()*J.t()*L
// v = J*qd = J*W.inv()*J.t()*L  ==>  L = (J*W.inv()*J.t()).inv()*v
// qd = W.inv()*J.t()*L = W.inv()*J.t()*(J*W.inv()*J.t()).inv()*v

//Vec3 Werr=(mHand[L]->Toj.Rj().Ex()%Uxstar)+(mHand[L]->Toj.Rj().Ey()%Uystar)+(mHand[L]->Toj.Rj().Ez()%Uzstar);



