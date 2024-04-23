# handPointing

**This module is currently under development/testing and should be not used in a user application.**

With this module R1 can use its arms to point towards a certain target which has been located through the camera of the robot.

## Description
This module opens a thread which receives the input coordinates of the target object to point to, and sends the proper commands to the ikin controller of either the right or the left arm (module "reaching"), so that R1 can point to the target. At the same time, a command is sent to the gaze controller (module "gaze-controller") so that R1 can move its head to look at the target.

The right arm is used to point at objects at the right side of the robot's torso, while the left arm is used for objects at the left side of the torso.

To avoid unnatural positions of the robot, when using one arm, the other arm is sent to its home position.

## Usage

### Input
An input port expects a bottle containing the pixel coordinates of the target object as seen by the robot's RGBD camera.

This coordinates can be easily retrieved as the output of a yarpview module.

The default input port name is `/handPointing/clicked_pos:i`, but it can be set through the clicked_pos_port setting of the .ini file

If the built-in RGBD camera of the robot is not used, this module still works as soon as the correct frame of the used camera is specified in the .ini file.

Input example:
`(120 59)`

An additional input port is created to perform the homing movement for the robot's arms and head.

The default input port name is `/handPointing/go_home:i`, but it can be set through the go_home_port setting of the .ini file.

The expected commands of this port are:
- `goHome`: sends the homing command to both the arms and the head of the robot
- `goHome (<partName1> <partName2> <partName3>)`: sends the homing command only to the specified parts. The allowed part names are `head`, `right-arm`, `left-arm`

### Output
When an input command is received through the `/handPointing/clicked_pos:i` port, the module computes the position of the end effector that the robot has to reach in order to point to the target coordinates.
The end effector position in the space is sent to the `/handPointing/r_target:o` or `/handPointing/l_target:o` output ports (depending on which arm is to be used), and the coordinates of the point to be looked at are sent to the `/handPointing/gazeTarget:o` port.

The above port names are the default ones, but they can be set through the corresponding settings of the .ini file.

The output of these ports is constructed so that it can be correctly read by the ikin controller of the `reaching` module, and by the gaze controller of the `gaze-controller` module.

As anticipated, if one arm is used to point to the target, the other arm is sent to its home position. To do this, while the end effector target position is sent to the above mentioned ports, another output containing the homing command is sent to the port `/handPointing/get_arm_home:o`.
This port is therefore to be connected to the `/handPointing/go_home:i` port.

Also in this case the name of the port can be set through the corresponding settings of the .ini file.

