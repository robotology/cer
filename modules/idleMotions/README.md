# disappointmentPose

## General description
Module that sets the robot R1 in a predefined pose.

This module has been designed to perform the final motion of the the robot at the end of a failed search.

## Usage:
When the module is launched, an input port is opened (default name: `/disappointmentPose/input:i`).
When this port receives any input, the robot assumes a predefined pose of the arms and of the head.

The pose is defined in a .ini file (default file name `disappointmentPose.ini`), where you can find different config groups some named `NOT ACTIVE`, and at least one named `ACTIVE`.
In these groups you can find the values for each dof of the arms and the head, related to a certain pose.
You can define how many poses you desire, but just the first `ACTIVE` group will be used for the motion of the robot.


