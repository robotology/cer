# idleManager

This module activates some predefined movements of the robot when it has been idle for a certain amount of time.
This amount of time is a random number between 20-30 seconds.
The motions to be executed can be:
- defined in a .sh file (e.g. r1_idleMotions.sh). This .sh file will contain a series of commands to the ctps services moving the robot parts. For this to work correctly, the name of the motions (i.e. the name of the functions in the file to be called) must be listed in the .ini file
- defined as methods of the class IdleMotions in the motions.cpp file
You can define as many motions you want. One will be picked randomly.

In order to avoid interferences with other open_pose modules, the input port to the gaze_controller is disconnected temporarily for the duration of the motion.