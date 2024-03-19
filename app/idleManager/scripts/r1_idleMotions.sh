
 #!/bin/bash

#######################################################################################
# HELP
#######################################################################################
usage() {
cat << EOF

***************************************************************************************
R1 IDLE MOTIONS

This script scripts through the commands available for the navigation of R1

USAGE:
        $0 [options]

***************************************************************************************
OPTIONS:
		go_home
		arms_home
		torso_home
		head_home
        stretch_left_arm
        stretch_right_arm
        stretch_back
        stretch_shoulders
        look_gripper
        look_watch
        yawn
***************************************************************************************
EXAMPLE USAGE:

***************************************************************************************
EOF
}

#######################################################################################
# MOTIONS
#######################################################################################

arms_home() {
    echo "ctpq time 3.0 off 0 pos (-9.0 15.0 -10.0 50.5 0.0 0.0 -0.0 -0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 3.0 off 0 pos (-9.0 15.0 -10.0 50.5 0.0 0.0 -0.0 -0.0)" | yarp rpc /ctpservice/left_arm/rpc
}

torso_home() {
    echo "ctpq time 3.0 off 0 pos (0.012 0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
}


head_home() {
    echo "ctpq time 2.5 off 0 pos (0.0 0.0)" | yarp rpc /ctpservice/head/rpc
}


go_home() {
    torso_home
    head_home
    arms_home
}


stretch_left_arm() {
    echo "ctpq time 2.0 off 0 pos (13.8 26.4 -10.2 13.5 50.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 2.0 off 0 pos (-15.0 15.2 -10.2 61.2 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 2.0 off 0 pos (4.2 15.3)" | yarp rpc /ctpservice/head/rpc
    sleep 1.0
    go_home
}


stretch_right_arm() {
    echo "ctpq time 2.0 off 0 pos (13.8 26.4 -10.2 13.5 50.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 2.0 off 0 pos (-15.0 15.2 -10.2 61.2 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 2.0 off 0 pos (4.2 -15.3)" | yarp rpc /ctpservice/head/rpc
    sleep 1.0
    go_home
}


stretch_shoulders() {
    echo "ctpq time 2.0 off 0 pos (0.03 0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
    echo "ctpq time 2.0 off 0 pos (-9.2 4.8 40.8 16.2 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 2.0 off 0 pos (-9.2 4.8 40.8 16.2 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 2.5 off 0 pos (12.6 0.0)" | yarp rpc /ctpservice/head/rpc
    sleep 1.0
    echo "ctpq time 2.0 off 0 pos (0.012 0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
    go_home
}

stretch_back() {
    echo "ctpq time 3.0 off 0 pos (0.03 0.0 0.0 -20.0)" | yarp rpc /ctpservice/torso/rpc
    echo "ctpq time 3.0 off 0 pos (9.0 -16.0)" | yarp rpc /ctpservice/head/rpc
    echo "ctpq time 3.0 off 0 pos (-20.0 35.2 -10.2 93.6 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 3.0 off 0 pos (21.8 12.0 -17.0 56.7 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 3.0
    echo "ctpq time 3.0 off 0 pos (0.03 0.0 0.0 20.0)" | yarp rpc /ctpservice/torso/rpc
    echo "ctpq time 3.0 off 0 pos (9.0 16.0)" | yarp rpc /ctpservice/head/rpc
    echo "ctpq time 3.0 off 0 pos (-20.0 35.2 -10.2 93.6 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 3.0 off 0 pos (21.8 12.0 -17.0 56.7 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 3.0
    echo "ctpq time 3.0 off 0 pos (0.012 0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
    go_home
}

look_gripper() {
    echo "ctpq time 2.5 off 0 pos (41.7 14.9 -40.6 74.6 78.8 0.0 6.1 10.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 2.5 off 0 pos (20.0 5.0)" | yarp rpc /ctpservice/head/rpc
    sleep 3.0
    echo "ctpq time 1 off 0 pos (85.0 60)" | yarp rpc /ctpservice/right_hand/rpc
    echo "ctpq time 2.5 off 0 pos (41.7 14.9 -40.6 74.6 -33.8 0.0 6.1 10.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 1.3
    echo "ctpq time 1 off 0 pos (25.0 25)" | yarp rpc /ctpservice/right_hand/rpc
    echo "ctpq time 2.5 off 0 pos (41.7 14.9 -40.6 74.6 78.8 0.0 6.1 10.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 1.3
    echo "ctpq time 1 off 0 pos (85.0 60)" | yarp rpc /ctpservice/right_hand/rpc
    sleep 1.3
    echo "ctpq time 1 off 0 pos (25.0 25)" | yarp rpc /ctpservice/right_hand/rpc
    sleep 2.0
    go_home
}

look_watch() {
    echo "ctpq time 2.5 off 0 pos (41.7 14.9 -40.6 74.6 -40.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 2.5 off 0 pos (20.0 5.0)" | yarp rpc /ctpservice/head/rpc
    sleep 1.0
    echo "ctpq time 2.5 off 0 pos (41.7 14.9 -40.6 74.6 -54.5)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 3.0
    echo "ctpq time 2.5 off 0 pos (57.5 30 -40.8 85.0 -73.6)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 2.5 off 0 pos (15.6 5.6)" | yarp rpc /ctpservice/head/rpc
    sleep 1.5
    go_home
}

yawn() {
    echo "ctpq time 1.5 off 0 pos (-3.0 -23.4)" | yarp rpc /ctpservice/head/rpc
    echo "ctpq time 2.5 off 0 pos (75.90 75.20 -44.20 94.50 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 2.5 off 0 pos (75.90 75.20 -44.20 94.50 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 2.0
    echo "ctpq time 3.0 off 0 pos (-2.30 46.40 -40.80 5.40 0.00)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 3.0 off 0 pos (-2.30 46.40 -40.80 5.40 0.00)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 3.0
    go_home
}


wave() {
    echo "ctpq time 3.0 off 0 pos (17.25 48.0 44.2 94.5 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 2.0 off 0 pos (17.25 40.0 44.2 81.9 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    go_home
}

hello() {
    wave
    }


handshake() {
    echo "ctpq time 1.5 off 0 pos (40.0 12.44 -2.0 52.18 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos (42.5 12.44 -2.0 53.087 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos (40.0 12.44 -2.0 52.18 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 2.0 off 0 pos (40.0 12.44 -2.0 52.18 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    go_home
}

shake() {
    handshake
    }


#######################################################################################
# "MAIN" FUNCTION:                                                                    #
#######################################################################################

if [ $# -eq 0 ] ; then
    echo "No options were passed!"
    echo ""
    usage
    exit 1
fi

echo "Asked to execute : $1"
echo ""

# Check if the argument is a function
if [ "$(type "$1" 2>/dev/null)" ]; then
    echo "Executing '$1'"
    "$1"
    exit 0
else
    echo "'$1' has not been defined or is not a function"
    exit 1
fi

