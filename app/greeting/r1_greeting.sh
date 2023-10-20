
 #!/bin/bash

#######################################################################################
# HELP
#######################################################################################
usage() {
cat << EOF

***************************************************************************************
R1 GREETING SCRIPTING

This script scripts through the commands available for the navigation of R1

USAGE:
        $0 [robot ( SIM_CER_ROBOT / cer )] [options] [other param]

***************************************************************************************
OPTIONS:
		wait_till_quiet
		speak    [sentence]
		go_home  [period]
		arms_up
		arms_down
		torso_up
		torso_down
		head_up
		head_down
		look_gripper
		gripper_move
		all_down
		wake_up
		wake_up_head
		reach_right
		reach_left
		rotate_base_left
		rotate_base_right
		rotate_base_home
		salute
		arms_cart
		arms_relieve
		close_hands
		open_hands
		arm_bottle
		arm_trash
		close_bottle
		sequence_1
		sequence_2
		say_hi
		queen_betty
***************************************************************************************
EXAMPLE USAGE:

***************************************************************************************
EOF
}


#######################################################################################
# HELPER FUNCTIONS
#######################################################################################

wait_till_quiet() {
    sleep 0.3
    isSpeaking=$(echo "stat" | yarp rpc /iSpeak/rpc)
    while [ "$isSpeaking" == "Response: speaking" ]; do
        isSpeaking=$(echo "stat" | yarp rpc /iSpeak/rpc)
        sleep 0.1
        # echo $isSpeaking
    done
    echo "I'm not speaking any more :)"
    echo $isSpeaking
}

speak() {
    echo "\"$2\"" | yarp write ... /iSpeak
}

go_home_helper() {
    go_home_helperR $1 $2
    go_home_helperL $1 $2
}

go_home_helperL()
{
    echo "ctpq time $2 off 0 pos (7.0 10.0 -10.0 21.5 0.0 0.0 -0.0 -0.0)" | yarp rpc /ctpservice/left_arm/rpc
}

go_home_helperR()
{
    echo "ctpq time $2 off 0 pos (7.0 15.0 -10.0 12.5 0.0 0.0 -0.0 -0.0)" | yarp rpc /ctpservice/right_arm/rpc
}

go_home()
{
    go_home_helper $1 4.0
}

arms_up() {
    echo "ctpq time 3.0 off 0 pos (9.0 25.0 -10.0 31.5 0.0 0.0 -0.0 -0.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 3.0 off 0 pos (9.0 25.0 -10.0 31.5 0.0 0.0 -0.0 -0.0)" | yarp rpc /ctpservice/right_arm/rpc
}

arms_down() {
    echo "ctpq time 3.0 off 0 pos (-9.0 15.0 -10.0 50.5 0.0 0.0 -0.0 -0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 3.0 off 0 pos (-9.0 15.0 -10.0 50.5 0.0 0.0 -0.0 -0.0)" | yarp rpc /ctpservice/left_arm/rpc
}

torso_up() {
    echo "ctpq time 6.0 off 0 pos (0.14 0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
}

torso_down() {
    echo "ctpq time 6.0 off 0 pos (0.03 0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
}

head_down() {
    echo "ctpq time 2.5 off 0 pos (30.0 0.0)" | yarp rpc /ctpservice/head/rpc
}

head_up() {
    echo "ctpq time 2.5 off 0 pos (0.0 0.0)" | yarp rpc /ctpservice/head/rpc
}

look_gripper() {
    echo "ctpq time 2.5 off 0 pos (41.7 14.9 -40.6 74.6 78.8 0.0 6.1 10.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 2.5 off 0 pos (20.0 5.0)" | yarp rpc /ctpservice/head/rpc
}

gripper_move() {
    echo "ctpq time 1 off 0 pos (85.0 60)" | yarp rpc /ctpservice/left_hand/rpc
    echo "ctpq time 2.5 off 0 pos (41.7 14.9 -40.6 74.6 -33.8 0.0 6.1 10.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.3
    echo "ctpq time 1 off 0 pos (25.0 25)" | yarp rpc /ctpservice/left_hand/rpc
    echo "ctpq time 2.5 off 0 pos (41.7 14.9 -40.6 74.6 78.8 0.0 6.1 10.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.3
    echo "ctpq time 1 off 0 pos (85.0 60)" | yarp rpc /ctpservice/left_hand/rpc
    sleep 1.3
    echo "ctpq time 1 off 0 pos (25.0 25)" | yarp rpc /ctpservice/left_hand/rpc
}

all_down() {
    go_home
    torso_down
    head_down
    echo "blck" | yarp rpc /faceExpressionImage/rpc
}

wake_up() {
    head_up
    echo "rst" | yarp rpc /faceExpressionImage/rpc
    torso_up
    arms_up
}

wake_up_head() {
    head_up
    echo "rst" | yarp rpc /faceExpressionImage/rpc
    torso_up
    arms_up
    
    sleep 2
    echo "ctpq time 2.0 off 0 pos (0.0 40.0)" | yarp rpc /ctpservice/head/rpc
    sleep 2
    echo "ctpq time 2.0 off 0 pos (0.0 -40.0)" | yarp rpc /ctpservice/head/rpc
}

reach_right() {

    echo "ctpq time 3.5 off 0 pos (9.0 25.0 -10.0 31.5 0.0 0.0 -0.0 -0.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 3.5 off 0 pos (32.5 52.2 -9.9 8.8 0.0 0.1 0.1 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 2.5 off 0 pos (20.0 -42.0)" | yarp rpc /ctpservice/head/rpc
    #echo "ctpq time 3.0 off 0 pos (0.05 -0.0 -10.0 0.0)" | yarp rpc /ctpservice/torso/rpc
    echo "set vels (0.013 0.013 0.013 0.0)" | yarp rpc /$1/torso/rpc:i
    echo "set poss (0.05 -0.0 -22.0 0.0)" | yarp rpc /$1/torso/rpc:i
}

reach_left() {

    echo "ctpq time 3.0 off 0 pos (9.0 25.0 -10.0 31.5 0.0 0.0 -0.0 -0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 3.0 off 0 pos (32.5 52.2 -9.9 8.8 0.0 0.1 0.1 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 2.5 off 0 pos (20.0 42.0)" | yarp rpc /ctpservice/head/rpc
    #echo "ctpq time 3.0 off 0 pos (0.05 -0.0 10.0 0.0)" | yarp rpc /ctpservice/torso/rpc
    echo "set vels (0.008 0.008 0.008 0.0)" | yarp rpc /$1/torso/rpc:i
    echo "set poss (0.05 -0.0 22.0 0.0)" | yarp rpc /$1/torso/rpc:i
}

rotate_base_left() {

    echo "ctpq time 2.0 off 0 pos (0.10 0.0 0.0 -28.0)" | yarp rpc /ctpservice/torso/rpc
    echo "ctpq time 3.0 off 0 pos (0.0 -16.0)" | yarp rpc /ctpservice/head/rpc
    for i in {1..20}
    do 
        echo "set vmos (-25.0 25.0)"| yarp rpc /$1/mobile_base/rpc:i
        sleep 0.05
    done
}

rotate_base_right() {

    echo "ctpq time 4.3 off 0 pos (0.10 0.0 0.0 28.0)" | yarp rpc /ctpservice/torso/rpc
    echo "ctpq time 3.0 off 0 pos (0.0 16.0)" | yarp rpc /ctpservice/head/rpc
    for i in {1..40}
    do 
        echo "set vmos (25.0 -25.0)"| yarp rpc /$1/mobile_base/rpc:i
        sleep 0.05
    done
}

rotate_base_home() {

    echo "ctpq time 3.0 off 0 pos (0.10 0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
    echo "ctpq time 3.0 off 0 pos (0.0 0.0)" | yarp rpc /ctpservice/head/rpc
    for i in {1..20}
    do 
        echo "set vmos (-25.0 25.0)"| yarp rpc /$1/mobile_base/rpc:i
        sleep 0.05
    done
}

salute() {
    echo "set vels (0.015 0.015 0.015 0.0)" | yarp rpc /$1/torso/rpc:i
    echo "set poss (0.10 -24.0 0.0 0.0)" | yarp rpc /$1/torso/rpc:i
    echo "ctpq time 2.0 off 0 pos (0.0 0.0)" | yarp rpc /ctpservice/head/rpc
    echo "ctpq time 3.0 off 0 pos (-9.0 15.0 -10.0 50.5 0.0 0.0 -0.0 -0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 3.0 off 0 pos (-9.0 15.0 -10.0 50.5 0.0 0.0 -0.0 -0.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 3
    echo "ctpq time 2.0 off 0 pos (-24.0 0.0)" | yarp rpc /ctpservice/head/rpc
    sleep 1
    echo "set vels (0.01 0.01 0.01 0.0)" | yarp rpc /$1/torso/rpc:i
    echo "set poss (0.10 0.0 0.0 0.0)" | yarp rpc /$1/torso/rpc:i
    echo "ctpq time 4 off 0 pos (0.0 0.0)" | yarp rpc /ctpservice/head/rpc
    arms_up
}

arms_cart() {
    echo "set vels (0.01 0.01 0.01 0.0)" | yarp rpc /$1/torso/rpc:i
    echo "set poss (0.10 0.0 0.0 0.0)" | yarp rpc /$1/torso/rpc:i
    echo "ctpq time 3.0 off 0 pos (23.2 3.2 -1.7 67.5 -81.3 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 3.0 off 0 pos (23.1 2.1 -1.7 74.5 -84.2 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
}

arms_relieve() {
    #echo "ctpq time 2.3 off 0 pos (0.15 0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
    
    echo "set vels (0.02 0.02 0.02 0.0)" | yarp rpc /$1/torso/rpc:i
    echo "set poss (0.12 7.0 0.0 0.0)" | yarp rpc /$1/torso/rpc:i

    for i in {1..15}
    do 
        echo "3.0 -0.05 -0.0 0.0 100.0 0.0 0.0 0.0 0.0"| yarp write ... /baseControl/aux_control:i
        sleep 0.025
    done

    arms_down
    sleep 1
    echo "set poss (0.10 0.0 0.0 0.0)" | yarp rpc /$1/torso/rpc:i
}


close_hands() {
    echo "ctpq time 1 off 0 pos (97.0 87)" | yarp rpc /ctpservice/left_hand/rpc
    echo "ctpq time 1 off 0 pos (97.0 87)" | yarp rpc /ctpservice/right_hand/rpc
}

open_hands() {
    echo "ctpq time 1 off 0 pos (25.0 25)" | yarp rpc /ctpservice/left_hand/rpc
    echo "ctpq time 1 off 0 pos (25.0 25)" | yarp rpc /ctpservice/right_hand/rpc
}

arm_bottle() {

    echo "ctpq time 3.0 off 0 pos (63.4 14.8 17.0 50.4 -1.1 0.0 0.2 0.09)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 4.0
    echo "ctpq time 2 off 0 pos (70.0 70.0)" | yarp rpc /ctpservice/left_hand/rpc
    sleep 2.0
    echo "ctpq time 2.0 off 0 pos (20.0 14.8 -12.0 54.4 0.0 0.0 0.0 0.00)" | yarp rpc /ctpservice/left_arm/rpc
    
}

arm_trash() {

    echo "ctpq time 3.0 off 0 pos (43.4 14.8 -12.0 50.4 0.0 0.1 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 6.0
    echo "ctpq time 1 off 0 pos (25.0 25.0)" | yarp rpc /ctpservice/left_hand/rpc
    sleep 2.0
    for i in {1..50}
    do 
        echo "3.0 -0.05 -0.0 0.0 100.0 0.0 0.0 0.0 0.0"| yarp write ... /baseControl/aux_control:i
        sleep 0.025
    done

    arms_down
    
}

close_bottle() {
    echo "ctpq time 1 off 0 pos (25.0 25)" | yarp rpc /ctpservice/left_hand/rpc
}




#######################################################################################
# "SEQUENCES" FUNCTION:                                                               #
#######################################################################################

sequence_1() {
    wake_up_head
    sleep 3
    look_gripper
    sleep 3
    gripper_move
    sleep 3
    reach_right
    sleep 9 
    reach_left
    sleep 9 
    wake_up
}

sequence_2() {

    echo "ctpq time 2.0 off 0 pos (0.10 0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
    sleep 1.5 
    rotate_base_left 
    reach_right
    sleep 1.5 
    rotate_base_right
    sleep 1.5
    rotate_base_home
}

say_hi() {
    
    echo "ctpq time 2.0 off 0 pos (17.25 48.0 44.2 94.5 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos (17.25 40.0 44.2 81.9 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos (17.25 48.0 44.2 94.5 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos (17.25 40.0 44.2 81.9 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    arms_down	
}

queen_betty() {
	
    echo "ctpq time 3.0 off 0 pos (51.75 25.6 15.3 88.2 -40.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos (51.75 25.6 15.3 88.2 10.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos (51.75 25.6 15.3 88.2 -40.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos (51.75 25.6 15.3 88.2 10.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos (51.75 25.6 15.3 88.2 -40.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos (51.75 25.6 15.3 88.2 10.0)" | yarp rpc /ctpservice/right_arm/rpc
    arms_down
}

#######################################################################################
# "MAIN" FUNCTION:                                                                    #
#######################################################################################
echo "********************************************************************************"
echo ""

$2 "$3"

if [[ $# -eq 0 ]] ; then
    echo "No options were passed!"
    echo ""
    usage
    exit 1
fi
