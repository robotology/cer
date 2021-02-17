 #!/bin/bash
 
home_arms() {
    echo "ctpq time 2.0 off 0 pos (-10.047 10.7227 -10.0196 34.9806 0.0659181 0.0297929 -0.0941737 -0.22836)" | yarp write ... /ctpservice/left_arm/rpc
    echo "ctpq time 2.0 off 0 pos (-9.96462 14.1504 -10.0196 34.9806 0.961306 0.0297633 -0.0753394 0.130492)" | yarp write ... /ctpservice/right_arm/rpc
}

home_head() {
    echo "ctpq time 2.0 off 0 pos (0.0 0.0)" | yarp write ... /ctpservice/head/rpc
}
 
left() { #swing_arms()
    echo $$ > swing_arms_pid.txt
    RUNS=5 #${1:-5}
    TIME=2.0  #${2:-2.0}
    TIMESLEEP=$TIME #$(bc <<< "$TIME * 1.0")
    RANGE=25 #${3:-25.0}
    BACKRIGHT=-18.0 #-20.16
    BACKLEFT=-18.0 #-20.1545
    FRONTLEFT=7.0 #$(bc <<< "$BACKLEFT + $RANGE")
    FRONTRIGHT=7.0 #$(bc <<< "$BACKRIGHT + $RANGE")
    for i in $(seq 1 $RUNS)
    do
        echo "ctpq time $TIME off 0 pos ($FRONTRIGHT 13.9746 -10.0196 34.9806 0.351563 0.0297692 -0.0565046 0.0978689)" | yarp write ... /ctpservice/right_arm/rpc
        echo "ctpq time $TIME off 0 pos ($BACKLEFT 10.8106 -10.1074 34.9806 0.131836 0.0297929 -0.0941737 -0.22836)" | yarp write ... /ctpservice/left_arm/rpc
        sleep $TIMESLEEP
        echo "ctpq time $TIME off 0 pos ($BACKRIGHT 14.2383 -10.0196 34.9806 0.928347 0.0297692 -0.0565046 0.0978689)" | yarp write ... /ctpservice/right_arm/rpc
        echo "ctpq time $TIME off 0 pos ($FRONTLEFT 10.8106 -10.0196 34.9806 -0.0988772 0.0298047 -0.0565042 -0.22836)" | yarp write ... /ctpservice/left_arm/rpc
        sleep $TIMESLEEP
    done
    
    home_arms
}
 
right() { #look_around()
    echo $$ > look_around_pid.txt
    RUNS=5 #${1:-5}
    TIME=2.0 #${2:-2.0}
    RANGEYAW=10.0 #${3:-10.0}
    RANGEPITCH=5.0 #${4:-5.0}
    
    for i in $(seq 1 $RUNS)
    do
        echo "ctpq time $TIME off 0 pos (0.0 0.0)" | yarp write ... /ctpservice/head/rpc
        sleep $TIME
        echo "ctpq time $TIME off 0 pos (0.0 $RANGEYAW)" | yarp write ... /ctpservice/head/rpc
        sleep $TIME
        echo "ctpq time $TIME off 0 pos (0.0 -$RANGEYAW)" | yarp write ... /ctpservice/head/rpc
        sleep $TIME
        sleep $TIME
        echo "ctpq time $TIME off 0 pos ($RANGEPITCH -$RANGEYAW)" | yarp write ... /ctpservice/head/rpc
        sleep $TIME
        echo "ctpq time $TIME off 0 pos ($RANGEPITCH $RANGEYAW)" | yarp write ... /ctpservice/head/rpc
        sleep $TIME
        echo "ctpq time $TIME off 0 pos (0.0 0.0)" | yarp write ... /ctpservice/head/rpc
        sleep $TIME
        sleep $TIME
        echo "ctpq time $TIME off 0 pos ($RANGEPITCH $RANGEYAW)" | yarp write ... /ctpservice/head/rpc
        sleep $TIME
        echo "ctpq time $TIME off 0 pos ($RANGEPITCH -$RANGEYAW)" | yarp write ... /ctpservice/head/rpc
        sleep $TIME
        sleep $TIME
        echo "ctpq time $TIME off 0 pos ($RANGEPITCH 0.0)" | yarp write ... /ctpservice/head/rpc
        sleep $TIME
        echo "ctpq time $TIME off 0 pos (0.0 $RANGEYAW)" | yarp write ... /ctpservice/head/rpc
        sleep $TIME
    done
    
    home_head
}

down() { #kill_all
	kill -9 $(cat swing_arms_pid.txt)
    sleep 1 
    ./actions_dg.sh home_arms
    kill -9 $(cat look_around_pid.txt)
    sleep 1 
    ./actions_dg.sh home_head
}
#######################################################################################
# "MAIN" FUNCTION:                                                                    #
#######################################################################################
echo "********************************************************************************"
echo ""

$1 $2 $3 $4 $5

if [[ $# -eq 0 ]] ; then
    echo "No options were passed!"
    echo ""
    usage
    exit 1
fi
