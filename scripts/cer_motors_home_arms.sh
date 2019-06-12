#!/bin/sh

echo "stop" | yarp rpc /cer_reaching-controller/left/rpc
echo "stop" | yarp rpc /cer_reaching-controller/right/rpc

sleep 0.2
echo "set icmd cmod 0 pos"  | yarp rpc /cer/left_arm/rpc:i
echo "set icmd cmod 1 pos"  | yarp rpc /cer/left_arm/rpc:i
echo "set icmd cmod 2 pos"  | yarp rpc /cer/left_arm/rpc:i
echo "set icmd cmod 3 pos"  | yarp rpc /cer/left_arm/rpc:i
echo "set icmd cmod 4 pos"  | yarp rpc /cer/left_arm/rpc:i
echo "set icmd cmod 5 pos"  | yarp rpc /cer/left_arm/rpc:i
echo "set icmd cmod 6 pos"  | yarp rpc /cer/left_arm/rpc:i
echo "set icmd cmod 7 pos"  | yarp rpc /cer/left_arm/rpc:i

echo "set icmd cmod 0 pos"  | yarp rpc /cer/right_arm/rpc:i
echo "set icmd cmod 1 pos"  | yarp rpc /cer/right_arm/rpc:i
echo "set icmd cmod 2 pos"  | yarp rpc /cer/right_arm/rpc:i
echo "set icmd cmod 3 pos"  | yarp rpc /cer/right_arm/rpc:i
echo "set icmd cmod 4 pos"  | yarp rpc /cer/right_arm/rpc:i
echo "set icmd cmod 5 pos"  | yarp rpc /cer/right_arm/rpc:i
echo "set icmd cmod 6 pos"  | yarp rpc /cer/right_arm/rpc:i
echo "set icmd cmod 7 pos"  | yarp rpc /cer/right_arm/rpc:i

sleep 0.2

echo "set reca homs"  | yarp rpc /cer/left_arm/rpc:i
echo "set reca homs"  | yarp rpc /cer/right_arm/rpc:i
