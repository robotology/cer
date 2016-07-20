cd $YARP_DIR
git pull
make -j4

cd $icub_firmware_shared_DIR
git pull
cmake ..
make -j4

cd $ICUB_DIR
git pull
make -j4

cd $CER_DIR
git pull
make -j4

cd $CER_ROOT/../navigation/build
git pull
make -j4



