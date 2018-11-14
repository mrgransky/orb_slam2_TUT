echo "-----------------------Configuring ORB_SLAM2-----------------------"
./build.sh

echo "-----------------------Building ORB_SLAM2(ROS) nodes-----------------------"


BUILD_TYPE=Release
NUM_PROC=4
BASEDIR="$PWD"

cd "$BASEDIR/Examples/ROS/ORB_SLAM2"
rm -rf cameraPub Mono MonoAR RGBD Stereo build/ lib/

#mkdir build
#cd build

#cmake -DROS_BUILD_TYPE=$BUILD_TYPE ..
#make -j$NUM_PROC
