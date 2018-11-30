echo "----------------------- ORB_SLAM2 -----------------------"
echo ""

BUILD_TYPE=Release
NUM_PROC=8
BASEDIR="$PWD"

echo "-------------------Configuring and building Thirdparty/DBoW2------------------------"

cd "$BASEDIR/Thirdparty/DBoW2"
rm -rf build/ lib/

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
make -j$NUM_PROC

echo "-------------------Configuring and building Thirdparty/g2o-------------------"
#echo "already installed library"
echo ""

cd "$BASEDIR/Thirdparty/g2o"
rm -rf config.h build/ lib/ bin/

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
make -j$NUM_PROC

echo "-----------------------Uncompressing vocabulary-----------------------"

echo ""
cd "$BASEDIR/Vocabulary"
rm -rf ORBvoc.txt

tar -xf ORBvoc.txt.tar.gz

echo ""

echo "----------------------- Building ORB_SLAM2 project-----------------------"

cd "$BASEDIR/Examples/Monocular"
rm -rf mono mono_euroc mono_kitti mono_tum

cd "$BASEDIR/Examples/RGB-D"
rm -rf rgbd_tum

cd "$BASEDIR/Examples/Stereo"
rm -rf stereo_kitti stereo_euroc

cd "$BASEDIR"
rm -rf build/ lib/

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
make -j$NUM_PROC

echo "-----------------------Compiling ---VISION ONLY--- ORB_SLAM2 >>>(ROS)<<< nodes-----------------------"

cd "$BASEDIR/Examples/ROS/ORB_SLAM2"
rm -rf cameraPub Mono MonoAR RGBD Stereo build/ lib/

mkdir build
cd build
cmake -DROS_BUILD_TYPE=$BUILD_TYPE ..
make -j$NUM_PROC


#echo "-----------------------Compiling ---VISUAL INERTIAL--- ORB_SLAM2 >>>(ROS)<<< nodes-----------------------"

#cd "$BASEDIR/Examples/ROS/ORB_VISLAM"
#rm -rf build/ lib/

#mkdir build
#cd build
#cmake -DROS_BUILD_TYPE=$BUILD_TYPE ..
#make -j$NUM_PROC

#echo ""
#echo "Launch file in Examples/ROS/ORB_VISLAM/launch."
#echo "Modify the configuration file config/euroc.yaml"
#echo "Run as: roslaunch ORB_VIO testeuroc.launch"
#echo ""

