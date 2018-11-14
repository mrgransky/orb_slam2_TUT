echo "-----------------------Compiling ORB_SLAM2-----------------------"

BUILD_TYPE=Release
NUM_PROC=4
BASEDIR="$PWD"

echo "Configuring and building Thirdparty/DBoW2 ..."

cd "$BASEDIR/Thirdparty/DBoW2"
rm -rf build/ lib/

#mkdir build
#cd build
#cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
#make -j$NUM_PROC

echo "Configuring and building Thirdparty/g2o ..."
cd "$BASEDIR/Thirdparty/g2o"
rm -rf build/ lib/

#mkdir build
#cd build
#cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
#make -j$NUM_PROC

echo "-----------------------Uncompressing vocabulary-----------------------"
cd "$BASEDIR/Vocabulary"
rm -rf ORBvoc.txt

#tar -xf ORBvoc.txt.tar.gz

echo "-----------------------Configuring & building ORB_SLAM2-----------------------"

cd "$BASEDIR/Examples/Monocular"
rm -rf mono mono_euroc mono_kitti mono_tum

cd "$BASEDIR/Examples/RGB-D"
rm -rf rgbd_tum

cd "$BASEDIR/Examples/Stereo"
rm -rf stereo_kitti stereo_euroc

cd "$BASEDIR"
rm -rf build/ lib/

#mkdir build
#cd build
#cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
#make -j$NUM_PROC

echo "-----------------------Compiling ORB_SLAM2 >>>(ROS)<<< nodes-----------------------"

cd "$BASEDIR/Examples/ROS/ORB_SLAM2"
rm -rf cameraPub Mono MonoAR RGBD Stereo build/ lib/

#mkdir build
#cd build
#cmake -DROS_BUILD_TYPE=$BUILD_TYPE ..
#make -j$NUM_PROC
