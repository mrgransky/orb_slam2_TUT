

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

echo "Uncompress vocabulary ..."
cd "$BASEDIR/Vocabulary"
rm -rf ORBvoc.txt

#tar -xf ORBvoc.txt.tar.gz

echo "Configuring and building ORB_SLAM2 ..."
cd "$BASEDIR"
rm -rf build/ lib/


cd "$BASEDIR/Examples/Monocular"
rm -rf mono mono_euroc mono_kitti mono_tum


cd "$BASEDIR/Examples/RGB-D"
rm -rf rgbd_tum

cd "$BASEDIR/Examples/Stereo"
rm -rf stereo_kitti stereo_euroc

#mkdir build
#cd build
#cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
#make -j$NUM_PROC
