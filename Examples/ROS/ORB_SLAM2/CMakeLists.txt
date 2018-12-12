# cmakeLists for Vision ONLY ORB_SLAM2
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package (Eigen3 3.1.0 REQUIRED)
find_package (Pangolin REQUIRED)
find_package (PCL 1.2 REQUIRED)
find_package (octomap REQUIRED)

find_package(Cholmod)
find_package(BLAS)
find_package(LAPACK)
find_package(SuiteSparse REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
${OCTOMAP_INCLUDE_DIRS}
${SUITESPARSE_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${Pangolin_LIBRARIES}
${OCTOMAP_LIBRARIES}

${SUITESPARSE_LIBRARIES}
${BLAS_LIBRARIES}
${LAPACK_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so

#${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so


${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_calibration_odom_laser.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_cli.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_core.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_csparse_extension.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_ext_freeglut_minimal.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_hierarchical.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_incremental.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_interactive.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_interface.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_opengl_helper.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_parser.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_simulator.so

${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_solver_cholmod.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_solver_csparse.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_solver_dense.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_solver_eigen.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_solver_pcg.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_solver_slam2d_linear.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_solver_structure_only.so

${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_stuff.so

${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_tutorial_slam2d.so

${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_types_data.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_types_icp.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_types_sba.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_types_sclam2d.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_types_sim3.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_types_slam2d.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_types_slam2d_addons.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_types_slam3d.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o_types_slam3d_addons.so


${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
-lboost_system
)

# Node for publish camera
rosbuild_add_executable(cameraPub
src/cameraPub.cc
)

target_link_libraries(cameraPub
${LIBS}
)

# Node for monocular camera
rosbuild_add_executable(RT_Mono
src/ros_mono.cc
)

target_link_libraries(RT_Mono
${LIBS}
)

# Node for monocular camera (Augmented Reality Demo)
rosbuild_add_executable(MonoAR
src/AR/ros_mono_ar.cc
src/AR/ViewerAR.h
src/AR/ViewerAR.cc
)

target_link_libraries(MonoAR
${LIBS}
)

# Node for stereo camera
rosbuild_add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
)

# Node for RGB-D camera
rosbuild_add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
)

