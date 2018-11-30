# cmakeList of main project:
cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

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

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL 1.2 REQUIRED)

find_package(Cholmod)
find_package(BLAS)
find_package(LAPACK)
find_package(SuiteSparse REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/Thirdparty/g2o
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${SUITESPARSE_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc

src/IMU/configParam.cc
src/IMU/imuData.cc
src/IMU/imuPreintegrator.cc
src/IMU/navState.cc
src/IMU/so3.cc
src/IMU/g2otypes.cc

)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${Pangolin_LIBRARIES}
${PCL_COMMON_LIBRARIES} 
${PCL_IO_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so3


${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_calibration_odom_laser.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_cli.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_core.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_csparse_extension.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_ext_freeglut_minimal.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_hierarchical.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_incremental.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_interactive.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_interface.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_opengl_helper.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_parser.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_simulator.so

${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_cholmod.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_csparse.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_dense.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_eigen.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_pcg.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_slam2d_linear.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_structure_only.so

${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_stuff.so

${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_tutorial_slam2d.so

${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_data.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_icp.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sba.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sclam2d.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sim3.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam2d.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam2d_addons.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam3d.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam3d_addons.so


${SUITESPARSE_LIBRARIES}
${BLAS_LIBRARIES}
${LAPACK_LIBRARIES}
-lcholmod
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
add_executable(rgbd_tum Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)
add_executable(stereo_kitti Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
add_executable(mono Examples/Monocular/mono.cc)
target_link_libraries(mono ${PROJECT_NAME})

add_executable(mono_tum Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})
