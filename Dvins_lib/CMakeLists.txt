cmake_minimum_required(VERSION 2.8.3)
project(Dvins)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

message("Build type:" ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3 -march=native")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(camera_model SHARED
    src/Camera_models/chessboard/Chessboard.cc
    src/Camera_models/calib/CameraCalibration.cc
    src/Camera_models/camera_models/Camera.cc
    src/Camera_models/camera_models/CameraFactory.cc
    src/Camera_models/camera_models/CostFunctionFactory.cc
    src/Camera_models/camera_models/PinholeCamera.cc
    src/Camera_models/camera_models/CataCamera.cc
    src/Camera_models/camera_models/EquidistantCamera.cc
    src/Camera_models/camera_models/ScaramuzzaCamera.cc
    src/Camera_models/sparse_graph/Transform.cc
    src/Camera_models/gpl/gpl.cc
    src/Camera_models/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${EIGEN3_LIBS})

add_library(${PROJECT_NAME} SHARED
    src/Estimator/System.cc
    src/FeatureTracker/Tracking.cc
    src/FeatureTracker/KeyFrame.cc
    src/FeatureTracker/Frame.cc
    src/IMU/ImuData.cc
    src/Utility/ConfigParam.cc
    )

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${CERES_LIBRARIES}
    ${Pangolin_LIBRARIES}
    camera_model
    boost_system
    glog
    )

# Test
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(run_euroc Test/run_euroc.cc)
target_link_libraries(run_euroc ${PROJECT_NAME})

add_executable(test_module Test/test_module.cc)
target_link_libraries(test_module ${PROJECT_NAME})
