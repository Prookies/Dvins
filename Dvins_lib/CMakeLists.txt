cmake_minimum_required(VERSION 2.8)
project(Dvins)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message("Build type:" ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3 -march=native")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
    src/System.cc
    src/Tracking.cc
    src/Optimizer.cc
    src/KeyFrame.cc
    src/Frame.cc
    src/Initializer.cc
    src/ImuData.cc
    src/ConfigParam.cc
    src/FeatureTracker.cc
    )

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${CERES_LIBRARIES}
    boost_system
    glog
    )

# Test
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(run_euroc Test/run_euroc.cc)
target_link_libraries(run_euroc ${PROJECT_NAME})

add_executable(test_module Test/test_module.cc)
target_link_libraries(test_module ${PROJECT_NAME})
