cmake_minimum_required(VERSION 3.15.0)

project(3dVision VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -o0 -Wall -g2 -ggdb")

set(DEMO_INCLUDE_PATH "")

find_package(Eigen3 REQUIRED)

if(Eigen3_FOUND)
message(STATUS "Eigen3 was Found:" ${EIGEN3_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
else()
message(FATAL_ERROR "Do Not Found Eigen3!!!")
endif()

#add_subdirectory(Calibration)
add_definitions(-DENABLE_DOUBLE_TYPE)

list(APPEND DEMO_INCLUDE_PATH "./Calibration" "./Calibration/HandEyeCalibration" "./StereoMatch" "./Common" "./")

FILE(GLOB_RECURSE CommonFiles "Common/*.h" "Common/*.hpp" "Common/*.cpp")
source_group("Common" FILES ${CommonFiles})

FILE(GLOB_RECURSE CalibrationFiles "Calibration/*.h" "Calibration/*.hpp" "Calibration/*.cpp" "Calibration/HandEyeCalibration/*.h" "Calibration/HandEyeCalibration/*.hpp" "Calibration/HandEyeCalibration/*.cpp")
source_group("Calibration" FILES ${CalibrationFiles})

FILE(GLOB_RECURSE MatchFiles "StereoMatch/*.h" "StereoMatch/*.hpp" "StereoMatch/*.cpp")
source_group("Match" FILES ${MatchFiles})

include_directories(${DEMO_INCLUDE_PATH})
add_executable(3dVision Test/demo.cpp ${CalibrationFiles} ${CommonFiles} ${MatchFiles})

