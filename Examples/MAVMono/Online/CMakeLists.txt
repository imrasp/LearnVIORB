cmake_minimum_required(VERSION 2.4.6) 
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

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

add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)
add_definitions(-DBOOST_ALL_DYN_LINK)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake_modules)

find_package(OpenCV 3.0.0 REQUIRED)
#find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
# PCL #######################
find_package(PCL 1.7 REQUIRED COMPONENTS common io)

include_directories(
		${PROJECT_SOURCE_DIR}
		${PROJECT_SOURCE_DIR}/../../../
		${PROJECT_SOURCE_DIR}/../../../src
		${PROJECT_SOURCE_DIR}/../../../include
		${PROJECT_SOURCE_DIR}/../../../Thirdparty/mavlink/include/mavlink/v2.0
		${PROJECT_SOURCE_DIR}/../../../Thirdparty/cpr/include
		${PROJECT_SOURCE_DIR}/../../../Thirdparty/json/include
        ${Pangolin_INCLUDE_DIRS}
#        ${PCL_INCLUDE_DIRS}
)

set(LIBS
	-lpthread
        ${EIGEN3_LIBS}
        -lboost_date_time
        -lboost_chrono
        -lboost_system
        -lboost_thread
        -lboost_filesystem
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARY_DIRS}
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
        ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
		${PROJECT_SOURCE_DIR}/../../../Thirdparty/cpr
        )

# Node for monocular camera
add_executable(MonoMAV_online
	
        src/mono_mav_live.cpp

	src/MAVCom/autopilot_interface.cpp
	src/MAVCom/autopilot_interface.h
	src/MAVCom/camera_recorder.cpp
	src/MAVCom/camera_recorder.h
	src/MAVCom/imu_recorder.cpp
	src/MAVCom/imu_recorder.h
	src/MAVCom/location_manager.cpp
	src/MAVCom/location_manager.h
	src/MAVCom/log.cpp
	src/MAVCom/log.h
	src/MAVCom/mavlink_control.cpp
	src/MAVCom/mavlink_control.h
	src/MAVCom/serial_port.cpp
	src/MAVCom/serial_port.h
        )

target_link_libraries(MonoMAV_online
        ${LIBS}
        )
