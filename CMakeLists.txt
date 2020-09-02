# Define project name

set( APPNAME camCalibr )
project( ${APPNAME} )

set(OpenCV_DIR /home/w/tmp_mine/v3s_lib/host/share/OpenCV)
# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

if(CMAKE_VERSION VERSION_LESS "2.8.11")
	# Add OpenCV headers location to your include paths
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
SET(CMAKE_INSTALL_RPATH "/home/w/tmp_mine/v3s_lib/host/lib")

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
aux_source_directory( ${CMAKE_CURRENT_SOURCE_DIR}/src DIR_SRCS)

# Declare the executable target built from your sources
add_executable(${APPNAME} ${DIR_SRCS} calibration_main.cpp)
add_executable(undistort ${DIR_SRCS} undistort_main.cpp)
add_executable(extCalibra extCalibra.cpp)
add_executable(orbTest orbTest.cpp src/tools.cpp)
add_executable(uv2xyz uv2xyz.cpp src/tools.cpp)

# Link your application with OpenCV libraries
target_link_libraries(${APPNAME} ${OpenCV_LIBS})
target_link_libraries(undistort ${OpenCV_LIBS})
target_link_libraries(extCalibra ${OpenCV_LIBS})
target_link_libraries(orbTest ${OpenCV_LIBS})
target_link_libraries(uv2xyz ${OpenCV_LIBS})

