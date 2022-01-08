cmake_minimum_required(VERSION 3.6)
project(target)

#set(OpenCV_DIR "/Users/zhaocongyang/Documents/opencv/release")
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp structure_light.cpp struct_light_calib.cpp structlight.cpp)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(/usr/local/include/opencv2)
#include_directories(/usr/local/include)
#find_library(LUALIB /usr/local/lib)
#set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_tracking)

add_executable(target ${SOURCE_FILES})
target_link_libraries(target ${OpenCV_LIBS})

