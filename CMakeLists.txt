SET(PROJECT_NAME slam2)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#set(CMAKE_VERBOSE_MAKEFILE ON)


#set(BUILD_TYPE Release)
#set(BUILD_TYPE RelWithDebInfo)
set(BUILD_TYPE Debug)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# required libraries
#find_package(SuiteParse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread) 
find_package(OpenCV REQUIRED)
find_package(Pangolin 0.2 QUIET)

# flags
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O3 -fPIC -g -std=c++0x -march=native"
#   "${SSE_FLAGS} -O3 -g -std=c++0x -fno-omit-frame-pointer"
)

# Sources files
set(slam_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/IO/DataSetReader.cpp
    ${PROJECT_SOURCE_DIR}/src/IO/ImageReader.cpp
    ${PROJECT_SOURCE_DIR}/src/Util/Undistorter.cpp
    ${PROJECT_SOURCE_DIR}/src/Util/Settings.cpp
    ${PROJECT_SOURCE_DIR}/src/Pangolin/SlamViewer.cpp
    ${PROJECT_SOURCE_DIR}/src/Odometry/Odometry.cpp
    ${PROJECT_SOURCE_DIR}/src/Odometry/Features.cpp
    ${PROJECT_SOURCE_DIR}/src/Odometry/Feature.cpp
    ${PROJECT_SOURCE_DIR}/src/Odometry/Filter.cpp
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src 
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS} 
) 


# compile main library.
include_directories( ${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR}) 
add_library(slam ${slam_SOURCE_FILES} ${slam_opencv_SOURCE_FILES} ${slam_pangolin_SOURCE_FILES})

#set_property( TARGET slam APPEND_STRING PROPERTY COMPILE_FLAGS -Wall )


# build main executable (only if we have both OpenCV and Pangolin)
message("--- compiling slam.")
add_executable(slam_dataset ${PROJECT_SOURCE_DIR}/src/main.cpp )
target_link_libraries(slam_dataset slam boost_system boost_thread cxsparse ${Pangolin_LIBRARIES} ${OpenCV_LIBS})
