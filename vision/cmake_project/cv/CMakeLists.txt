# project name
PROJECT(cv)
# requirement of cmake version
cmake_minimum_required(VERSION 3.5)

set(SOURCES src/vision.cpp src/main.cpp)

# set the directory of executable files
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${cv_SOURCE_DIR}/bin)

# find required opencv
find_package(OpenCV REQUIRED)
find_package(Eigen3)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
# directory of opencv headers
include_directories(${OpenCV_INCLUDE_DIRS})
# name of executable file and path of source file
add_executable(cv ${SOURCES})
target_include_directories(cv
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)

# directory of opencv library
link_directories(${OpenCV_LIBRARY_DIRS})
# opencv libraries
target_link_libraries(cv ${OpenCV_LIBS})
