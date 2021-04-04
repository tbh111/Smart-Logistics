project(Astar)
cmake_minimum_required(VERSION 3.5)
set(SOURCES src/main.cpp src/Astar.cpp)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${Astar_SOURCE_DIR}/bin)
add_executable(Astar ${SOURCES})
target_include_directories(Astar
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)
