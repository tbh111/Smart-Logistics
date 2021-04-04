project(Dijkstra)
cmake_minimum_required(VERSION 3.5)
set(SOURCES src/main.cpp src/dijkstra.cpp)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${Dijkstra_SOURCE_DIR}/bin)
add_executable(Dijkstra ${SOURCES})
target_include_directories(Dijkstra
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)
