cmake_minimum_required(VERSION 3.15...3.20)

project(roomba
    VERSION 0.1.0
    DESCRIPTION "Roomba in webots"
    LANGUAGES CXX)

set(WEBOTS_HOME "/usr/local/webots")

link_directories(${WEBOTS_HOME}/lib/controller)
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

find_package(spdlog 1.11 REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/controllers/bumper)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/controllers/simple)
