set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

CMAKE_MINIMUM_REQUIRED(VERSION 3.15)

project(socket)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${OpenCV_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(client client.cpp)
add_executable(server server.cpp)