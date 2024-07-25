CMAKE_MINIMUM_REQUIRED(VERSION 3.15)

project(kalman)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${OpenCV_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(cv main.cpp KalmanFilter.cpp)


target_link_libraries(cv ${OpenCV_LIBS})
target_link_libraries(cv Eigen3::Eigen)