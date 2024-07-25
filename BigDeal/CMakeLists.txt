set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

CMAKE_MINIMUM_REQUIRED(VERSION 3.15)

project(BigDeal)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${OpenCV_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(cv main.cpp src/number_classifier.cpp src/KalmanFilter.cpp)



target_link_libraries(cv ${OpenCV_LIBS})
target_link_libraries(cv Eigen3::Eigen)