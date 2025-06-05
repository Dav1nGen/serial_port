cmake_minimum_required(VERSION 3.14)
project(serial_port)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

find_package(OpenCV REQUIRED)

add_executable(
  ${PROJECT_NAME}
  ./src/main.cpp
  ./src/serial_port.cpp
)

include_directories(
#   ${CERES_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
#   ${X11_LIBRARIES}
#   ${CERES_LIBRARIES}
#   pthread
#   Python3::Python
#   pybind11::pybind11
)