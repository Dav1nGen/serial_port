cmake_minimum_required(VERSION 3.14)
project(serial_port)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BUILD_TYPE Release) # Release mode

find_package(OpenCV REQUIRED)

file(GLOB SRC
  ${CMAKE_CURRENT_LIST_DIR}/src/*.c
  ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
)

add_executable(
  ${PROJECT_NAME}
  ${SRC}
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  ${OPENCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
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

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
  add_compile_options(-fno-omit-frame-pointer)
  # 定义 Sanitizers 选项
  option(ENABLE_SANITIZER_ADDRESS "Enable AddressSanitizer" ON)
  option(ENABLE_SANITIZER_UNDEFINED "Enable UndefinedBehaviorSanitizer" ON)
  option(ENABLE_SANITIZER_THEAD "Enable ThreadSanitizer" OFF)

  function(enable_sanitizers target)
    if(ENABLE_SANITIZER_ADDRESS)
      target_compile_options(${target} PRIVATE -fsanitize=address)
      target_link_options(${target} PRIVATE -fsanitize=address)
    endif()

    if(ENABLE_SANITIZER_UNDEFINED)
      target_compile_options(${target} PRIVATE -fsanitize=undefined)
      target_link_options(${target} PRIVATE -fsanitize=undefined)
    endif()

    if(ENABLE_SANITIZER_THREAD)
      target_compile_options(${target} PRIVATE -fsanitize=thread)
      target_link_options(${target} PRIVATE -fsanitize=thread)
    endif()

    target_compile_options(${target} PRIVATE
      -fno-omit-frame-pointer
      -g
    )
  endfunction()

  enable_sanitizers(${PROJECT_NAME})
endif()
