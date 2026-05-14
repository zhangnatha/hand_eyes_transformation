# Pose6d / 向量运算；仅需 Eigen3。在仓库根目录：
#   cmake -B build -S . && cmake --build build
cmake_minimum_required(VERSION 3.14)
project(pose_vector LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
    find_path(
        EIGEN3_INCLUDE_DIR
        NAMES Eigen/Core
        PATHS
            /usr/include/eigen3
            /usr/local/include/eigen3
            "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen"
        DOC "Eigen 头文件路径（例如 libeigen3-dev，或 -DEIGEN3_INCLUDE_DIR）"
    )
    if(NOT EIGEN3_INCLUDE_DIR)
        message(
            FATAL_ERROR
                "未找到 Eigen3。请安装开发包（如 libeigen3-dev），或指定：\n"
                "  cmake -DEIGEN3_INCLUDE_DIR=<eigen目录> ..."
        )
    endif()
    add_library(Eigen3_Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3_Eigen INTERFACE "${EIGEN3_INCLUDE_DIR}")
    add_library(Eigen3::Eigen ALIAS Eigen3_Eigen)
endif()

add_library(pose_vector_math INTERFACE)
target_include_directories(
    pose_vector_math INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/interface"
)
target_link_libraries(pose_vector_math INTERFACE Eigen3::Eigen)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE pose_vector_math)
