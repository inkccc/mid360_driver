cmake_minimum_required(VERSION 3.10)
project(mid360_driver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
    # march=native: SIMD/AVX 指令集 (参考 mid360_driver 项目)
    # -ffast-math -fno-finite-math-only: 快速数学运算但保留 NaN 守卫 (shouldKeepPoint 依赖)
    # -pipe: 管道加速编译 (减少磁盘 I/O)
    add_compile_options(-march=native -ffast-math -fno-finite-math-only -pipe)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE SOURCE src/*.cpp)
ament_auto_add_library(${PROJECT_NAME}_node_lib SHARED ${SOURCE})

rclcpp_components_register_node(${PROJECT_NAME}_node_lib
    PLUGIN mid360_driver::Mid360DriverNode
    EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
        ament_cmake_copyright
        ament_cmake_uncrustify
    )
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
    USE_SCOPED_HEADER_INSTALL_DIR
    INSTALL_TO_SHARE
    config
    launch
)
