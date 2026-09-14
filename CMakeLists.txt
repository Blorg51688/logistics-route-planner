cmake_minimum_required(VERSION 3.20)
project(logistics_route_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 核心层：纯 C++，不依赖 Qt，可脱离 GUI 独立编译与测试
add_library(core STATIC
    src/core/Node.cpp
    src/core/LogisticsGraph.cpp
)
target_include_directories(core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# 核心层测试（不依赖 Qt，不引入测试框架）
add_executable(core_tests tests/core_tests.cpp)
target_link_libraries(core_tests PRIVATE core)

enable_testing()
add_test(NAME core_tests COMMAND core_tests)

# GUI 目标：Qt6 已安装，待 gui 层源码就位后启用
# find_package(Qt6 REQUIRED COMPONENTS Widgets)
# add_executable(app src/main.cpp src/gui/MainWindow.cpp)
# target_link_libraries(app PRIVATE core Qt6::Widgets)
