cmake_minimum_required(VERSION 3.20)
project(logistics_route_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- core：图结构与领域类型，纯 C++，不依赖 Qt ----
add_library(core STATIC
    src/core/Node.cpp
    src/core/LogisticsGraph.cpp
)
target_include_directories(core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# ---- io：配置解析与加载，同样不依赖 Qt ----
add_library(io STATIC
    src/io/IniFile.cpp
    src/io/ConfigLoader.cpp
)
target_link_libraries(io PUBLIC core)
target_include_directories(io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# ---- 测试：均可脱离 Qt 独立编译运行 ----
add_executable(core_tests tests/core_tests.cpp)
target_link_libraries(core_tests PRIVATE core)

add_executable(config_tests tests/config_tests.cpp)
target_link_libraries(config_tests PRIVATE io)
target_compile_definitions(config_tests PRIVATE
    DEFAULT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/default.ini")

enable_testing()
add_test(NAME core_tests COMMAND core_tests)
add_test(NAME config_tests COMMAND config_tests)

# ---- GUI：Qt6 已安装，待 gui 层源码就位后启用 ----
# find_package(Qt6 REQUIRED COMPONENTS Widgets)
# add_executable(app src/main.cpp src/gui/MainWindow.cpp)
# target_link_libraries(app PRIVATE io Qt6::Widgets)
