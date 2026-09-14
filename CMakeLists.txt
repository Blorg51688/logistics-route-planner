cmake_minimum_required(VERSION 3.20)
project(logistics_route_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- core：图结构与领域类型，纯 C++，不依赖 Qt ----
add_library(core STATIC
    src/core/Node.cpp
    src/core/LogisticsGraph.cpp
    src/core/MinHeap.cpp
    src/core/Dijkstra.cpp
    src/core/RoutePlanner.cpp
    src/core/Random.cpp
    src/core/GraphEdit.cpp
    src/core/Traffic.cpp
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

add_executable(minheap_tests tests/minheap_tests.cpp)
target_link_libraries(minheap_tests PRIVATE core)

add_executable(dijkstra_tests tests/dijkstra_tests.cpp)
target_link_libraries(dijkstra_tests PRIVATE core)

add_executable(routeplanner_tests tests/routeplanner_tests.cpp)
target_link_libraries(routeplanner_tests PRIVATE core)

add_executable(integration_tests tests/integration_tests.cpp)
target_link_libraries(integration_tests PRIVATE io)
target_compile_definitions(integration_tests PRIVATE
    DEFAULT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/default.ini")

add_executable(traffic_tests tests/traffic_tests.cpp)
target_link_libraries(traffic_tests PRIVATE core)

add_executable(graphedit_tests tests/graphedit_tests.cpp)
target_link_libraries(graphedit_tests PRIVATE core)

enable_testing()
add_test(NAME core_tests COMMAND core_tests)
add_test(NAME config_tests COMMAND config_tests)
add_test(NAME minheap_tests COMMAND minheap_tests)
add_test(NAME dijkstra_tests COMMAND dijkstra_tests)
add_test(NAME routeplanner_tests COMMAND routeplanner_tests)
add_test(NAME integration_tests COMMAND integration_tests)
add_test(NAME traffic_tests COMMAND traffic_tests)
add_test(NAME graphedit_tests COMMAND graphedit_tests)

# ---- 期望值预言机自检（开发工具，不参与交付程序；缺 python3 时自动跳过）----
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    add_test(NAME oracle_selfcheck
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/tools/oracle.py --check)
    # 校验 default.ini 确实出自生成器，且通过数据有效性自检
    add_test(NAME dataset_selfcheck
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_dataset.py)
else()
    message(STATUS "未找到 python3，跳过 oracle_selfcheck 测试")
endif()

# ---- GUI：Qt6 Widgets。未安装 Qt 时自动跳过，核心层与测试不受影响 ----
find_package(Qt6 QUIET COMPONENTS Widgets)
if(Qt6_FOUND)
    add_executable(app
        src/main.cpp
        src/gui/NodeItem.cpp
        src/gui/EdgeItem.cpp
        src/gui/GraphScene.cpp
        src/gui/MainWindow.cpp
    )
    target_link_libraries(app PRIVATE io Qt6::Widgets)
    target_compile_definitions(app PRIVATE
        DEFAULT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/default.ini")
    set_target_properties(app PROPERTIES AUTOMOC ON)
else()
    message(STATUS "未找到 Qt6 Widgets，跳过 GUI 目标（app）")
endif()
