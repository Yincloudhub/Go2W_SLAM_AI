cmake_minimum_required(VERSION 3.10)
project(go2w_cpp_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(go2w_plan_executor_core
    src/gateway_client.cpp
    src/plan_executor.cpp
    src/operator_panel.cpp
    src/semantic_router.cpp
)

target_include_directories(go2w_plan_executor_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

add_executable(go2w_plan_executor_dry_run
    src/plan_executor_main.cpp
)

target_link_libraries(go2w_plan_executor_dry_run PRIVATE go2w_plan_executor_core)

add_executable(go2w_operator_panel
    src/operator_panel_main.cpp
)

target_link_libraries(go2w_operator_panel PRIVATE go2w_plan_executor_core)
