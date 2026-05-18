cmake_minimum_required(VERSION 3.10)
project(slam_gateway_cpp_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(UNITREE_SDK2_ROOT "" CACHE PATH "Path to unitree_sdk2 root")
if(NOT UNITREE_SDK2_ROOT)
    if(DEFINED ENV{UNITREE_SDK2_ROOT})
        set(UNITREE_SDK2_ROOT $ENV{UNITREE_SDK2_ROOT})
    elseif(EXISTS "/home/unitree/unitree_sdk2")
        set(UNITREE_SDK2_ROOT "/home/unitree/unitree_sdk2")
    elseif(EXISTS "/home/ysy/unitree_sdk2")
        set(UNITREE_SDK2_ROOT "/home/ysy/unitree_sdk2")
    endif()
endif()

message(STATUS "UNITREE_SDK2_ROOT=${UNITREE_SDK2_ROOT}")

set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(SDK_INCLUDE_DIRS "")
set(SDK_LIBRARY_DIRS "")
if(UNITREE_SDK2_ROOT)
    list(APPEND SDK_INCLUDE_DIRS
        ${UNITREE_SDK2_ROOT}/include
        ${UNITREE_SDK2_ROOT}/thirdparty/include
        ${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx
    )
    list(APPEND SDK_LIBRARY_DIRS
        ${UNITREE_SDK2_ROOT}/lib
        ${UNITREE_SDK2_ROOT}/lib/aarch64
        ${UNITREE_SDK2_ROOT}/lib/x86_64
        ${UNITREE_SDK2_ROOT}/thirdparty/lib
        ${UNITREE_SDK2_ROOT}/thirdparty/lib/aarch64
        ${UNITREE_SDK2_ROOT}/thirdparty/lib/x86_64
    )
endif()


# Unitree GO2 images commonly ship nlohmann/json.hpp with the ROS2 examples
# and CycloneDDS C++ headers under ddscxx/. Add only existing locations.
foreach(EXTRA_INCLUDE_DIR
    /home/unitree/cyclonedds_ws/src/unitree_ros2/example/src/include
    /home/unitree/ws_backup_merge/go2_msgs_ws_src/unitree_ros2/example/src/include
    /usr/local/include/ddscxx
)
    if(EXISTS ${EXTRA_INCLUDE_DIR})
        list(APPEND SDK_INCLUDE_DIRS ${EXTRA_INCLUDE_DIR})
    endif()
endforeach()

list(APPEND SDK_LIBRARY_DIRS /usr/local/lib /usr/lib /usr/lib/aarch64-linux-gnu)

find_library(UNITREE_SDK2_LIB
    NAMES unitree_sdk2 unitree_robot_sdk unitree_robot_sdk2
    HINTS ${SDK_LIBRARY_DIRS}
)

find_library(DDSC_LIB NAMES ddsc HINTS ${SDK_LIBRARY_DIRS})
find_library(DDSCXX_LIB NAMES ddscxx HINTS ${SDK_LIBRARY_DIRS})

message(STATUS "UNITREE_SDK2_LIB=${UNITREE_SDK2_LIB}")
message(STATUS "DDSC_LIB=${DDSC_LIB}")
message(STATUS "DDSCXX_LIB=${DDSCXX_LIB}")

add_library(slam_gateway_core
    src/slam_gateway.cpp
    src/topology_manager.cpp
    src/lidar_geometry_perception.cpp
    src/safety_supervisor.cpp
    src/llm_command_processor.cpp
)

target_include_directories(slam_gateway_core PUBLIC
    ${PROJECT_INCLUDE_DIR}
    ${SDK_INCLUDE_DIRS}
)

target_link_directories(slam_gateway_core PUBLIC ${SDK_LIBRARY_DIRS})

if(UNITREE_SDK2_LIB)
    target_link_libraries(slam_gateway_core PUBLIC ${UNITREE_SDK2_LIB})
else()
    target_link_libraries(slam_gateway_core PUBLIC unitree_sdk2)
endif()

if(DDSC_LIB)
    target_link_libraries(slam_gateway_core PUBLIC ${DDSC_LIB})
else()
    target_link_libraries(slam_gateway_core PUBLIC ddsc)
endif()

if(DDSCXX_LIB)
    target_link_libraries(slam_gateway_core PUBLIC ${DDSCXX_LIB})
else()
    target_link_libraries(slam_gateway_core PUBLIC ddscxx)
endif()

target_link_libraries(slam_gateway_core PUBLIC pthread)

add_executable(slam_keyboard_client src/keyboard_main.cpp)
target_link_libraries(slam_keyboard_client PRIVATE slam_gateway_core)

add_executable(slam_llm_command_client src/llm_command_main.cpp)
target_link_libraries(slam_llm_command_client PRIVATE slam_gateway_core)

install(TARGETS slam_gateway_core slam_keyboard_client slam_llm_command_client
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
