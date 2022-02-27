cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    roscpp
    tf
    geometry_msgs
    unitree_legged_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
)

# include_directories(
#     include
#     ${Boost_INCLUDE_DIR}
#     ${catkin_INCLUDE_DIRS}
#     ${EIGEN3_INCLUDE_DIR}
#     $ENV{UNITREE_LEGGED_SDK_PATH}/include
#     # $ENV{ALIENGO_SDK_PATH}/include
#     # /home/unitree/aliengo_sdk/include
# )

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    # ${EIGEN3_INCLUDE_DIR}
    "/usr/include/eigen3"
    "/usr/local/include/eigen3"
)
include_directories(/home/unitree/Downloads/unitree_legged_sdk/include)
link_directories(/home/unitree/Downloads/unitree_legged_sdk/lib)

# message("UNITREE_LEGGED_SDK_PATH: $ENV{UNITREE_LEGGED_SDK_PATH}")
# message("ALIENGO_SDK_PATH: $ENV{ALIENGO_SDK_PATH}")
# link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
# link_directories(/home/unitree/aliengo_sdk/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_arm64.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

set(CMAKE_CXX_FLAGS "-O3")

add_executable(ros_control src/exe/ros_control.cpp)
target_link_libraries(ros_control ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_control ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lcm_server_3_2 /home/unitree/Downloads/unitree_legged_sdk/examples/lcm_server.cpp)
target_link_libraries(lcm_server_3_2 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(lcm_server_3_2 ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(lcm_server_3_1 $ENV{ALIENGO_SDK_PATH}/examples/lcm_server.cpp)
# target_link_libraries(lcm_server_3_1 ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(lcm_server_3_1 ${${PEOJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
