cmake_minimum_required(VERSION 2.8.3)
project(prm_localization)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(${PROJECT_NAME}_CATKIN_DEPS
        roscpp std_msgs rospy nodelet sensor_msgs pcl_ros pcl_msgs ndt_omp nav_msgs geometry_msgs tf tf_conversions message_generation message_filters
        )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        message_generation ${${PROJECT_NAME}_CATKIN_DEPS})


find_package(PCL REQUIRED)
include_directories(
        include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})


find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

add_message_files(FILES
        DriveInfo.msg
        )

add_service_files(
        FILES
        save_data.srv
        locali.srv
)

generate_messages(DEPENDENCIES
        std_msgs
        geometry_msgs
        )
###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES globalmap_provider_nodelet rt_localization_nodelet
        CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
        #  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(EIGEN_INCLUDE_PATH "/usr/include/eigen3")
include_directories(
        include
        ${EIGEN_INCLUDE_PATH}
        ${catkin_INCLUDE_DIRS}

)

add_executable(relocalizaer
        src/relocalizaer.cpp
        )
add_dependencies(relocalizaer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(relocalizaer
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
# loca_odom
# add_executable(fused_odom
#        src/fused_odometry.cpp
#        )
#add_dependencies(fused_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(fused_odom
#        ${catkin_LIBRARIES}
#        ${PCL_LIBRARIES}
#        )

#nodelets
add_library(globalmap_provider_nodelet src/globalmap_provider_nodelet.cpp)
target_link_libraries(globalmap_provider_nodelet
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_library(rt_localization_nodelet src/rt_localization_nodelet.cpp)
target_link_libraries(rt_localization_nodelet
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

#install library http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
#install(TARGETS rt_localization_nodelet globalmap_provider_nodelet
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
#
#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
#
#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        PATTERN ".svn" EXCLUDE)
#
##install launch
#install(DIRECTORY launch
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        )
#node
add_executable(blank
        src/blank.cpp
        )
add_dependencies(blank ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(blank
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

