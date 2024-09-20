cmake_minimum_required(VERSION 3.10)
project(centauro_cartesio VERSION 1.0.0)

# Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(OpenSoT REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(cartesio_acceleration_support QUIET)
find_package(xbot2 QUIET)

# Generate services in the 'srv' folder
set(srv_files
  srv/SetNormal.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    DEPENDENCIES
    builtin_interfaces
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenSoT_INCLUDE_DIRS}
)

# centauro_cartesio_addon library
add_library(centauro_cartesio_addon
    src/cartesio_addon.cpp
    src/centauro_ankle_steering.cpp
)

target_link_libraries(centauro_cartesio_addon
    cartesian_interface::CartesianInterface
    cartesian_interface::CartesianInterfaceSolverOpenSot
    rclcpp::rclcpp
    ${PROJECT_NAME}__rosidl_typesupport_cpp
)

install(TARGETS centauro_cartesio_addon
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# omnisteering_controller library
add_library(omnisteering_controller SHARED
    src/omnisteering_controller.cpp)

target_link_libraries(omnisteering_controller
    cartesian_interface::CartesianInterface
    centauro_cartesio_addon
)

install(TARGETS omnisteering_controller
    DESTINATION lib)

# # omnisteering_controller ndoe
# add_executable(omnisteering_controller_node
#     src/omnisteering_controller_node.cpp)

# target_link_libraries(omnisteering_controller_node
#     omnisteering_controller
# )

# install(TARGETS omnisteering_controller omnisteering_controller_node
#     EXPORT ${PROJECT_NAME}Targets
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

# )

# omnisteering_controller plugin
if(${xbot2_FOUND})
    add_xbot2_plugin(omnisteering_controller_plugin
        src/omnisteering_controller_plugin.cpp)

    target_link_libraries(omnisteering_controller_plugin
        PRIVATE
        omnisteering_controller
        xbot2::xbot2_ros2_support)

    install(TARGETS omnisteering_controller_plugin
        DESTINATION lib
    )
endif()

#if(${cartesio_acceleration_support_FOUND} AND ${xbot2_FOUND})

#    add_xbot2_plugin(centauro_gcomp_plugin
#        src/centauro_gcomp.cpp)

#    target_include_directories(centauro_gcomp_plugin PRIVATE
#        ${cartesio_acceleration_support_INCLUDE_DIRS}
#    )

#    target_link_libraries(centauro_gcomp_plugin
#        PRIVATE
#        cartesian_interface::CartesianInterface
#        xbot2::xbot2_ros_support)

#    install(TARGETS centauro_gcomp_plugin
#        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    )
#endif()


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/centauro_cartesio_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

install(DIRECTORY configs
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.launch"
    PATTERN ".svn" EXCLUDE
)

# bindings
add_subdirectory(python)


include(cmake/ExportProject.cmake)
# export_project()
