
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(xbot_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 20)

include_directories(
    ${roscpp_INCLUDE_DIRS}
    ${xbot_msgs_INCLUDE_DIRS}
    )

add_library(robotinterface2_ros2 SHARED
    robotinterface2_ros.cpp)

target_link_libraries(robotinterface2_ros2
    PUBLIC
    rclcpp::rclcpp
    tf2_eigen::tf2_eigen
    tf2_geometry_msgs::tf2_geometry_msgs
    xbot_msgs::xbot_msgs__rosidl_typesupport_cpp
    xbot2_interface)

target_compile_options(robotinterface2_ros2
    PUBLIC
    PRIVATE
    -fvisibility-inlines-hidden
    -fvisibility=hidden)

install(
    TARGETS
    robotinterface2_ros2
    DESTINATION lib)

install(
    FILES
    config_from_param.hpp
    DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}/ros2
)
