find_package(roscpp REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(eigen_conversions REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(
    ${roscpp_INCLUDE_DIRS}
    ${xbot_msgs_INCLUDE_DIRS}
    )

add_library(robotinterface2_ros SHARED
    robotinterface2_ros.cpp)

target_link_libraries(robotinterface2_ros
    PUBLIC
    ${roscpp_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    xbot2_interface)

target_compile_options(robotinterface2_ros
    PUBLIC
    PRIVATE
    -fvisibility-inlines-hidden
    -fvisibility=hidden)

install(
    TARGETS
    robotinterface2_ros
    DESTINATION lib)

install(
    FILES
    config_from_param.hpp
    DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}/ros
)
