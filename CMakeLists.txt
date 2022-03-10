cmake_minimum_required(VERSION 3.5)

project(xbot2_interface LANGUAGES CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(include/
    ${srdfdom_INCLUDE_DIRS})

add_library(xbot2_interface
    src/state_interface.cpp
    src/xbotinterface2.cpp
    src/chain.cpp
    src/joint.cpp)

target_link_libraries(xbot2_interface
    PUBLIC
    pinocchio::pinocchio
    ${urdf_LIBRARIES}
    ${srdfdom_LIBRARIES}
    Eigen3::Eigen)

add_library(xbot2_model_pin
    src/modelinterface2_pin.cpp)

target_link_libraries(xbot2_model_pin
    PUBLIC
    pinocchio::pinocchio
    xbot2_interface)


add_executable(test_main main.cpp)
target_link_libraries(test_main xbot2_interface xbot2_model_pin)