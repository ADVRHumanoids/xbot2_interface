cmake_minimum_required(VERSION 3.5)

project(xbot2_interface LANGUAGES CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INSTALL_RPATH $ORIGIN)
set(CMAKE_BUILD_RPATH "$ORIGIN;$ORIGIN/pinocchio;$ORIGIN/rbdl;$ORIGIN/ros")

find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include/
    ${srdfdom_INCLUDE_DIRS})

add_library(xbot2_interface SHARED
    src/state_interface.cpp
    src/xbotinterface2.cpp
    src/robotinterface2.cpp
    src/chain.cpp
    src/joint.cpp
    src/types.cpp
    src/utils.cpp)

target_link_libraries(xbot2_interface
    PUBLIC
    ${urdf_LIBRARIES}
    ${srdfdom_LIBRARIES}
    Eigen3::Eigen)



find_package(pinocchio QUIET)
if(${pinocchio_FOUND})
    add_subdirectory(pinocchio/)
endif()

find_package(RBDL QUIET)
if(${RBDL_FOUND})
    add_subdirectory(rbdl/)
endif()

find_package(roscpp QUIET)
if(googletest${roscpp_FOUND})
    add_subdirectory(ros/)
endif()

add_executable(test_main main.cpp)
target_link_libraries(test_main xbot2_interface)

add_subdirectory(test)
