cmake_minimum_required(VERSION 3.5)

project(xbot2_interface LANGUAGES CXX VERSION 1.0.0)

option(XBOT2_IFC_BUILD_PINOCCHIO "Build Pinocchio implementation" ON)
option(XBOT2_IFC_BUILD_RBDL "Build RBDL implementation" OFF)
option(XBOT2_IFC_BUILD_ROS "Build ROS implementation" ON)
option(XBOT2_IFC_BUILD_COLLISION "Build collision support (required hpp-fcl)" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INSTALL_RPATH $ORIGIN)
set(CMAKE_BUILD_RPATH "$ORIGIN;$ORIGIN/pinocchio;$ORIGIN/rbdl;$ORIGIN/ros")

find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(Eigen3 REQUIRED)

include(GNUInstallDirs)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

# internal fmt
add_subdirectory(external/fmt-10.2.1 EXCLUDE_FROM_ALL)

add_library(xbot2_interface SHARED
    src/state_interface.cpp
    src/xbotinterface2.cpp
    src/robotinterface2.cpp
    src/chain.cpp
    src/joint.cpp
    src/types.cpp
    src/utils.cpp
    src/sensor.cpp
    src/imu.cpp
    src/force_torque.cpp
    src/logger.cpp
)

add_library(xbot2_interface::xbot2_interface ALIAS xbot2_interface)

target_link_libraries(xbot2_interface
    PRIVATE
    fmt::fmt-header-only
    PUBLIC
    ${urdf_LIBRARIES}
    ${srdfdom_LIBRARIES}
    Eigen3::Eigen)

target_compile_options(xbot2_interface
    PUBLIC
    PRIVATE
    -fvisibility-inlines-hidden
    -fvisibility=hidden)

target_compile_definitions(xbot2_interface
    PUBLIC
    PRIVATE
    -DXBOT2IFC_DLL
    -DXBOT2IFC_DLL_EXPORTS)

target_include_directories(xbot2_interface
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
    )

target_include_directories(xbot2_interface
    SYSTEM PUBLIC
    ${srdfdom_INCLUDE_DIRS}
    )

set_target_properties(xbot2_interface PROPERTIES
    SOVERSION ${xbot2_interface_VERSION_MAJOR}.${xbot2_interface_VERSION_MINOR})

install(
    TARGETS xbot2_interface
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION lib
)

install(DIRECTORY include/xbot2_interface
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/xbot2_interface
    )


if(${XBOT2_IFC_BUILD_PINOCCHIO})
    add_subdirectory(pinocchio/)
endif()

if(${XBOT2_IFC_BUILD_RBDL})
    add_subdirectory(rbdl/)
endif()

if(${XBOT2_IFC_BUILD_ROS})
    add_subdirectory(ros/)
endif()

if(${XBOT2_IFC_BUILD_COLLISION})
    add_subdirectory(src/collision/)
endif()

add_subdirectory(python)

add_subdirectory(examples)

enable_testing()
add_subdirectory(test)

include(cmake/ExportProject.cmake)
export_project()
