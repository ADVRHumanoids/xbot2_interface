find_package(GTest)
find_package(Threads)

## compile GTest
if(NOT GTEST_FOUND)
    include(ExternalProject)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
        PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
        INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
    set(GTEST_DEPENDS GTest-ext)
endif()

include_directories(
    ${GTEST_INCLUDE_DIRS}
    )

link_directories(${GTEST_LIB_DIRS})

add_definitions(-DXBOT2_TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources/")

## utils

# common libs to link against
set(TestLibs
    ${GTEST_BOTH_LIBRARIES}
    pthread
    xbot2_interface)

# function to ease adding tests with correct options
function(compile_test_executable name)
    add_executable(${name} src/${name}.cpp)
    if(DEFINED GTEST_DEPENDS)
        add_dependencies(${name} ${GTEST_DEPENDS})
    endif()
    target_link_libraries(${name} PRIVATE ${TestLibs})
endfunction()

function(add_test_executable name)
    compile_test_executable(${name})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

## test suite
add_test_executable(test_parser)

add_test_executable(test_parametrization)

add_test_executable(test_kinematics)

add_test_executable(test_memory)
target_link_libraries(test_memory PRIVATE xbot2_interface::collision)

add_test_executable(test_collision)
target_link_libraries(test_collision PRIVATE xbot2_interface::collision)

add_executable(eigen_ref_calls_free src/eigen_ref_calls_free.cpp)
target_link_libraries(eigen_ref_calls_free Eigen3::Eigen dl)
