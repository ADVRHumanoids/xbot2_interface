
include(${CMAKE_SOURCE_DIR}/cmake/cmrc.cmake)

cmrc_add_resource_library(example_resources
    resources/mobile_manipulator_3dof.urdf)

add_executable(print_model_example cpp/print_model.cpp)

target_link_libraries(print_model_example
    example_resources
    xbot2_interface::xbot2_interface
    fmt::fmt-header-only
)
