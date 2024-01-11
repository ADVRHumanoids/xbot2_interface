
include(${CMAKE_SOURCE_DIR}/cmake/cmrc.cmake)

cmrc_add_resource_library(example_resources
    resources/mobile_manipulator_3dof.urdf)

set(examples_dependencies
    example_resources
    xbot2_interface::xbot2_interface
    fmt::fmt-header-only
)

#
add_executable(print_model_example cpp/print_model.cpp)

target_link_libraries(print_model_example
    ${examples_dependencies}
)

#
add_executable(ik_example cpp/ik.cpp)

target_link_libraries(ik_example
    ${examples_dependencies}
)

#
add_executable(reduced_model_example cpp/reduced_model.cpp)

target_link_libraries(reduced_model_example
    ${examples_dependencies}
)

#
add_executable(payload_link_example cpp/payload_link.cpp)

target_link_libraries(payload_link_example
    ${examples_dependencies}
)

#
install(
    TARGETS
    print_model_example
    ik_example
    reduced_model_example
    payload_link_example
    DESTINATION
    share/${PROJECT_NAME}/examples/cpp)
