
find_package(hpp-fcl REQUIRED)
find_package(geometric_shapes REQUIRED)

add_library(collision SHARED
    collision.cpp
)

add_library(xbot2_interface::collision ALIAS collision)

target_link_libraries(collision
    PRIVATE
    fmt::fmt-header-only
    hpp-fcl::hpp-fcl
    ${geometric_shapes_LIBRARIES}
    -Wl,--exclude-libs,ALL
    PUBLIC
    xbot2_interface)

if(TARGET geometric_shapes::geometric_shapes)
    target_link_libraries(collision PRIVATE geometric_shapes::geometric_shapes)
endif()


target_compile_options(collision
    PUBLIC
    PRIVATE
    -fvisibility-inlines-hidden
    -fvisibility=hidden)

target_compile_definitions(collision
    PUBLIC
    -DXBOT2IFC_COLLISION_SUPPORT
    PRIVATE
    -DXBOT2IFC_DLL
    -DXBOT2IFC_DLL_EXPORTS)

target_include_directories(collision
    SYSTEM PUBLIC
    ${geometric_shapes_INCLUDE_DIRS}
    )

set_target_properties(collision PROPERTIES
    OUTPUT_NAME xbot2_interface_collision
    SOVERSION ${xbot2_interface_VERSION_MAJOR}.${xbot2_interface_VERSION_MINOR})

install(
    TARGETS collision
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION lib
)
