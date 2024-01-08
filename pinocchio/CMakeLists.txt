find_package(pinocchio REQUIRED)

add_library(modelinterface2_pin SHARED
    modelinterface2_pin.cpp
    modelinterface2_pin_aba.cpp
    modelinterface2_pin_crba.cpp
    modelinterface2_pin_rnea.cpp
    modelinterface2_pin_ccrba.cpp)

target_link_libraries(modelinterface2_pin
    PUBLIC
    pinocchio::pinocchio
    xbot2_interface)

target_compile_options(modelinterface2_pin
    PUBLIC
    PRIVATE
    -fvisibility-inlines-hidden
    -fvisibility=hidden)

install(
    TARGETS modelinterface2_pin
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION lib
)
