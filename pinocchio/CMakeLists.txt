find_package(pinocchio REQUIRED)

add_library(modelinterface2_pin SHARED
    modelinterface2_pin.cpp
    modelinterface2_pin_aba.cpp
    modelinterface2_pin_crba.cpp
    modelinterface2_pin_rnea.cpp)

target_link_libraries(modelinterface2_pin
    PUBLIC
    pinocchio::pinocchio
    xbot2_interface)

install(
    TARGETS modelinterface2_pin
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION lib
)
