find_package(pinocchio REQUIRED)

add_library(modelinterface2_pin SHARED
    modelinterface2_pin.cpp)

target_link_libraries(modelinterface2_pin
    PUBLIC
    pinocchio::pinocchio
    xbot2_interface)
