find_package(RBDL REQUIRED)

include_directories(${RBDL_INCLUDE_DIR})

add_library(modelinterface2_rbdl SHARED
    modelinterface2_rbdl.cpp)

target_link_libraries(modelinterface2_rbdl
    PUBLIC
    ${RBDL_LIBRARY}
    ${RBDL_URDFREADER_LIBRARY}
    xbot2_interface)
