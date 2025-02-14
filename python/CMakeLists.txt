cmake_policy(SET CMP0094 NEW)
find_package(Python3 COMPONENTS Interpreter Development.Module NumPy)
find_package(pybind11 QUIET)

if(${pybind11_FOUND})

    message(STATUS "Found python (${Python3_EXECUTABLE}) will compile python bindings")

    if("${BUILD_BY_PYTHON}")
        set(PYTHON_SITE ".")
    else()
        # get python install path
        execute_process(
            COMMAND ${Python3_EXECUTABLE} -c "import site, os; print(os.path.relpath(site.USER_SITE, site.USER_BASE))"
            OUTPUT_VARIABLE PYTHON_SITE
            OUTPUT_STRIP_TRAILING_WHITESPACE)

        message(STATUS "Python install dir: ${PYTHON_SITE}")
    endif()

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})


    pybind11_add_module(pyxbot2_interface pyxbot2_interface.cpp)

    target_link_libraries(pyxbot2_interface PUBLIC xbot2_interface)

    install(TARGETS pyxbot2_interface
        DESTINATION ${PYTHON_SITE}/xbot2_interface
        COMPONENT python-bindings
        )


    pybind11_add_module(pyaffine3 pyaffine3.cpp)

    target_link_libraries(pyaffine3 PUBLIC Eigen3::Eigen)

    install(TARGETS pyaffine3
        DESTINATION ${PYTHON_SITE}/xbot2_interface
        COMPONENT python-bindings
        )

    if (${XBOT2_IFC_BUILD_COLLISION})
        pybind11_add_module(pyxbot2_collision pyxbot2_collision.cpp)

        target_link_libraries(pyxbot2_collision PUBLIC xbot2_interface::collision)

        install(TARGETS pyxbot2_collision
            DESTINATION ${PYTHON_SITE}/xbot2_interface
            COMPONENT python-bindings
            )
    endif()

    install(FILES __init__.py DESTINATION ${PYTHON_SITE}/xbot2_interface COMPONENT python-bindings)

else()
    if(${BUILD_BY_PYTHON})
        # Fail explicitely as python bindings are needed for building by python
        message(FATAL_ERROR "pybind11 not found - cannot build library")
    endif()
endif()
