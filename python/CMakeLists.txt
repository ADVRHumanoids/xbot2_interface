
find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 QUIET)

option(COMPILE_PYTHON_BINDINGS "Compile python bindings (requires pybind11)" TRUE)

if(${pybind11_FOUND} AND ${COMPILE_PYTHON_BINDINGS})

    message(STATUS "Will compile python bindings")

    # get python install path
    execute_process(
        COMMAND python3 -c "import site, os; print(os.path.relpath(site.USER_SITE, site.USER_BASE))"
        OUTPUT_VARIABLE PYTHON_SITE
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    message(STATUS "Python install dir: ${PYTHON_SITE}")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    pybind11_add_module(pyxbot2_interface pyxbot2_interface.cpp)

    target_link_libraries(pyxbot2_interface PUBLIC xbot2_interface)

    install(TARGETS pyxbot2_interface
        DESTINATION ${PYTHON_SITE}/xbot2_interface)

    file(WRITE ${CMAKE_BINARY_DIR}/__init__.py "")

    install(FILES  ${CMAKE_BINARY_DIR}/__init__.py DESTINATION ${PYTHON_SITE}/xbot2_interface)

endif()
