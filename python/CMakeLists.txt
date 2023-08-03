find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

add_compile_options(-std=c++14)

if(${pybind11_FOUND})

    message(STATUS "Will compile python bindings")

    # get python install path
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "if True:
            from distutils import sysconfig as sc
            print(sc.get_python_lib(prefix='', plat_specific=True))"
        OUTPUT_VARIABLE PYTHON_SITE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    message(STATUS "Python install dir: ${PYTHON_SITE}")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    pybind11_add_module(simple_steering simple_steering.cpp)
    target_link_libraries(simple_steering PRIVATE centauro_cartesio_addon omnisteering_controller)
    install(TARGETS simple_steering
            DESTINATION ${PYTHON_SITE}/centauro_cartesio)

    file(WRITE ${CMAKE_BINARY_DIR}/__init__.py "")
    install(FILES  ${CMAKE_BINARY_DIR}/__init__.py DESTINATION ${PYTHON_SITE}/centauro_cartesio)


else()
    message(STATUS "Pybind11 bindings won't be available")
endif()
