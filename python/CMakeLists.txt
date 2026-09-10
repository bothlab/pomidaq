# CMakeLists for PoMiDAQ Python module

find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module NumPy)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 2.12 CONFIG REQUIRED)

pybind11_add_module(pyminiscope
    pyminiscope.cpp
    cvmatndsliceconvert.h
    cvmatndsliceconvert.cpp
)

target_link_libraries(pyminiscope PRIVATE
    miniscope
    Python::NumPy
)
target_include_directories(pyminiscope PRIVATE ${CMAKE_SOURCE_DIR})
set_target_properties(pyminiscope PROPERTIES OUTPUT_NAME miniscope)

if (SKBUILD)
    # we are building a wheel with scikit-build-core, the module
    # goes into the root of the wheel's platlib directory
    set(PYMINISCOPE_INSTALL_DIR "${SKBUILD_PLATLIB_DIR}")
else()
    set(PYMINISCOPE_INSTALL_DIR "lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/dist-packages")
endif()

install(TARGETS pyminiscope
    LIBRARY DESTINATION "${PYMINISCOPE_INSTALL_DIR}"
    COMPONENT python
)
