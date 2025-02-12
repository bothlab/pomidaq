# CMakeLists for PoMiDAQ Python module

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 REQUIRED)
find_package(NumPy REQUIRED)

pybind11_add_module(pyminiscope
    pyminiscope.cpp
    cvmatndsliceconvert.h
    cvmatndsliceconvert.cpp
    qstringtopy.h
)

target_link_libraries(pyminiscope PRIVATE
    miniscope
)
set_target_properties(pyminiscope PROPERTIES OUTPUT_NAME miniscope)

install(TARGETS pyminiscope
    LIBRARY DESTINATION "lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages/"
)

include_directories(SYSTEM
    ${NUMPY_INCLUDE_DIRS}
)
include_directories(
    ../libminiscope/
)
