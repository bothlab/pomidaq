# CMakeLists for PoMiDAQ Python module

find_package(PythonInterp 3 REQUIRED)
find_package(Boost COMPONENTS python3)
find_package(PythonLibs 3 REQUIRED)
find_package(NumPy REQUIRED)

python_add_module(pyminiscope
    pyminiscope.cpp
    cvmatndsliceconvert.h
    cvmatndsliceconvert.cpp
)

target_link_libraries(pyminiscope
    ${PYTHON_LIBRARIES}
    ${Boost_LIBRARIES}
    miniscope
)
set_target_properties(pyminiscope PROPERTIES OUTPUT_NAME miniscope)

install(TARGETS pyminiscope
    LIBRARY DESTINATION "lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages/"
    COMPONENT python
)

include_directories(SYSTEM
    ${PYTHON_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${NUMPY_INCLUDE_DIRS}
)
include_directories(
    ../libminiscope/
)
