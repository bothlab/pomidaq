# CMakeLists for PoMiDAQ Python module

find_package(PythonInterp 3 REQUIRED)
find_package(Boost COMPONENTS python3)
find_package(PythonLibs 3 REQUIRED)

python_add_module(pyminiscope pyminiscope.cpp)

target_link_libraries(pyminiscope
    ${PYTHON_LIBRARIES}
    ${Boost_LIBRARIES}
    miniscope
)
set_target_properties(pyminiscope PROPERTIES OUTPUT_NAME miniscope)

include_directories(SYSTEM
    ${PYTHON_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
include_directories(
    ../libminiscope/
)
