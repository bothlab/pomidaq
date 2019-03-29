
set(LIBMINISCOPE_SRC
    miniscope.cpp
    videowriter.cpp
)

set(LIBMINISCOPE_PRIV_HEADERS
    definitions.h
    videowriter.h
)

set(LIBMINISCOPE_HEADERS
    miniscope.h
)

add_library(miniscope SHARED
    ${LIBMINISCOPE_SRC}
    ${LIBMINISCOPE_HEADERS}
    ${LIBMINISCOPE_PRIV_HEADERS}
)
set_target_properties(miniscope PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(miniscope PROPERTIES SOVERSION ${LIBSOVERSION})

target_include_directories(miniscope PRIVATE .)
set_target_properties(miniscope PROPERTIES PUBLIC_HEADER ${LIBMINISCOPE_HEADERS})

target_link_libraries(miniscope
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
    ${OPENGL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${FFMPEG_LIBRARIES}
)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${FFMPEG_INCLUDE_DIRS}
)

install(TARGETS miniscope
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
