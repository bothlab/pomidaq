
set(LIBMINISCOPE_SRC
    miniscope.cpp
    videowriter.cpp
    mediatypes.cpp
)

set(LIBMINISCOPE_PRIV_HEADERS
    scopeintf.h
    videowriter.h
)

set(LIBMINISCOPE_HEADERS
    miniscope.h
    mediatypes.h
)

qt5_add_resources(LIBMINISCOPE_RES_SRC mscoperes.qrc)

add_library(miniscope
    ${LIBMINISCOPE_SRC}
    ${LIBMINISCOPE_HEADERS}
    ${LIBMINISCOPE_PRIV_HEADERS}
    ${LIBMINISCOPE_RES_SRC}
)
set_target_properties(miniscope PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(miniscope PROPERTIES SOVERSION ${LIBSOVERSION})

set(MINISCOPE_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR}/miniscope)
target_include_directories(miniscope PRIVATE .)
set_target_properties(miniscope PROPERTIES PUBLIC_HEADER "${LIBMINISCOPE_HEADERS}")
set_target_properties(miniscope PROPERTIES CXX_VISIBILITY_PRESET hidden)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/miniscope.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/miniscope.pc @ONLY
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/miniscope.pc DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

target_link_libraries(miniscope
    ${CMAKE_THREAD_LIBS_INIT}
    Qt5::Core
    ${OpenCV_LIBS}
    ${OPENGL_LIBRARIES}
    ${FFMPEG_LIBRARIES}
)

include_directories(SYSTEM
    ${OpenCV_INCLUDE_DIRS}
    ${FFMPEG_INCLUDE_DIRS}
)

install(TARGETS miniscope
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${MINISCOPE_INSTALL_INCLUDEDIR}
    COMPONENT library
)
