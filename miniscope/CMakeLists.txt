
set(LIBMINISCOPE_SRC
    miniscope.cpp
    mediatypes.cpp
    videowriter.cpp
    csvwriter.cpp
    zstackcapture.cpp
    asynctask.cpp
    logging.cpp
)

set(LIBMINISCOPE_PRIV_HEADERS
    scopeintf.h
    loginternal.h
    resources.h
    videowriter.h
    csvwriter.h
    zstackcapture.h
    asynctask-private.h
)

set(LIBMINISCOPE_HEADERS
    asynctask.h
    miniscope.h
    mediatypes.h
    logging.h
)

#
# Embedded resources
#
set(LIBMINISCOPE_RES_INPUTS
    ${CMAKE_CURRENT_SOURCE_DIR}/miniscopes.json
    ${CMAKE_SOURCE_DIR}/data/png/orientation-indicator-good.png
    ${CMAKE_SOURCE_DIR}/data/png/orientation-indicator-bad.png
)
set(LIBMINISCOPE_RES_SYMBOLS
    miniscopes_json
    orientation_indicator_good_png
    orientation_indicator_bad_png
)
set(LIBMINISCOPE_RES_SRC ${CMAKE_CURRENT_BINARY_DIR}/mscope_resources.cpp)
add_custom_command(
    OUTPUT ${LIBMINISCOPE_RES_SRC}
    COMMAND ${CMAKE_COMMAND}
            "-DOUTPUT=${LIBMINISCOPE_RES_SRC}"
            "-DNAMESPACE=Miniscope::Res"
            "-DINPUTS=${LIBMINISCOPE_RES_INPUTS}"
            "-DSYMBOLS=${LIBMINISCOPE_RES_SYMBOLS}"
            -P ${CMAKE_SOURCE_DIR}/contrib/cmake/EmbedResources.cmake
    DEPENDS ${LIBMINISCOPE_RES_INPUTS} ${CMAKE_SOURCE_DIR}/contrib/cmake/EmbedResources.cmake
    COMMENT "Embedding libminiscope resources"
    VERBATIM
)

add_library(miniscope
    ${LIBMINISCOPE_SRC}
    ${LIBMINISCOPE_HEADERS}
    ${LIBMINISCOPE_PRIV_HEADERS}
    ${LIBMINISCOPE_RES_SRC}
)
set_target_properties(miniscope PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(miniscope PROPERTIES SOVERSION ${LIBSOVERSION})

target_include_directories(miniscope PRIVATE .)
set_target_properties(miniscope PROPERTIES PUBLIC_HEADER "${LIBMINISCOPE_HEADERS}")
set_target_properties(miniscope PROPERTIES CXX_VISIBILITY_PRESET hidden)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/miniscope.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/miniscope.pc @ONLY
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/miniscope.pc DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

target_link_libraries(miniscope
    PUBLIC
        ${OpenCV_LIBS}
    PRIVATE
        Threads::Threads
        nlohmann_json::nlohmann_json
        ${FFMPEG_LIBRARIES}
)

include_directories(SYSTEM
    ${OpenCV_INCLUDE_DIRS}
    ${FFMPEG_INCLUDE_DIRS}
)

install(TARGETS miniscope
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT library
)
