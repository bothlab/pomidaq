# CMakeLists for PoMiDAQ GUI executable

if(WIN32)
  set(GUI_TYPE WIN32)
elseif(APPLE)
  set(GUI_TYPE MACOSX_BUNDLE)
endif()

set(POMIDAQ_SRC
    main.cpp
    mainwindow.h
    mainwindow.cpp
    mscontrolwidget.h
    mscontrolwidget.cpp
    imageviewwidget.h
    imageviewwidget.cpp
    elidedlabel.h
    elidedlabel.cpp
)

set(POMIDAQ_UI
    mainwindow.ui
)

qt6_wrap_ui(POMIDAQ_UI_SRC ${POMIDAQ_UI})

qt6_add_resources(POMIDAQ_RES_SRC ../data/resources.qrc)

if(WIN32)
    # Convert the SVG icon to a multi-resolution ICO file for the Windows executable.
    find_program(RSVG_CONVERT rsvg-convert REQUIRED)
    find_program(ICOTOOL icotool REQUIRED)

    set(_svg_src "${CMAKE_SOURCE_DIR}/data/pomidaq.svg")
    set(_ico_out "${CMAKE_CURRENT_BINARY_DIR}/pomidaq.ico")

    foreach(_sz 16 32 48 256)
        list(APPEND _png_files "${CMAKE_CURRENT_BINARY_DIR}/icon_${_sz}.png")
        add_custom_command(
            OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/icon_${_sz}.png"
            COMMAND ${RSVG_CONVERT} -w ${_sz} -h ${_sz} -o "${CMAKE_CURRENT_BINARY_DIR}/icon_${_sz}.png" "${_svg_src}"
            DEPENDS "${_svg_src}"
            COMMENT "Rendering icon at ${_sz}×${_sz}"
        )
    endforeach()

    add_custom_command(
        OUTPUT "${_ico_out}"
        COMMAND ${ICOTOOL} -c -o "${_ico_out}" ${_png_files}
        DEPENDS ${_png_files}
        COMMENT "Creating pomidaq.ico"
    )

    # Generate the .rc file at configure time, pointing at the build-dir ICO.
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/pomidaq.rc"
        "IDI_ICON1 ICON \"${_ico_out}\"\n")

    # Compile the .rc file via a standalone windres invocation so that CMake does
    # not forward compiler flags (e.g. -Wall) that windres cannot accept.
    set(_rc_obj "${CMAKE_CURRENT_BINARY_DIR}/pomidaq.rc.obj")
    add_custom_command(
        OUTPUT "${_rc_obj}"
        COMMAND windres "${CMAKE_CURRENT_BINARY_DIR}/pomidaq.rc" "${_rc_obj}"
        DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/pomidaq.rc" "${_ico_out}"
        COMMENT "Compiling pomidaq.rc"
    )
    set(_rc_sources "${_rc_obj}")
    install(FILES "${_ico_out}" DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/")
else()
    set(_rc_sources "")
endif()

add_executable(pomidaq
    ${GUI_TYPE}
    ${POMIDAQ_SRC}
    ${POMIDAQ_UI_SRC}
    ${POMIDAQ_RES_SRC}
    ${_rc_sources}
)

target_link_libraries(pomidaq
    ${CMAKE_DL_LIBS}
    Qt6::Widgets
    Qt6::OpenGLWidgets
    Qt6::Svg
    ${KF6_ConfigWidgets}
    ${OpenCV_LIBS}
    ${OPENGL_LIBRARIES}
    miniscope
)

include_directories(SYSTEM
    ${OpenCV_INCLUDE_DIRS}
)
include_directories(
    ../libminiscope/
)

install(TARGETS pomidaq DESTINATION ${CMAKE_INSTALL_BINDIR})

# More Windows stuff
if(MSVC)
    set_target_properties(pomidaq PROPERTIES LINK_FLAGS_DEBUG "/SUBSYSTEM:CONSOLE")
    set_target_properties(pomidaq PROPERTIES COMPILE_DEFINITIONS_DEBUG "_CONSOLE")
    set_target_properties(pomidaq PROPERTIES LINK_FLAGS_RELWITHDEBINFO "/SUBSYSTEM:windows")
    set_target_properties(pomidaq PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:windows")
    set_target_properties(pomidaq PROPERTIES LINK_FLAGS_MINSIZEREL "/SUBSYSTEM:windows")
endif()
