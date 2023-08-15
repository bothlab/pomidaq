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

qt5_wrap_ui(POMIDAQ_UI_SRC ${POMIDAQ_UI})

qt5_add_resources(POMIDAQ_RES_SRC ../data/resources.qrc)

add_executable(pomidaq
    ${GUI_TYPE}
    ${POMIDAQ_SRC}
    ${POMIDAQ_UI_SRC}
    ${POMIDAQ_RES_SRC}
)

target_link_libraries(pomidaq
    ${CMAKE_DL_LIBS}
    Qt5::Widgets
    Qt5::Svg
    ${KF5_ConfigWidgets}
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

# Windows stuff
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows")
   add_definitions("-D_UNICODE" "-DUNICODE")
   set_target_properties(pomidaq PROPERTIES LINK_FLAGS_DEBUG "/SUBSYSTEM:CONSOLE")
   set_target_properties(pomidaq PROPERTIES COMPILE_DEFINITIONS_DEBUG "_CONSOLE")
   set_target_properties(pomidaq PROPERTIES LINK_FLAGS_RELWITHDEBINFO "/SUBSYSTEM:windows")
   set_target_properties(pomidaq PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:windows")
   set_target_properties(pomidaq PROPERTIES LINK_FLAGS_MINSIZEREL "/SUBSYSTEM:windows")
endif()
