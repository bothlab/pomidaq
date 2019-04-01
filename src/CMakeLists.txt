
set(POMIDAQ_SRC
    main.cpp
    mainwindow.h
    mainwindow.cpp
    videoviewwidget.h
    videoviewwidget.cpp
)

set(POMIDAQ_UI
    mainwindow.ui
)

qt5_wrap_ui(POMIDAQ_UI_SRC ${POMIDAQ_UI})

qt5_add_resources(POMIDAQ_RES_SRC ../data/resources.qrc)

add_executable(pomidaq
    ${POMIDAQ_SRC}
    ${POMIDAQ_UI_SRC}
    ${POMIDAQ_RES_SRC}
)

target_link_libraries(pomidaq
    ${CMAKE_DL_LIBS}
    Qt5::Widgets
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
