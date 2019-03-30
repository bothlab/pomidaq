
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

add_executable(pomidaq
    ${POMIDAQ_SRC}
    ${POMIDAQ_UI_SRC}
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
