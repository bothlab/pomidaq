
if (GUI)
  # .desktop file
  install(FILES pomidaq.desktop DESTINATION "${CMAKE_INSTALL_DATADIR}/applications")

  # AppStream metainfo file
  install(FILES io.github.bothlab.pomidaq.metainfo.xml DESTINATION "${CMAKE_INSTALL_DATADIR}/metainfo")

  # icon
  install(FILES pomidaq.svg DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps/")
endif()
