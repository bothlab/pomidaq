
if (GUI)

  # desktop-entry file
  install(FILES pomidaq.desktop DESTINATION "${CMAKE_INSTALL_DATADIR}/applications")

  # icon
  install(FILES pomidaq.svg DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps/")

  # AppStream metainfo data
  set(SRC_METAINFO_FILENAME "io.github.bothlab.pomidaq.metainfo.xml")
  find_program(ASCLI_EXE appstreamcli)
  if (ASCLI_EXE)
    # add release info and install metadata
    set(REL_METAINFO_FILE "${CMAKE_CURRENT_BINARY_DIR}/${SRC_METAINFO_FILENAME}")
    add_custom_command(OUTPUT ${REL_METAINFO_FILE}
      COMMAND ${ASCLI_EXE}
              news-to-metainfo --limit=4 --format=text
              "${CMAKE_SOURCE_DIR}/NEWS"
              ${SRC_METAINFO_FILENAME}
              ${REL_METAINFO_FILE}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${SRC_METAINFO_FILENAME}"
      COMMENT "Adding release info to metainfo file"
      VERBATIM
    )
    add_custom_target(metainfo_add_release ALL DEPENDS ${REL_METAINFO_FILE})
    install(FILES ${REL_METAINFO_FILE} DESTINATION "${CMAKE_INSTALL_DATADIR}/metainfo")

    # validate our metadata
    add_test(NAME metainfo-validate
      COMMAND ${ASCLI_EXE} validate --pedantic --no-net ${REL_METAINFO_FILE}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
  else()
     # if we have no appstreamcli, we just install the unprocessed metainfo file
     message("appstreamcli not found, metadata may be incomplete.")
     install(FILES ${SRC_METAINFO_FILENAME} DESTINATION "${CMAKE_INSTALL_DATADIR}/metainfo")
  endif()

endif()
