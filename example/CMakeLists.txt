# Loop over all available xml files
FILE(GLOB xmls [a-zA-Z\-]*.xml)

FOREACH(xml ${xmls})
  GET_FILENAME_COMPONENT(xml ${xml} NAME)
  INSTALL(FILES ${xml}
    DESTINATION ${CMAKE_INSTALL_DATADIR}/ACTSTracking/example)
ENDFOREACH()
