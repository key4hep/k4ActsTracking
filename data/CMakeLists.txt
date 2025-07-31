# Tracking geometry
INSTALL(FILES MuColl_v1.root
  DESTINATION ${CMAKE_INSTALL_DATADIR}/ACTSTracking/data)

# Material map
INSTALL(FILES material-maps.json
  DESTINATION ${CMAKE_INSTALL_DATADIR}/ACTSTracking/data)
