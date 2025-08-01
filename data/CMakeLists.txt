# Tracking geometry
INSTALL(FILES MuColl_v1.root MuSIC_v1.root MuSIC_v2.root MAIA_v0.root
  DESTINATION ${CMAKE_INSTALL_DATADIR}/ACTSTracking/data)

# Geometry description
INSTALL(FILES MuColl_v1.json MuSIC_v1.json MuSIC_v2.json MAIA_v0.json
  DESTINATION ${CMAKE_INSTALL_DATADIR}/ACTSTracking/data)

# Material map
INSTALL(FILES material-maps.json MAIA_v0_material.json
  DESTINATION ${CMAKE_INSTALL_DATADIR}/ACTSTracking/data)
