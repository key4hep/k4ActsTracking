########################################################
# cmake file for building Gaudi ACTSTracking Package
# @author Samuel Ferraro
########################################################

### DATA ###########################################################
ADD_SUBDIRECTORY(data)
ADD_SUBDIRECTORY(example)

### LIBRARY #################################################################

# sources
CONFIGURE_FILE(src/config.h.in config.h)
FILE( GLOB _plugin_sources src/*.cxx )
gaudi_add_module(ACTSTrackingPlugins
		SOURCES ${_plugin_sources}
		LINK	${ROOT_LIBRARIES}
			k4FWCore::k4FWCore
			DD4hep::DDCore
			Gaudi::GaudiKernel
			EDM4HEP::edm4hep
			ActsCore ActsPluginTGeo ActsPluginJson)

INSTALL( TARGETS ACTSTrackingPlugins 
	EXPORT ${CMAKE_PROJECT_NAME}Targets
	RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin 
	LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
	PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/@{CMAKE_PROJECT_NAME}"
	COMPONENT dev )

TARGET_INCLUDE_DIRECTORIES(ACTSTrackingPlugins
		PRIVATE	./ACTSTracking
			${CMAKE_CURRENT_BINARY_DIR}
    			${Gaudi_INCLUDE_DIRS}
    			${ROOT_INCLUDE_DIRS})

SET( GAUDI_GENCONF_DIR "genConfDir" )
if (${Gaudi_VERSION} VERSION_LESS 35.1)
	SET( GAUDI_GENCONF_DIR "genConf" )
endif()


