#ifndef ACTSAlgBase_h
#define ACTSAlgBase_h 1

// ACTS
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Utilities/CalibrationContext.hpp>

#ifdef K4ACTSTRACKING_ACTS_HAS_TGEO_PLUGIN
#include <Acts/Plugins/TGeo/TGeoDetectorElement.hpp>
#else
#include <Acts/Plugins/Root/TGeoDetectorElement.hpp>
#endif

// edm4hep
#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackerHitPlaneCollection.h>

// Gaudi
#include <Gaudi/Property.h>

// k4FWCore
#include <k4FWCore/DataHandle.h>
#include <k4FWCore/BaseClass.h>
#include <k4FWCore/Transformer.h>
#include <k4Interface/IGeoSvc.h>

// Standard
#include <tuple>
#include <string>

// ACTSTracking
#include "k4ActsTracking/GeometryIdMappingTool.hxx"

//! Base processor for ACTS tracking
/**
 * Performs tasks common to all ACTS processors
 *  - loading tracking geometry
 *
 * Assumes that the global TGeoManager with the geometry
 * description is already loaded. For example, via the
 * InitializeDD4hep processor.
 *
 * @author Karol Krizka
 * @author Samuel Ferraro
 * @version $Id$
 */
struct ACTSAlgBase : k4FWCore::MultiTransformer<std::tuple<
		    edm4hep::TrackCollection, 
		    edm4hep::TrackCollection>(
		    const edm4hep::TrackerHitPlaneCollection &)> {
	using DetectorElementPtr = std::shared_ptr<const Acts::TGeoDetectorElement>;
	using DetectorStore = std::vector<DetectorElementPtr>;

public:
	/**
 	 * @brief Constructer for ACTSAlgBase.
 	 * @param name unique string identifier for this instance
 	 * @param svcLoc a Service Locator passed by the Gaudi AlgManager
 	 */  
	ACTSAlgBase(const std::string& name, ISvcLocator* svcLoc);
	/**
 	 * @brief an initializer for the base reconstruction algorithm
 	 * Sets up geometry.
 	 */
	StatusCode initialize();
private:
	/**
 	 * @brief Sets up ACTS Detector geometry
 	 */ 
	void buildDetector();
	/**
 	 * @brief Builds the Magnetic field from dd4hep Detector information
 	 * @TODO: This can be done more naturally with a GeoSvc.
 	 */
	void buildBfield();

protected:
	//! Path to material file
	Gaudi::Property<std::string> m_matFile{this, "MatFile", std::string(""), "Path to the material description JSON file. Can be empty."};

	//! Path to tracker geometry file
	Gaudi::Property<std::string> m_tgeoFile{this, "TGeoFile", std::string(""), "Path to the tracker geometry file."};

	//! Encoding String for Tracker CellIDs
	Gaudi::Property<std::string> m_encodingStringVariable{this, "EncodingStringParameterName", "GlobalTrackerReadoutID", "The name of the DD4hep constant that contains the Encoding string for the detector"};


	/**
 	 * @brief Gets the geometry Mapping Tool (To decode Cell IDs)
 	 * @return ACTSTracking Geometry Mapping Tool
 	 */
	std::shared_ptr<ACTSTracking::GeometryIdMappingTool> geoIDMappingTool() const;

	/**
 	 * @brief Gets the Magnetic Field Context (ACTS)
 	 * @return ACTS Magnetic Field Context
 	 */
	const Acts::MagneticFieldContext& magneticFieldContext() const; 
	/**
         * @brief Gets the Geometry Context (ACTS)
         * @return ACTS Geometry Context
         */
	const Acts::GeometryContext& geometryContext() const;
	/**
         * @brief Gets the Calibration Context (ACTS)
         * @return ACTS Calibration Context
         */
	const Acts::CalibrationContext& calibrationContext() const; 

	/**
         * @brief Gets the Magnetic Field (ACTS)
         * @return ACTS Magnetic Field Provider
         */
	std::shared_ptr<Acts::MagneticFieldProvider> magneticField() const;
	/**
         * @brief Gets the Tracking Geometry (ACTS)
         * @return ACTS Tracking Geometry
         */
	std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry() const; 

	/**
         * @brief Determines which surface corresponds to provided hit
	 * @param hit A Tracker Hit
         * @return ACTS Surface
         */
	const Acts::Surface* findSurface(const edm4hep::TrackerHit hit) const;

private:
	std::shared_ptr<ACTSTracking::GeometryIdMappingTool> m_geoIDMappingTool; ///< Tool to decode Cell IDs

	Acts::MagneticFieldContext m_magneticFieldContext; ///< Magnetic Field Context
	std::shared_ptr<Acts::MagneticFieldProvider> m_magneticField; ///< Actual Magnetic Field

	Acts::GeometryContext m_geometryContext; ///< Geomtry Context
	DetectorStore m_detectorStore; ///< Detector Information
	std::shared_ptr<const Acts::TrackingGeometry> m_trackingGeometry = nullptr; ///< Tracking Geometry

	Acts::CalibrationContext m_calibrationContext; ///< Calibration Context

	SmartIF<IGeoSvc> m_geoSvc;
};

#endif
