// D. Elitez, July 2022
// Based on eic/juggler

#ifndef GEOSVC_H
#define GEOSVC_H

#include <Acts/Material/IMaterialDecorator.hpp>
#include "Acts/Definitions/Common.hpp"
#include "Acts/Definitions/Units.hpp"
#include "Acts/Geometry/GeometryContext.hpp"
#include "Acts/Geometry/TrackingGeometry.hpp"
#include "Acts/Plugins/DD4hep/DD4hepDetectorElement.hpp"
#include "Acts/Surfaces/Surface.hpp"
#include "Acts/Utilities/Logger.hpp"
#include "DD4hep/DD4hepUnits.h"
#include "DD4hep/Detector.h"
#include "DDRec/Surface.h"
#include "DDRec/SurfaceManager.h"
#include "GaudiKernel/MsgStream.h"
#include "GaudiKernel/Service.h"
#include "GaudiKernel/ServiceHandle.h"
#include "IGeoSvc.h"

class GeoSvc : public extends<Service, IGeoSvc> {
public:
  using VolumeSurfaceMap = std::unordered_map<uint64_t, const Acts::Surface*>;

private:
  dd4hep::Detector* m_dd4hepGeo = nullptr;

  /// DD4hep surface map
  std::map<int64_t, dd4hep::rec::Surface*> m_surfaceMap;

  /// ACTS Logging Level
  Acts::Logging::Level m_actsLoggingLevel = Acts::Logging::INFO;

  /// ACTS Tracking Geometry Context
  Acts::GeometryContext m_trackingGeoCtx;

  /// ACTS Tracking Geometry
  std::unique_ptr<const Acts::TrackingGeometry> m_trackingGeo{nullptr};

  /// ACTS Material Decorator
  std::shared_ptr<const Acts::IMaterialDecorator> m_materialDeco{nullptr};

  /// ACTS surface lookup container for hit surfaces that generate smeared hits
  VolumeSurfaceMap m_surfaces;

  /// XML-files with the detector description
  Gaudi::Property<std::vector<std::string>> m_xmlFileNames{this, "detectors", {}, "Detector descriptions XML-files"};

  /// Option for the Debug Geometry
  Gaudi::Property<bool> m_debugGeometry{this, "debugGeometry", false, "Option for geometry debugging"};
  /// Output file name
  Gaudi::Property<std::string> m_outputFileName{this, "outputFileName", "", "Output file name"};

  /// Gaudi logging output
  MsgStream m_log;

public:
  GeoSvc(const std::string& name, ISvcLocator* svc);

  virtual ~GeoSvc();

  virtual StatusCode initialize() final;

  virtual StatusCode execute() final;

  virtual StatusCode finalize() final;

  StatusCode buildDD4HepGeo();

  StatusCode createGeoObj();

  virtual const Acts::TrackingGeometry& trackingGeometry() const;
};

inline const Acts::TrackingGeometry& GeoSvc::trackingGeometry() const { return *m_trackingGeo; }
#endif
