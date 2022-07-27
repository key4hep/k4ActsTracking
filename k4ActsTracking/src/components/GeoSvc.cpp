// D. Elitez, July 2022
// Based on eic/juggler

#include "GeoSvc.h"
#include "Acts/Geometry/TrackingGeometry.hpp"
#include "Acts/MagneticField/MagneticFieldContext.hpp"
#include "Acts/Plugins/DD4hep/ConvertDD4hepDetector.hpp"
#include "Acts/Plugins/Json/JsonMaterialDecorator.hpp"
#include "Acts/Plugins/Json/MaterialMapJsonConverter.hpp"
#include "Acts/Surfaces/PlaneSurface.hpp"
#include "Acts/Visualization/GeometryView3D.hpp"
#include "Acts/Visualization/ObjVisualization3D.hpp"
#include "DD4hep/Printout.h"
#include "GaudiKernel/Service.h"
#include "TGeoManager.h"

using namespace Gaudi;

DECLARE_COMPONENT(GeoSvc)

GeoSvc::GeoSvc(const std::string& name, ISvcLocator* svc) : base_class(name, svc), m_log(msgSvc(), name) {}

GeoSvc::~GeoSvc() {
  if (m_dd4hepGeo != nullptr) {
    try {
      m_dd4hepGeo->destroyInstance();
      m_dd4hepGeo = nullptr;
    } catch (...) {
    }
  }
}

StatusCode GeoSvc::initialize() {
  if (m_xmlFileNames.size() > 0) {
    m_log << MSG::INFO << "Loading xml files from:  '" << m_xmlFileNames << "'" << endmsg;
  } else {
    m_log << MSG::ERROR << "No xml file!" << endmsg;
    return StatusCode::FAILURE;
  }

  /// Check if the DD4Hep Geometry is built successfully
  if (buildDD4HepGeo().isFailure()) {
    m_log << MSG::ERROR << "Could not build DD4Hep geometry" << endmsg;
    return StatusCode::FAILURE;
  } else {
    m_log << MSG::INFO << "DD4Hep geometry SUCCESSFULLY built" << endmsg;
  }

  Acts::BinningType bTypePhi              = Acts::equidistant;
  Acts::BinningType bTypeR                = Acts::equidistant;
  Acts::BinningType bTypeZ                = Acts::equidistant;
  double            layerEnvelopeR        = Acts::UnitConstants::mm;
  double            layerEnvelopeZ        = Acts::UnitConstants::mm;
  double            defaultLayerThickness = Acts::UnitConstants::fm;
  using Acts::sortDetElementsByID;
  m_trackingGeo = Acts::convertDD4hepDetector(m_dd4hepGeo->world(), m_actsLoggingLevel, bTypePhi, bTypeR, bTypeZ,
                                              layerEnvelopeR, layerEnvelopeZ, defaultLayerThickness,
                                              sortDetElementsByID, m_trackingGeoCtx, m_materialDeco);

  /// Setting geometry debug option
  if (m_debugGeometry == true) {
    m_log << MSG::INFO << "Geometry debugging is ON." << endmsg;

    if (createGeoObj().isFailure()) {
      m_log << MSG::ERROR << "Could not create geometry OBJ" << endmsg;
      return StatusCode::FAILURE;
    } else {
      m_log << MSG::INFO << "Geometry OBJ SUCCESSFULLY created" << endmsg;
    }
  } else {
    m_log << MSG::VERBOSE << "Geometry debugging is OFF." << endmsg;
    return StatusCode::SUCCESS;
  }
  std::cout << "works!" << std::endl;

  return StatusCode::SUCCESS;
}

StatusCode GeoSvc::execute() { return StatusCode::SUCCESS; }

StatusCode GeoSvc::finalize() { return StatusCode::SUCCESS; }

StatusCode GeoSvc::buildDD4HepGeo() {
  m_dd4hepGeo = &(dd4hep::Detector::getInstance());
  m_dd4hepGeo->addExtension<IGeoSvc>(this);

  /// Load geometry
  for (auto& filename : m_xmlFileNames) {
    m_log << MSG::INFO << "Loading geometry from file:  '" << filename << "'" << endmsg;
    m_dd4hepGeo->fromCompact(filename);
  }
  m_dd4hepGeo->volumeManager();
  m_dd4hepGeo->apply("DD4hepVolumeManager", 0, nullptr);

  return StatusCode::SUCCESS;
}

/// Create a geometry OBJ file
StatusCode GeoSvc::createGeoObj() {
  // Convert DD4Hep geometry to acts

  Acts::ObjVisualization3D m_obj;

  if (!m_trackingGeo) {
    return StatusCode::FAILURE;
  }
  m_trackingGeo->visitSurfaces([&](const Acts::Surface* surface) {
    if (surface == nullptr) {
      info() << "no surface??? " << endmsg;
      return;
    }
    Acts::GeometryView3D::drawSurface(m_obj, *surface, m_trackingGeoCtx);
  });
  m_obj.write(m_outputFileName);
  m_log << MSG::INFO << m_outputFileName << " SUCCESSFULLY written." << endmsg;

  return StatusCode::SUCCESS;
}
