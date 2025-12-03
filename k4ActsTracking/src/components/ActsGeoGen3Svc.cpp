#include "ActsGeoGen3Svc.h"

#include "k4ActsTracking/ActsGaudiLogger.h"

#include <k4FWCore/GaudiChecks.h>

#include <Acts/Definitions/Units.hpp>
#include <Acts/Geometry/Blueprint.hpp>
#include <Acts/Geometry/BlueprintNode.hpp>
#include <Acts/Geometry/BlueprintOptions.hpp>
#include <Acts/Geometry/ContainerBlueprintNode.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Geometry/GeometryIdentifier.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/Utilities/AxisDefinitions.hpp>
#include <Acts/Visualization/ObjVisualization3D.hpp>
#include <ActsPlugins/DD4hep/BlueprintBuilder.hpp>

#include <DD4hep/DetElement.h>
#include <DD4hep/Detector.h>

#include <GaudiKernel/StatusCode.h>

DECLARE_COMPONENT(ActsGeoGen3Svc)

ActsGeoGen3Svc::ActsGeoGen3Svc(const std::string& name, ISvcLocator* svcLoc) : base_class(name, svcLoc) {}

StatusCode ActsGeoGen3Svc::initialize() {
  m_geoSvc = Gaudi::svcLocator()->service<IGeoSvc>("GeoSvc");
  K4_GAUDI_CHECK(m_geoSvc);

  auto gaudiLogger = makeActsGaudiLogger(this);

  ActsPlugins::DD4hep::BlueprintBuilder builder{{
                                                    .dd4hepDetector = m_geoSvc->getDetector(),
                                                    .lengthScale    = Acts::UnitConstants::cm,
                                                },
                                                gaudiLogger->cloneWithSuffix("BlpBld")};

  using Acts::Experimental::Blueprint;
  using Acts::Experimental::BlueprintOptions;
  using namespace Acts::UnitLiterals;
  using enum Acts::AxisDirection;

  Blueprint::Config cfg;
  // Padding around subvolumes of the world volume
  cfg.envelope[AxisZ] = {20_mm, 20_mm};
  cfg.envelope[AxisR] = {0_mm, 20_mm};
  Blueprint root{cfg};

  debug() << "Finding " << m_detElementName.value() << " detector element" << endmsg;
  auto innerTrackerBarrelElem = builder.findDetElementByName(m_detElementName.value()).value();
  debug() << "Adding layers from " << m_detElementName.value() << " matching " << m_layerPattern.value() << endmsg;
  auto innerTrackerBarrel = builder.addLayers(innerTrackerBarrelElem, "XYZ", AxisR, std::regex{m_layerPattern.value()});

  debug() << "Found " << innerTrackerBarrel->children().size() << " children in innerTrackerBarrel" << endmsg;

  debug() << "Adding inner tracker barrel to the root blueprint" << endmsg;
  root.addChild(innerTrackerBarrel);

  BlueprintOptions      options;
  Acts::GeometryContext gctxt{};

  debug() << "Constructing tracking geometry" << endmsg;
  m_trackingGeo = root.construct(options, gctxt);

  debug() << "Creating visualiztion" << endmsg;
  Acts::ObjVisualization3D vis{};
  m_trackingGeo->visualize(vis, gctxt);
  vis.write("dumped_acts_geo.obj");

  return StatusCode::SUCCESS;
}
