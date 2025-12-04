#include "ActsGeoGen3Svc.h"

#include "k4ActsTracking/ActsGaudiLogger.h"

#include <k4FWCore/GaudiChecks.h>

#include <Acts/Definitions/Algebra.hpp>
#include <Acts/Definitions/Units.hpp>
#include <Acts/Geometry/Blueprint.hpp>
#include <Acts/Geometry/BlueprintNode.hpp>
#include <Acts/Geometry/BlueprintOptions.hpp>
#include <Acts/Geometry/ContainerBlueprintNode.hpp>
#include <Acts/Geometry/CylinderVolumeBounds.hpp>
#include <Acts/Geometry/Extent.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Geometry/GeometryIdentifier.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/Geometry/VolumeAttachmentStrategy.hpp>
#include <Acts/MagneticField/ConstantBField.hpp>
#include <Acts/Utilities/AxisDefinitions.hpp>
#include <Acts/Visualization/ObjVisualization3D.hpp>
#include <ActsPlugins/DD4hep/BlueprintBuilder.hpp>

#include <DD4hep/DetElement.h>
#include <DD4hep/Detector.h>

#include <GaudiKernel/StatusCode.h>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <array>

DECLARE_COMPONENT(ActsGeoGen3Svc)

ActsGeoGen3Svc::ActsGeoGen3Svc(const std::string& name, ISvcLocator* svcLoc) : base_class(name, svcLoc) {}

StatusCode ActsGeoGen3Svc::initialize() {
  m_geoSvc = Gaudi::svcLocator()->service<IGeoSvc>("GeoSvc");
  K4_GAUDI_CHECK(m_geoSvc);

  std::array<double, 3> magneticFieldVector = {0, 0, 0};
  std::array<double, 3> position            = {0, 0, 0};
  m_geoSvc->getDetector()->field().magneticField(position.data(), magneticFieldVector.data());
  debug() << fmt::format("Retrieved magnetic field at position {}: {}", position, magneticFieldVector) << endmsg;
  m_magneticField = std::make_shared<Acts::ConstantBField>(
      Acts::Vector3(magneticFieldVector[0] / dd4hep::tesla * Acts::UnitConstants::T,
                    magneticFieldVector[1] / dd4hep::tesla * Acts::UnitConstants::T,
                    magneticFieldVector[2] / dd4hep::tesla * Acts::UnitConstants::T));

  auto gaudiLogger = makeActsGaudiLogger(this);

  info() << fmt::format("Acts::cm: {}, dd4hep::cm: {}", Acts::UnitConstants::cm, dd4hep::cm) << endmsg;

  ActsPlugins::DD4hep::BlueprintBuilder builder{{
                                                    .dd4hepDetector = m_geoSvc->getDetector(),
                                                    .lengthScale    = Acts::UnitConstants::cm / dd4hep::cm,
                                                },
                                                gaudiLogger->cloneWithSuffix("|BlpBld")};

  using Acts::Experimental::Blueprint;
  using Acts::Experimental::BlueprintOptions;
  using namespace Acts::UnitLiterals;
  using enum Acts::AxisDirection;

  Blueprint::Config cfg;
  // Padding around subvolumes of the world volume
  cfg.envelope[AxisZ] = {20_mm, 20_mm};
  cfg.envelope[AxisR] = {0_mm, 20_mm};
  Blueprint root{cfg};

  auto& outer = root.addCylinderContainer("MAIA_v0", AxisR);
  outer.addStaticVolume(Acts::Transform3::Identity(),
                        std::make_unique<Acts::CylinderVolumeBounds>(0_mm, 10_mm, 1000_mm), "Beampipe");
  // We want to pull the next volume in towards the beampipe to map material to
  // the correct places in the end. We need to ensure that the enclosing
  // cylinder contains the beampipe entirely.
  outer.setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

  outer.addCylinderContainer("InnerTracker", AxisZ, [&](auto& innerTracker) {
    auto envelope = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {5_mm, 5_mm});

    auto barrel = builder.layerHelper()
                      .barrel()
                      .setAxes("XYZ")
                      .setPattern(m_layerPattern.value())
                      .setContainer(m_detElementName.value())
                      .setEnvelope(envelope)
                      .build();

    barrel->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

    // TODO: This currently doesn't work because the cylinder we get from the
    // barrel overlaps in z with the cylinder we get from here, because the
    // first (innermost) endcap layer "sticks" into the envelope of the barrel.
    // This will require dedicated stacking of the cylinders in r and z in the
    // order that doesn't produce overlaps.
    //
    // auto negEndcap = builder.layerHelper()
    //                      .endcap()
    //                      .setAxes("XZY")
    //                      .setContainer("InnerTrackerEndcap")
    //                      .setPattern("layer_pos\\d")
    //                      .setEnvelope(envelope)
    //                      .build();

    // negEndcap->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

    innerTracker.addChild(barrel);
    // innerTracker.addChild(negEndcap);
  });

  BlueprintOptions      options;
  Acts::GeometryContext gctxt{};

  debug() << "Constructing tracking geometry" << endmsg;
  m_trackingGeo = root.construct(options, gctxt, *gaudiLogger->cloneWithSuffix("|Construct"));

  debug() << "Creating visualiztion" << endmsg;
  Acts::ObjVisualization3D vis{};
  m_trackingGeo->visualize(vis, gctxt);
  vis.write("dumped_acts_geo.obj");

  return StatusCode::SUCCESS;
}
