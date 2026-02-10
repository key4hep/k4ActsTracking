#include "ActsGeoGen3PlaneSvc.h"

#include "k4ActsTracking/ActsGaudiLogger.h"

#include <k4FWCore/GaudiChecks.h>

#include <Acts/Definitions/Algebra.hpp>
#include <Acts/Definitions/Units.hpp>
#include <Acts/Geometry/Blueprint.hpp>
#include <Acts/Geometry/BlueprintNode.hpp>
#include <Acts/Geometry/BlueprintOptions.hpp>
#include <Acts/Geometry/ContainerBlueprintNode.hpp>
#include <Acts/Geometry/CuboidVolumeBounds.hpp>
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

DECLARE_COMPONENT(ActsGeoGen3PlaneSvc)

ActsGeoGen3PlaneSvc::ActsGeoGen3PlaneSvc(const std::string& name, ISvcLocator* svcLoc) : base_class(name, svcLoc) {}

StatusCode ActsGeoGen3PlaneSvc::initialize() {
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


// --- DD4hep sanity check: print layer world transforms before building planes ---
auto det = m_geoSvc->getDetector();
auto trackerDE = det->detector("Tracker");  // name in compact <detector name="Tracker">

info() << "DD4hep sanity: Tracker DetElement path/name=" << trackerDE.path()
       << " / " << trackerDE.name() << endmsg;

// helper lambda to print translation
auto dumpDE = [&](const dd4hep::DetElement& de) {
  // Copy the matrix (avoid dangling reference to temporary)
  auto w = de.nominal().worldTransformation();  // returns TGeoHMatrix in this setup

  const double* tr = w.GetTranslation();        // ROOT public API: returns double[3]

  // Convert dd4hep internal length units to mm explicitly
  double x_mm = tr[0] / dd4hep::mm;
  double y_mm = tr[1] / dd4hep::mm;
  double z_mm = tr[2] / dd4hep::mm;

  info() << fmt::format(
               "  DE {:<20} path={:<40}  world T [mm] = ({:9.3f}, {:9.3f}, {:9.3f})",
               de.name(), de.path(), x_mm, y_mm, z_mm)
         << endmsg;
};

// In your logs, layers are named layer0..layer3 (not id=1..4). We'll dump those.
for (int i = 0; i < 4; ++i) {
  std::string lname = fmt::format("layer{}", i);
  auto layerDE = trackerDE.child(lname);
  if (!layerDE.isValid()) {
    warning() << "DD4hep sanity: cannot find child DetElement '" << lname
              << "' under Tracker. Available path=" << trackerDE.path() << endmsg;
    continue;
  }
  dumpDE(layerDE);
}
//---------sanity check over--------------

  auto gaudiLogger = makeActsGaudiLogger(this);

  //info() << fmt::format("Acts::cm: {}, dd4hep::mm: {}", Acts::UnitConstants::cm, dd4hep::mm) << endmsg;

  ActsPlugins::DD4hep::BlueprintBuilder builder{{
                                                    .dd4hepDetector = m_geoSvc->getDetector(),
                                                    .lengthScale    = Acts::UnitConstants::mm / dd4hep::mm,
                                                },
                                                gaudiLogger->cloneWithSuffix("|BlpBld")};

  using Acts::Experimental::Blueprint;
  using Acts::Experimental::BlueprintOptions;
  using namespace Acts::UnitLiterals;
  using enum Acts::AxisDirection;

  Blueprint::Config cfg;
  // Padding around subvolumes of the world volume
  cfg.envelope[AxisX] = {10_mm, 10_mm};
  cfg.envelope[AxisY] = {10_mm, 10_mm};
  cfg.envelope[AxisZ] = {10_mm, 10_mm};
  Blueprint root{cfg};

// --------------------------------------
root.addCuboidContainer("LUXE", AxisZ, [&](auto& worldBox) {
  auto& tracker = worldBox.addCuboidContainer("OuterBox", AxisZ);
  auto envelope = Acts::ExtentEnvelope{}
                        .set(AxisZ, {0.4_mm, 0.4_mm})
                        .set(AxisX, {0.4_mm, 0.4_mm})
                        .set(AxisY, {0.4_mm, 0.4_mm});

    auto planes = builder.planeHelper()
                      .setAxes("XYZ")
		      .setLayerAxes("XYZ")
                      .setPattern(m_layerPattern.value())       // e.g. r"layer\d"
                      .setContainer(m_detElementName.value())   // e.g. "Tracker"
                      .setEnvelope(envelope)
                      .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::Gap)   // Gap, Midpoint, First
		      .build();
    tracker.addChild(planes);
 });
// --------------------------------------

  BlueprintOptions options;
  Acts::GeometryContext gctxt{};

  debug() << "Constructing tracking geometry" << endmsg;
  m_trackingGeo = root.construct(options, gctxt, *gaudiLogger->cloneWithSuffix("|Construct"));

  debug() << "Creating visualiztion" << endmsg;
  Acts::ObjVisualization3D vis{};
  m_trackingGeo->visualize(vis, gctxt);
  vis.write("dumped_acts_geo.obj");

  return StatusCode::SUCCESS;
}
