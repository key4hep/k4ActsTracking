#include "ActsGeoSvc.h"

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
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/Utilities/AxisDefinitions.hpp>
#include <Acts/Visualization/ObjVisualization3D.hpp>
#include <ActsPlugins/DD4hep/BlueprintBuilder.hpp>
#include <ActsPlugins/DD4hep/DD4hepDetectorElement.hpp>

#include <DD4hep/DD4hepUnits.h>
#include <DD4hep/DetElement.h>
#include <DD4hep/Detector.h>

#include <GaudiKernel/StatusCode.h>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <array>

DECLARE_COMPONENT(ActsGeoSvc)

template <> struct fmt::formatter<Acts::GeometryIdentifier> : fmt::ostream_formatter {};

ActsGeoSvc::ActsGeoSvc(const std::string& name, ISvcLocator* svcLoc) : base_class(name, svcLoc) {}

StatusCode ActsGeoSvc::initialize() {
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

  // We have to create the inner tracker in several steps, because the inner
  // most endcap layer protrudes into the envelope that is created by the
  // outermost barrel layer. That creates an overlap in z while stacking. Hence,
  // we build it in steps grouping the innermost two layers of the barrel and
  // the innermost layer of the endcap into an "inner" inner tracker (stacking
  // them along z), we then stack the last barrel layer along r, before stacking
  // the remaining endcap layers along z. Additionally, we have to first put the
  // whole vertex detector inside the two innermost InnerTrackerBarrel layers
  // because the outermost vertex layer extends further in r, than the innermost
  // border of the InnerTracker endcaps. Hence, we also need to stack them in
  // the correct order.

  // NOTE: Need to set rather small padding here for the R-direction, because
  // the innermost two layers are a double layer for which the cylindrical
  // volumes are overlapping otherwise
  auto barrelEnvelope = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {0.4_mm, 0.4_mm});
  auto vertexBarrel =
      builder.layerHelper()
          .barrel()
          .setAxes("ZYX")
          .setPattern("layer_\\d")
          .setContainer("VertexBarrel")
          .setEnvelope(barrelEnvelope)
          .customize([&](const dd4hep::DetElement&, std::shared_ptr<Acts::Experimental::LayerBlueprintNode> layer) {
            // Force the Barrel onto the z-axis by not using the
            // center of gravity for auto-sizing. We do this because
            // the VertexBarrel has an odd number of modules, which
            // shifts them off-axis when using CoG
            layer->setUseCenterOfGravity(false, false, true);
            return layer;
          })
          .build();
  vertexBarrel->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

  // The VertexEncap again suffers from not having a *layer* DetElement
  auto endcapEnvelope        = Acts::ExtentEnvelope{}.set(AxisZ, {1_mm, 1_mm}).set(AxisR, {5_mm, 5_mm});
  auto vtxEndcapDetElem      = builder.findDetElementByName("VertexEndcap");
  auto negVtxEndcapContainer = std::make_shared<Acts::Experimental::CylinderContainerBlueprintNode>(
      "VertexEndcapNeg", Acts::AxisDirection::AxisZ);
  negVtxEndcapContainer->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);
  auto posVtxEndcapContainer = std::make_shared<Acts::Experimental::CylinderContainerBlueprintNode>(
      "VertexEndcapPos", Acts::AxisDirection::AxisZ);
  posVtxEndcapContainer->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

  for (int i = 0; i < 8; ++i) {
    auto layerName      = "layer" + std::to_string(i);
    auto layerPattern   = std::regex{layerName + "_module0_sensor\\d+_neg"};
    auto sensorElements = builder.findDetElementByNamePattern(vtxEndcapDetElem.value(), layerPattern);

    auto layer = builder.makeLayer(vtxEndcapDetElem.value(), sensorElements, "XZY", layerName + "_neg");
    layer->setEnvelope(endcapEnvelope);
    negVtxEndcapContainer->addChild(layer);

    layerPattern   = layerName + "_module0_sensor\\d+_pos";
    sensorElements = builder.findDetElementByNamePattern(vtxEndcapDetElem.value(), layerPattern);
    layer          = builder.makeLayer(vtxEndcapDetElem.value(), sensorElements, "XZY", layerName + "_pos");
    posVtxEndcapContainer->addChild(layer);
  }

  auto vertex = std::make_shared<Acts::Experimental::CylinderContainerBlueprintNode>("Vertex", AxisZ);
  vertex->addChild(vertexBarrel);
  vertex->addChild(negVtxEndcapContainer);
  vertex->addChild(posVtxEndcapContainer);

  auto envelope         = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {5_mm, 5_mm});
  auto innerInnerBarrel = builder.layerHelper()
                              .barrel()
                              .setAxes("XYZ")
                              .setPattern("layer[01]")
                              .setContainer("InnerTrackerBarrel")
                              .setEnvelope(envelope)
                              .build();
  innerInnerBarrel->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);
  innerInnerBarrel->addChild(vertex);

  auto outerInnerBarrel = builder.layerHelper()
                              .barrel()
                              .setAxes("XYZ")
                              .setPattern("layer2")
                              .setContainer("InnerTrackerBarrel")
                              .setEnvelope(envelope)
                              .build();
  outerInnerBarrel->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

  auto innerPosEndcapInner = builder.layerHelper()
                                 .endcap()
                                 .setAxes("YXZ")
                                 .setContainer("InnerTrackerEndcap")
                                 .setPattern("layer_pos0")
                                 .setEnvelope(envelope)
                                 .build();
  innerPosEndcapInner->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);
  auto outerPosEndcapInner = builder.layerHelper()
                                 .endcap()
                                 .setAxes("YXZ")
                                 .setContainer("InnerTrackerEndcap")
                                 .setPattern("layer_pos[1-6]")
                                 .setEnvelope(envelope)
                                 .build();
  outerPosEndcapInner->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

  auto innerNegEndcapInner = builder.layerHelper()
                                 .endcap()
                                 .setAxes("YXZ")
                                 .setContainer("InnerTrackerEndcap")
                                 .setPattern("layer_neg0")
                                 .setEnvelope(envelope)
                                 .build();
  innerNegEndcapInner->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);
  auto outerNegEndcapInner = builder.layerHelper()
                                 .endcap()
                                 .setAxes("YXZ")
                                 .setContainer("InnerTrackerEndcap")
                                 .setPattern("layer_neg[1-6]")
                                 .setEnvelope(envelope)
                                 .build();
  outerNegEndcapInner->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

  auto innerInnerTracker =
      std::make_shared<Acts::Experimental::CylinderContainerBlueprintNode>("InnerInnerTracker", AxisZ);
  innerInnerTracker->addChild(innerPosEndcapInner);
  innerInnerTracker->addChild(innerNegEndcapInner);
  innerInnerTracker->addChild(innerInnerBarrel);

  auto innerTrackerBarrel =
      std::make_shared<Acts::Experimental::CylinderContainerBlueprintNode>("InnerTrackerBarrel", AxisR);
  innerTrackerBarrel->addChild(innerInnerTracker);
  innerTrackerBarrel->addChild(outerInnerBarrel);

  outer.addCylinderContainer("InnerTracker", AxisZ, [&](auto& innerTracker) {
    innerTracker.addChild(innerTrackerBarrel);
    innerTracker.addChild(outerNegEndcapInner);
    innerTracker.addChild(outerPosEndcapInner);
  });

  // The OuterTracker is a bit more simple because it has the barrel and endcap
  // more clearly separated
  outer.addCylinderContainer("OuterTracker", AxisZ, [&](auto& outerTracker) {
    auto barrel = builder.layerHelper()
                      .barrel()
                      .setAxes("XYZ")
                      .setPattern("layer\\d")
                      .setContainer("OuterTrackerBarrel")
                      .setEnvelope(envelope)
                      .build();
    barrel->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

    auto negEndcap = builder.layerHelper()
                         .endcap()
                         .setAxes("YXZ")
                         .setContainer("OuterTrackerEndcap")
                         .setPattern("layer_neg\\d")
                         .setEnvelope(envelope)
                         .build();
    negEndcap->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

    auto posEndcap = builder.layerHelper()
                         .endcap()
                         .setAxes("YXZ")
                         .setContainer("OuterTrackerEndcap")
                         .setPattern("layer_pos\\d")
                         .setEnvelope(envelope)
                         .build();
    posEndcap->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

    outerTracker.addChild(barrel);
    outerTracker.addChild(negEndcap);
    outerTracker.addChild(posEndcap);
  });

  BlueprintOptions      options;
  Acts::GeometryContext gctxt{};

  debug() << "Constructing tracking geometry" << endmsg;
  m_trackingGeo = root.construct(options, gctxt, *gaudiLogger->cloneWithSuffix("|Construct"));

  std::size_t nSurfaces = 0;
  m_trackingGeo->visitSurfaces([&](const Acts::Surface* surface) {
    nSurfaces++;
    const auto& actsDetElem =
        dynamic_cast<const ActsPlugins::DD4hepDetectorElement&>(*surface->associatedDetectorElement());
    const auto& detElem = actsDetElem.sourceElement();
    verbose() << fmt::format("Adding Acts surface {} pointing to dd4hep DetElement {}", surface->geometryId(),
                             detElem.volumeID())
              << endmsg;
    const auto& [existing, inserted] = m_cellIDToSurface.emplace(detElem.volumeID(), surface);
    if (!inserted) {
      error() << fmt::format(
                     "The Acts surface {} pointing to dd4hep DetElement with cellID {} is already registered in the "
                     "map for Acts surface {}",
                     surface->geometryId(), detElem.volumeID(), existing->second->geometryId())
              << endmsg;
    }
  });

  info() << fmt::format("Visited {} Surfaces and inserted {} pairs of CellID -> Acts::Surface* into the map.",
                        nSurfaces, m_cellIDToSurface.size())
         << endmsg;
  if (m_dumpVisualization.value()) {
    info() << "Creating visualiztion" << endmsg;
    // Adjust the scale here to make it easier to import in blender
    Acts::ObjVisualization3D vis{4, 0.001};
    m_trackingGeo->visualize(vis, gctxt);
    vis.write(m_objDumpFileName.value());
  }

  return StatusCode::SUCCESS;
}
