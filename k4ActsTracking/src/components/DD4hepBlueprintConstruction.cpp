#include "DD4hepBlueprintConstruction.h"

#include <Acts/Definitions/Units.hpp>
#include <Acts/Geometry/Blueprint.hpp>
#include <Acts/Geometry/BlueprintNode.hpp>
#include <Acts/Geometry/ContainerBlueprintNode.hpp>
#include <Acts/Geometry/Extent.hpp>
#include <Acts/Geometry/VolumeAttachmentStrategy.hpp>
#include <Acts/Utilities/AxisDefinitions.hpp>
#include <ActsPlugins/DD4hep/BlueprintBuilder.hpp>
#include <ActsPlugins/Root/TGeoAxes.hpp>

#include <algorithm>
#include <ranges>
#include <regex>

using Acts::Experimental::ContainerBlueprintNode;
using Acts::Experimental::CylinderContainerBlueprintNode;

using namespace Acts::UnitLiterals;
using enum Acts::AxisDirection;

namespace Blueprints {
  /// Add a cylindrical beampipe to the passed node using the measures passed as arguments.
  ///
  /// We use this to enclose our actual beampipe because that is not a sipmle
  /// cylinder. However, we mainly need a surface / volume to attach the
  /// material of the beampipe, for which we use this cylindrical volume here.
  ///
  /// @param node The Blueprint container to which the beampipe should be added
  /// @param rMax The (initial) max radius of the cylinder
  /// @param halfZ the half-length in z of this cylinder
  void addCylindricalBeampipe(ContainerBlueprintNode& node, double rMax = 10_mm, double halfZ = 1000_mm) {
    node.addStaticVolume(Acts::Transform3::Identity(), std::make_unique<Acts::CylinderVolumeBounds>(0_mm, rMax, halfZ),
                         "Beampipe");
    // We want to pull the next volume in towards the beampipe to map material to
    // the correct places in the end. We need to ensure that the enclosing
    // cylinder contains the beampipe entirely.
    node.setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);
  }

  /// Make the Acts volumes for a VertexBarrel detector where the layers are
  /// grouped into double layers such that these double layers end up in one
  /// volume in the Acts geometry.
  ///
  /// This might be necessary in case the spacing between double layers is too
  /// small to have cylinder shells that do not overlap
  ///
  /// @param builder       The Blueprint builder that drives the construction
  /// @param containerName The detector name in which all the sensitive elements
  ///                      are placed
  /// @param layerRgx      The match expression to filter out the sensitive
  ///                      elements. @note that these should not be the
  ///                      "top-level" layer DetElements, but rather the
  ///                      ladders. @note This needs to contain exactly one
  ///                      matching group which has to be convertible to int as
  ///                      that is what will be used for grouping them into
  ///                      double layers
  ///
  /// @returns The vertex barrel blueprint node
  std::shared_ptr<CylinderContainerBlueprintNode> makeDoubleLayerVertexBarrel(
      ActsPlugins::DD4hep::BlueprintBuilder& builder, const std::string& containerName = "VertexBarrel",
      const std::regex& layerRgx = std::regex{"VertexBarrel_layer(\\d)_ladder\\d+"}) {
    // Vertex Barrel has a double layer gap of only 1 mm. This makes it
    // (almost) impossible to fit them into mutually exclusive cylinder shell
    // volumes. Hence, we make each double layer an Acts layer / volume.
    const auto vtxBarrelDetElem    = builder.findDetElementByName(containerName);
    const auto vtxBarrelLayerElems = builder.findDetElementByNamePattern(vtxBarrelDetElem.value(), layerRgx);

    const auto doubleLayerGrouper = [&](const std::span<const dd4hep::DetElement> elements) {
      const auto layerElements = elements | std::views::transform([&](const auto e) {
                                   std::smatch       match;
                                   const std::string elemName = e.name();
                                   std::regex_match(elemName, match, layerRgx);
                                   const auto layer = std::stoi(match[1].str());
                                   return std::make_pair(layer, e);
                                 });
      const auto nLayers = std::ranges::max(layerElements, std::less{}, [](const auto p) { return p.first; }).first;
      std::vector<std::vector<dd4hep::DetElement>> layers((nLayers + 1) / 2);

      for (const auto& [layer, elem] : layerElements) {
        layers[layer / 2].emplace_back(elem);
      }

      return layers;
    };

    auto vtxBarrel      = std::make_shared<Acts::Experimental::CylinderContainerBlueprintNode>("VertexBarrel",
                                                                                               Acts::AxisDirection::AxisR);
    auto barrelEnvelope = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {1_mm, 1_mm});

    int layerNum = 0;
    for (const auto& layerElems : doubleLayerGrouper(vtxBarrelLayerElems)) {
      const auto layerSpec =
          ActsPlugins::DD4hep::DD4hepBackend::LayerSpec{.axes      = ActsPlugins::TGeoAxes("ZYX"),
                                                        .layerAxes = std::nullopt,
                                                        .layerName = "doubleLayer_" + std::to_string(layerNum++)};
      auto layer = builder.makeLayer(vtxBarrelDetElem.value(), layerElems, layerSpec);
      layer->setEnvelope(barrelEnvelope);

      // Force the Barrel onto the z-axis by not using the
      // center of gravity for auto-sizing. We do this because
      // the VertexBarrel has an odd number of modules, which
      // shifts them off-axis when using CoG
      layer->setUseCenterOfGravity(false, false, true);
      vtxBarrel->addChild(layer);
    }
    vtxBarrel->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

    return vtxBarrel;
  }

  /// Attach the Endcaps to the VertexBarrel after constructing them to create
  /// the full Vertex detector node.
  ///
  /// This accepts an existing VertexBarrel blueprint node and stacks the
  /// endcaps onto it along the z-axis.
  ///
  /// @param builder       The Blueprint builder that drives the construction
  /// @param vtxBarrel     The vertex barrel bluprint node
  /// @param containerName The detector name in which all the sensitive elements
  ///                      are placed
  ///
  /// @param posLayerPattern The expression for selecting layers from the
  ///                      DetElement with the @containerName name for the
  ///                      positive endcap
  /// @param negLayerPattern The expression for selecting layers from the
  ///                      DetElement with the @containerName name for the
  ///                      negative endcap
  std::shared_ptr<CylinderContainerBlueprintNode> completeVertexWithEndcaps(
      ActsPlugins::DD4hep::BlueprintBuilder& builder, std::shared_ptr<CylinderContainerBlueprintNode>&& vtxBarrel,
      const std::string& containerName   = "VertexEndcap",
      const std::regex&  posLayerPattern = std::regex{"layer_pos\\d+"},
      const std::regex&  negLayerPattern = std::regex{"layer_neg\\d+"}) {
    // We use an Endcap envelope with smaller z-padding to accomodate for the double layer structure
    auto vtxEndcapEnvelope     = Acts::ExtentEnvelope{}.set(AxisZ, {1_mm, 1_mm}).set(AxisR, {5_mm, 5_mm});
    auto posVtxEndcapContainer = builder.layers()
                                     .endcap()
                                     .setAxes("XZY")
                                     .setContainer(containerName)
                                     .setFilter(posLayerPattern)
                                     .setEnvelope(vtxEndcapEnvelope)
                                     .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                     .build();

    auto negVtxEndcapContainer = builder.layers()
                                     .endcap()
                                     .setAxes("XZY")
                                     .setContainer(containerName)
                                     .setFilter(negLayerPattern)
                                     .setEnvelope(vtxEndcapEnvelope)
                                     .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                     .build();

    auto vertex = std::make_shared<Acts::Experimental::CylinderContainerBlueprintNode>("Vertex", AxisZ);
    vertex->addChild(vtxBarrel);
    vertex->addChild(negVtxEndcapContainer);
    vertex->addChild(posVtxEndcapContainer);
    return vertex;
  }

}  // namespace Blueprints

namespace MuColl {
  namespace MAIA_v0 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder) {
      auto& outer = root.addCylinderContainer(detName, AxisR);

      Blueprints::addCylindricalBeampipe(outer);

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
          builder.layers()
              .barrel()
              .setAxes("ZYX")
              .setFilter("layer_\\d")
              .setContainer("VertexBarrel")
              .setEnvelope(barrelEnvelope)
              .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
              .onLayer([&](const dd4hep::DetElement&, std::shared_ptr<Acts::Experimental::LayerBlueprintNode> layer) {
                // Force the Barrel onto the z-axis by not using the
                // center of gravity for auto-sizing. We do this because
                // the VertexBarrel has an odd number of modules, which
                // shifts them off-axis when using CoG
                layer->setUseCenterOfGravity(false, false, true);
                return layer;
              })
              .build();

      auto vertex = Blueprints::completeVertexWithEndcaps(builder, std::move(vertexBarrel));

      auto envelope         = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {5_mm, 5_mm});
      auto innerInnerBarrel = builder.layers()
                                  .barrel()
                                  .setAxes("XYZ")
                                  .setFilter("layer[01]")
                                  .setContainer("InnerTrackerBarrel")
                                  .setEnvelope(envelope)
                                  .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                  .build();
      innerInnerBarrel->addChild(vertex);

      auto outerInnerBarrel = builder.layers()
                                  .barrel()
                                  .setAxes("XYZ")
                                  .setFilter("layer2")
                                  .setContainer("InnerTrackerBarrel")
                                  .setEnvelope(envelope)
                                  .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                  .build();

      auto innerPosEndcapInner = builder.layers()
                                     .endcap()
                                     .setAxes("YXZ")
                                     .setContainer("InnerTrackerEndcap")
                                     .setFilter("layer_pos0")
                                     .setEnvelope(envelope)
                                     .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                     .build();

      auto outerPosEndcapInner = builder.layers()
                                     .endcap()
                                     .setAxes("YXZ")
                                     .setContainer("InnerTrackerEndcap")
                                     .setFilter("layer_pos[1-6]")
                                     .setEnvelope(envelope)
                                     .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                     .build();

      auto innerNegEndcapInner = builder.layers()
                                     .endcap()
                                     .setAxes("YXZ")
                                     .setContainer("InnerTrackerEndcap")
                                     .setFilter("layer_neg0")
                                     .setEnvelope(envelope)
                                     .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                     .build();
      auto outerNegEndcapInner = builder.layers()
                                     .endcap()
                                     .setAxes("YXZ")
                                     .setContainer("InnerTrackerEndcap")
                                     .setFilter("layer_neg[1-6]")
                                     .setEnvelope(envelope)
                                     .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                     .build();

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
        auto barrel = builder.layers()
                          .barrel()
                          .setAxes("XYZ")
                          .setFilter("layer\\d")
                          .setContainer("OuterTrackerBarrel")
                          .setEnvelope(envelope)
                          .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                          .build();

        auto negEndcap = builder.layers()
                             .endcap()
                             .setAxes("YXZ")
                             .setContainer("OuterTrackerEndcap")
                             .setFilter("layer_neg\\d")
                             .setEnvelope(envelope)
                             .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                             .build();

        auto posEndcap = builder.layers()
                             .endcap()
                             .setAxes("YXZ")
                             .setContainer("OuterTrackerEndcap")
                             .setFilter("layer_pos\\d")
                             .setEnvelope(envelope)
                             .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                             .build();

        outerTracker.addChild(barrel);
        outerTracker.addChild(negEndcap);
        outerTracker.addChild(posEndcap);
      });
    }

  }  // namespace MAIA_v0
}  // namespace MuColl

namespace FCCee {
  namespace ILD_FCCee_v01 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder) {
      auto& outer = root.addCylinderContainer(detName, AxisR);

      Blueprints::addCylindricalBeampipe(outer);

      auto vtxBarrel = Blueprints::makeDoubleLayerVertexBarrel(builder);
      auto vertex    = Blueprints::completeVertexWithEndcaps(builder, std::move(vtxBarrel));

      auto envelope           = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {5_mm, 5_mm});
      auto innerTrackerBarrel = builder.layers()
                                    .barrel()
                                    .setAxes("XYZ")
                                    .setContainer("InnerTrackerBarrel")
                                    .setFilter("layer\\d")
                                    .setEnvelope(envelope)
                                    .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                    .build();
      auto innerTrackerPosEndcap = builder.layers()
                                       .endcap()
                                       .setAxes("YXZ")
                                       .setContainer("InnerTrackerEndcap")
                                       .setFilter("layer_pos\\d")
                                       .setEnvelope(envelope)
                                       .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                       .build();
      auto innerTrackerNegEndcap = builder.layers()
                                       .endcap()
                                       .setAxes("YXZ")
                                       .setContainer("InnerTrackerEndcap")
                                       .setFilter("layer_neg\\d")
                                       .setEnvelope(envelope)
                                       .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                       .build();

      auto trackerBarrel = std::make_shared<Acts::Experimental::CylinderContainerBlueprintNode>("TrackerBarrel", AxisR);
      trackerBarrel->addChild(vertex);
      trackerBarrel->addChild(innerTrackerBarrel);

      outer.addCylinderContainer("InnerTracker", AxisZ, [&](auto& innerTracker) {
        innerTracker.addChild(trackerBarrel);
        innerTracker.addChild(innerTrackerNegEndcap);
        innerTracker.addChild(innerTrackerPosEndcap);
      });
    }
  }  // namespace ILD_FCCee_v01

  namespace ILD_FCCee_v02 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder) {
      using namespace Acts::UnitLiterals;
      using enum Acts::AxisDirection;

      auto& outer = root.addCylinderContainer(detName, AxisR);

      Blueprints::addCylindricalBeampipe(outer);
      auto vtxBarrel = Blueprints::makeDoubleLayerVertexBarrel(builder);
      auto vertex    = Blueprints::completeVertexWithEndcaps(builder, std::move(vtxBarrel));

      outer.addCylinderContainer("Vertex", AxisR, [&](auto& innerTracker) { innerTracker.addChild(vertex); });
    }
  }  // namespace ILD_FCCee_v02
}  // namespace FCCee
