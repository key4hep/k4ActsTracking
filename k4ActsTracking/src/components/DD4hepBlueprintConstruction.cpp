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

namespace Blueprints {
  void addCylindricalBeampipe(Acts::Experimental::ContainerBlueprintNode& node, double rMax, double halfZ) {
    node.addStaticVolume(Acts::Transform3::Identity(), std::make_unique<Acts::CylinderVolumeBounds>(0_mm, rMax, halfZ),
                         "Beampipe");
    // We want to pull the next volume in towards the beampipe to map material to
    // the correct places in the end. We need to ensure that the enclosing
    // cylinder contains the beampipe entirely.
    node.setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);
  }
}  // namespace Blueprints

namespace MuColl {
  namespace MAIA_v0 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder) {
      using namespace Acts::UnitLiterals;
      using enum Acts::AxisDirection;
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
              .onLayer([&](const dd4hep::DetElement&, std::shared_ptr<Acts::Experimental::LayerBlueprintNode> layer) {
                // Force the Barrel onto the z-axis by not using the
                // center of gravity for auto-sizing. We do this because
                // the VertexBarrel has an odd number of modules, which
                // shifts them off-axis when using CoG
                layer->setUseCenterOfGravity(false, false, true);
                return layer;
              })
              .build();
      vertexBarrel->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

      // We use an Endcap envelope with smaller z-padding to accomodate for the double layer structure
      auto vtxEndcapEnvelope     = Acts::ExtentEnvelope{}.set(AxisZ, {1_mm, 1_mm}).set(AxisR, {5_mm, 5_mm});
      auto posVtxEndcapContainer = builder.layers()
                                       .endcap()
                                       .setAxes("XZY")
                                       .setContainer("VertexEndcap")
                                       .setFilter("layer_pos\\d+")
                                       .setEnvelope(vtxEndcapEnvelope)
                                       .build();

      auto negVtxEndcapContainer = builder.layers()
                                       .endcap()
                                       .setAxes("XZY")
                                       .setContainer("VertexEndcap")
                                       .setFilter("layer_neg\\d+")
                                       .setEnvelope(vtxEndcapEnvelope)
                                       .build();

      auto vertex = std::make_shared<Acts::Experimental::CylinderContainerBlueprintNode>("Vertex", AxisZ);
      vertex->addChild(vertexBarrel);
      vertex->addChild(negVtxEndcapContainer);
      vertex->addChild(posVtxEndcapContainer);

      auto envelope         = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {5_mm, 5_mm});
      auto innerInnerBarrel = builder.layers()
                                  .barrel()
                                  .setAxes("XYZ")
                                  .setFilter("layer[01]")
                                  .setContainer("InnerTrackerBarrel")
                                  .setEnvelope(envelope)
                                  .build();
      innerInnerBarrel->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);
      innerInnerBarrel->addChild(vertex);

      auto outerInnerBarrel = builder.layers()
                                  .barrel()
                                  .setAxes("XYZ")
                                  .setFilter("layer2")
                                  .setContainer("InnerTrackerBarrel")
                                  .setEnvelope(envelope)
                                  .build();
      outerInnerBarrel->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

      auto innerPosEndcapInner = builder.layers()
                                     .endcap()
                                     .setAxes("YXZ")
                                     .setContainer("InnerTrackerEndcap")
                                     .setFilter("layer_pos0")
                                     .setEnvelope(envelope)
                                     .build();
      innerPosEndcapInner->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);
      auto outerPosEndcapInner = builder.layers()
                                     .endcap()
                                     .setAxes("YXZ")
                                     .setContainer("InnerTrackerEndcap")
                                     .setFilter("layer_pos[1-6]")
                                     .setEnvelope(envelope)
                                     .build();
      outerPosEndcapInner->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

      auto innerNegEndcapInner = builder.layers()
                                     .endcap()
                                     .setAxes("YXZ")
                                     .setContainer("InnerTrackerEndcap")
                                     .setFilter("layer_neg0")
                                     .setEnvelope(envelope)
                                     .build();
      innerNegEndcapInner->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);
      auto outerNegEndcapInner = builder.layers()
                                     .endcap()
                                     .setAxes("YXZ")
                                     .setContainer("InnerTrackerEndcap")
                                     .setFilter("layer_neg[1-6]")
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
        auto barrel = builder.layers()
                          .barrel()
                          .setAxes("XYZ")
                          .setFilter("layer\\d")
                          .setContainer("OuterTrackerBarrel")
                          .setEnvelope(envelope)
                          .build();
        barrel->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

        auto negEndcap = builder.layers()
                             .endcap()
                             .setAxes("YXZ")
                             .setContainer("OuterTrackerEndcap")
                             .setFilter("layer_neg\\d")
                             .setEnvelope(envelope)
                             .build();
        negEndcap->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

        auto posEndcap = builder.layers()
                             .endcap()
                             .setAxes("YXZ")
                             .setContainer("OuterTrackerEndcap")
                             .setFilter("layer_pos\\d")
                             .setEnvelope(envelope)
                             .build();
        posEndcap->setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);

        outerTracker.addChild(barrel);
        outerTracker.addChild(negEndcap);
        outerTracker.addChild(posEndcap);
      });
    }

  }  // namespace MAIA_v0
}  // namespace MuColl

namespace FCCee {
  namespace ILD_FCCee {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder) {
      using namespace Acts::UnitLiterals;
      using enum Acts::AxisDirection;

      auto& outer = root.addCylinderContainer(detName, AxisR);

      Blueprints::addCylindricalBeampipe(outer);

      auto barrelEnvelope = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {1_mm, 1_mm});
      // Vertex Barrel has a double layer gap of only 1 mm. This makes it
      // (almost) impossible to fit them into mutually exclusive cylinder shell
      // volumes. Hence, we make each double layer an Acts layer / volume.
      const auto vtxBarrelDetElem    = builder.findDetElementByName("VertexBarrel");
      const auto vtxBarrelLayerRgx   = std::regex{"VertexBarrel_layer(\\d)_ladder\\d+"};
      const auto vtxBarrelLayerElems = builder.findDetElementByNamePattern(vtxBarrelDetElem.value(), vtxBarrelLayerRgx);

      const auto doubleLayerGrouper = [&](const std::span<const dd4hep::DetElement> elements) {
        const auto layerElements = elements | std::views::transform([&](const auto e) {
                                     std::smatch       match;
                                     const std::string elemName = e.name();
                                     std::regex_match(elemName, match, vtxBarrelLayerRgx);
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

      auto vtxBarrel = std::make_shared<Acts::Experimental::CylinderContainerBlueprintNode>("VertexBarrel",
                                                                                            Acts::AxisDirection::AxisR);

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

      // We use an Endcap envelope with smaller z-padding to accomodate for the double layer structure
      auto vtxEndcapEnvelope     = Acts::ExtentEnvelope{}.set(AxisZ, {1_mm, 1_mm}).set(AxisR, {5_mm, 5_mm});
      auto posVtxEndcapContainer = builder.layers()
                                       .endcap()
                                       .setAxes("XZY")
                                       .setContainer("VertexEndcap")
                                       .setFilter("layer_pos\\d+")
                                       .setEnvelope(vtxEndcapEnvelope)
                                       .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                       .build();

      auto negVtxEndcapContainer = builder.layers()
                                       .endcap()
                                       .setAxes("XZY")
                                       .setContainer("VertexEndcap")
                                       .setFilter("layer_neg\\d+")
                                       .setEnvelope(vtxEndcapEnvelope)
                                       .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                       .build();

      auto vertex = std::make_shared<Acts::Experimental::CylinderContainerBlueprintNode>("Vertex", AxisZ);
      vertex->addChild(vtxBarrel);
      vertex->addChild(negVtxEndcapContainer);
      vertex->addChild(posVtxEndcapContainer);

      auto envelope = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {5_mm, 5_mm});

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
  }  // namespace ILD_FCCee
}  // namespace FCCee
