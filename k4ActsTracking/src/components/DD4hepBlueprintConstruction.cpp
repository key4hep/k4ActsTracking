/*
 * Copyright (c) 2014-2024 Key4hep-Project.
 *
 * This file is part of Key4hep.
 * See https://key4hep.github.io/key4hep-doc/ for further info.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "DD4hepBlueprintConstruction.h"

#include <Acts/Definitions/Units.hpp>
#include <Acts/Geometry/Blueprint.hpp>
#include <Acts/Geometry/BlueprintBuilder.hpp>
#include <Acts/Geometry/BlueprintNode.hpp>
#include <Acts/Geometry/ContainerBlueprintNode.hpp>
#include <Acts/Geometry/Extent.hpp>
#include <Acts/Geometry/LayerBlueprintNode.hpp>
#include <Acts/Geometry/VolumeAttachmentStrategy.hpp>
#include <Acts/Utilities/AxisDefinitions.hpp>
#include <ActsPlugins/DD4hep/BlueprintBuilder.hpp>
#include <ActsPlugins/Root/TGeoAxes.hpp>

#include <fmt/core.h>

#include <memory>
#include <regex>
#include <stdexcept>

using Acts::Experimental::ContainerBlueprintNode;
using Acts::Experimental::CylinderContainerBlueprintNode;
using Acts::Experimental::LayerBlueprintNode;

using AxisDefinition = ActsPlugins::DD4hep::BlueprintBuilder::AxisDefinition;
using LayerGrouper   = Acts::Experimental::SensorLayerAssembler<ActsPlugins::DD4hep::DD4hepBackend>::LayerGrouper;

using namespace Acts::UnitLiterals;
using enum Acts::AxisDirection;

namespace Blueprints {
  /// Commonly used envelopes for blueprint construction below
  const auto kTrackerEnvelope      = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {5_mm, 5_mm});
  const auto kBarrelEnvelope       = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {1_mm, 1_mm});
  const auto kTightBarrelEnvelope  = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {0.4_mm, 0.4_mm});
  const auto kVertexEndcapEnvelope = Acts::ExtentEnvelope{}.set(AxisZ, {1_mm, 1_mm}).set(AxisR, {5_mm, 5_mm});
  const auto kUngroupedVertexEndcapEnvelope =
      Acts::ExtentEnvelope{}.set(AxisZ, {0.5_mm, 0.5_mm}).set(AxisR, {5_mm, 5_mm});

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

  /// Layer customizer function to force the Barrel onto the z-axis by not using
  /// the center of gravity for auto-sizing. This is useful for cases where the
  /// detectors have has an odd number of modules, which shifts them off the
  /// z-axis with the default sizing
  std::shared_ptr<LayerBlueprintNode> unsetXYCoG(const std::optional<dd4hep::DetElement>&,
                                                 std::shared_ptr<LayerBlueprintNode> layer) {
    layer->setUseCenterOfGravity(false, false, true);
    return layer;
  }

  /// Build a LayerGrouper that assigns sensor DetElements to named layer groups.
  ///
  /// The returned callable matches each DetElement name against @p groupRgx.
  /// Capture group 1 is extracted, optionally transformed by @p transformMatch,
  /// and appended to @p labelBase to form the group key
  /// (e.g. `"VertexBarrel|doubleLayer_0"`).  Throws if the name does not match.
  ///
  /// @param groupRgx      Regex with exactly one capture group selecting the
  ///                      layer index or identifier within the element name
  /// @param labelBase     Prefix for the resulting group label
  /// @param transformMatch Optional transform applied to capture group 1 before
  ///                       appending to @p labelBase (default: identity)
  template <typename TransformF = std::function<std::string(const std::string&)>>
  LayerGrouper makeLayerGrouper(
      std::regex groupRgx, std::string labelBase,
      TransformF transformMatch = [](const std::string& match) -> std::string { return match; }) {
    return [=](const auto& e) {
      std::smatch       match;
      const std::string elemName = e.name();
      if (std::regex_match(elemName, match, groupRgx)) {
        const auto matchgroup = match[1].str();
        return fmt::format("{}_{}", labelBase, transformMatch(match[1].str()));
      }
      throw std::invalid_argument(fmt::format("Could not match regex for grouping layers. DetElem name: {}", elemName));
    };
  }

  /// Add one endcap side to @p parent using pre-grouped layer DetElements.
  ///
  /// Delegates to @c builder.layers().endcap(), which requires the layers to
  /// already be organised into dedicated DetElements matched by @p filter
  /// inside @p container.
  ///
  /// @param builder   Blueprint builder driving the construction
  /// @param parent    Node to attach the endcap side to
  /// @param container Name of the DetElement that contains the endcap layers
  /// @param filter    Regex selecting the relevant layer DetElements
  /// @param axes      Sensor coordinate axes for this endcap side
  /// @param envelope  Extent envelope applied to the resulting volume
  void addGroupedEndcapSide(ActsPlugins::DD4hep::BlueprintBuilder& builder, Acts::Experimental::BlueprintNode& parent,
                            const std::string& container, const std::regex& filter, AxisDefinition axes,
                            const Acts::ExtentEnvelope& envelope) {
    builder.layers()
        .endcap()
        .setSensorAxes(std::move(axes))
        .setContainer(container)
        .setLayerFilter(filter)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(parent);
  }

  /// Add one endcap side to @p parent by collecting sensor DetElements and
  /// grouping them into layers via @c builder.layersFromSensors().
  ///
  /// Use this when sensors have not been placed into dedicated layer
  /// DetElements.  The grouper is built with @p labelPrefix as the label base
  /// and @p keyXform to convert capture group 1 of @p filter to a group key.
  ///
  /// @param builder     Blueprint builder driving the construction
  /// @param parent      Node to attach the endcap side to
  /// @param container   Name of the DetElement that contains the sensors
  /// @param filter      Regex selecting sensor DetElements; capture group 1
  ///                    is used as the layer discriminator
  /// @param axes        Sensor coordinate axes for this endcap side
  /// @param labelPrefix Prefix passed to @c makeLayerGrouper as the label base
  /// @param envelope    Extent envelope applied to the resulting volume
  /// @param keyXform    Transform applied to capture group 1 to derive the
  ///                    group key (default: identity)
  void addUngroupedEndcapSide(
      ActsPlugins::DD4hep::BlueprintBuilder& builder, Acts::Experimental::BlueprintNode& parent,
      const std::string& container, const std::regex& filter, AxisDefinition axes, const std::string& labelPrefix,
      const Acts::ExtentEnvelope&                    envelope,
      std::function<std::string(const std::string&)> keyXform = [](const std::string& m) { return m; }) {
    const auto detElem = builder.findDetElementByName(container);
    auto       sensors = builder.findDetElementByNamePattern(detElem.value(), filter);
    auto       grouper = makeLayerGrouper(filter, labelPrefix, std::move(keyXform));
    builder.layersFromSensors()
        .endcap()
        .setSensorAxes(std::move(axes))
        .setContainerName(container)
        .setSensors(std::move(sensors))
        .groupBy(std::move(grouper))
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(parent);
  }

  /// Make the Acts volumes for a barrel detector where sensors are grouped into
  /// double layers, each double layer ending up in one volume in the Acts geometry.
  ///
  /// Use this when sensors are not placed into dedicated layer DetElements and
  /// the spacing between adjacent layers is too small for non-overlapping
  /// cylinder shells if each layer were its own volume.
  ///
  /// @param builder       The Blueprint builder that drives the construction
  /// @param containerName The name of the DetElement containing the sensors
  /// @param layerRgx      Regex to select sensor DetElements. Must not match
  ///                      top-level layer DetElements but the individual sensors
  ///                      (e.g. ladders). Must contain exactly one capture group
  ///                      whose value is convertible to int — adjacent pairs
  ///                      (floor(n/2)) are merged into a single double layer.
  ///
  /// @returns The barrel blueprint node
  std::shared_ptr<ContainerBlueprintNode> makeDoubleLayerBarrel(ActsPlugins::DD4hep::BlueprintBuilder& builder,
                                                                const std::string& containerName = "VertexBarrel",
                                                                const std::regex&  layerRgx      = std::regex{
                                                                    "VertexBarrel_layer(\\d)_ladder\\d+"}) {
    // Vertex Barrel has a double layer gap of only 1 mm. This makes it
    // (almost) impossible to fit them into mutually exclusive cylinder shell
    // volumes. Hence, we make each double layer an Acts layer / volume.
    const auto barrelDetElem    = builder.findDetElementByName(containerName);
    const auto barrelLayerElems = builder.findDetElementByNamePattern(barrelDetElem.value(), layerRgx);

    const auto doubleLayerName = makeLayerGrouper(layerRgx, fmt::format("{}|doubleLayer", containerName),
                                                  [](const auto& m) { return std::stoi(m) / 2; });

    return builder.layersFromSensors()
        .barrel()
        .setEnvelope(kBarrelEnvelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .setSensorAxes("ZYX")
        .setSensors(std::move(barrelLayerElems))
        .groupBy(doubleLayerName)
        .setContainerName(containerName)
        .onLayer(unsetXYCoG)
        .build();
  }

  /// A simple struct to contain the configuration for building a regular
  /// detector where the barrel and the endcaps can be cleanly stacked along the
  /// z-axis
  struct TrackerSpec {
    enum class Layout { Grouped, Ungrouped };
    std::string    barrelContainer;  ///< Name of the DetElement containing the barrel
    AxisDefinition barrelAxes;       ///< The axes directions for the barrel sensors
    std::regex     barrelFilter;     ///< The layer pattern to filter out barrel layers
    std::string    endcapContainer;  ///< Name of the DetElement containing the endcaps
    AxisDefinition endcapAxes;       ///< The axes directions for the endcap sensors
    std::regex     endcapPosFilter;  ///< The layer pattern to filter out positive endcap layers
    std::regex     endcapNegFilter;  ///< The layer pattern to filter out negative endcap layers
    Layout         layout = Layout::Grouped;
  };

  // Vertex endcap specs reuse TrackerSpec — barrel fields are ignored since the
  // vertex barrel is always built separately via makeDoubleLayerVertexBarrel.
  const auto GroupedVertexSpec = TrackerSpec{
      .barrelContainer = {},
      .barrelAxes      = "XYZ",
      .barrelFilter    = {},
      .endcapContainer = "VertexEndcap",
      .endcapAxes      = "XZY",
      .endcapPosFilter = std::regex{"layer_pos\\d+"},
      .endcapNegFilter = std::regex{"layer_neg\\d+"},
  };

  const auto UngroupedVertexSpec = TrackerSpec{
      .barrelContainer = {},
      .barrelAxes      = "XYZ",
      .barrelFilter    = {},
      .endcapContainer = "VertexEndcap",
      .endcapAxes      = "XZY",
      .endcapPosFilter = std::regex{"layer(\\d+)_module\\d+_sensor\\d+_pos"},
      .endcapNegFilter = std::regex{"layer(\\d+)_module\\d+_sensor\\d+_neg"},
      .layout          = TrackerSpec::Layout::Ungrouped,
  };

  /// Make a barrel blueprint node for a generic cylindrical detector.
  ///
  /// Uses the pre-grouped layer DetElements directly (via @c layers()), so
  /// the barrel layers must already be organised into dedicated DetElements
  /// matching @p spec.barrelFilter inside @p spec.barrelContainer.
  ///
  /// @param builder The Blueprint builder that drives the construction
  /// @param spec    Configuration spec (barrelContainer, barrelAxes,
  ///                barrelFilter used; endcap fields ignored)
  ///
  /// @returns The barrel blueprint node
  std::shared_ptr<ContainerBlueprintNode> makeBarrel(ActsPlugins::DD4hep::BlueprintBuilder& builder,
                                                     const TrackerSpec&                     spec) {
    return builder.layers()
        .barrel()
        .setSensorAxes(spec.barrelAxes)
        .setLayerFilter(spec.barrelFilter)
        .setContainer(spec.barrelContainer)
        .setEnvelope(kBarrelEnvelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .onLayer(unsetXYCoG)
        .build();
  }

  /// Attach endcaps to an existing barrel node to form a complete cylindrical
  /// detector node stacked along the z-axis.
  ///
  /// Dispatches on @p spec.layout: Grouped uses pre-existing layer DetElements
  /// directly; Ungrouped collects sensor DetElements and groups them internally.
  ///
  /// @param builder       The Blueprint builder that drives the construction
  /// @param barrel        The barrel blueprint node to attach the endcaps to
  /// @param spec          Endcap configuration (container name, axes, pos/neg filters,
  ///                      layout); barrel fields of the spec are ignored
  /// @param containerName Name of the resulting top-level cylinder container node
  /// @param keyXform      Transform applied to capture group 1 of the filter regex
  ///                      to derive the layer-group key (Ungrouped path only)
  std::shared_ptr<CylinderContainerBlueprintNode> attachEndcaps(
      ActsPlugins::DD4hep::BlueprintBuilder& builder, std::shared_ptr<ContainerBlueprintNode>&& barrel,
      const TrackerSpec& spec = GroupedVertexSpec, const std::string& containerName = "Vertex",
      std::function<std::string(const std::string&)> keyXform = [](const std::string& m) {
        return std::to_string(std::stoi(m) / 2);
      }) {
    auto node = std::make_shared<CylinderContainerBlueprintNode>(containerName, AxisZ);
    node->addChild(barrel);

    if (spec.layout == TrackerSpec::Layout::Grouped) {
      addGroupedEndcapSide(builder, *node, spec.endcapContainer, spec.endcapPosFilter, spec.endcapAxes,
                           kVertexEndcapEnvelope);
      addGroupedEndcapSide(builder, *node, spec.endcapContainer, spec.endcapNegFilter, spec.endcapAxes,
                           kVertexEndcapEnvelope);
    } else {
      addUngroupedEndcapSide(builder, *node, spec.endcapContainer, spec.endcapPosFilter, spec.endcapAxes,
                             fmt::format("{}|doubleLayer_pos", spec.endcapContainer), kUngroupedVertexEndcapEnvelope,
                             keyXform);
      addUngroupedEndcapSide(builder, *node, spec.endcapContainer, spec.endcapNegFilter, spec.endcapAxes,
                             fmt::format("{}|doubleLayer_neg", spec.endcapContainer), kUngroupedVertexEndcapEnvelope,
                             std::move(keyXform));
    }
    return node;
  }

  const auto OuterTrackerSpec = TrackerSpec{
      .barrelContainer = "OuterTrackerBarrel",
      .barrelAxes      = "XYZ",
      .barrelFilter    = std::regex{"layer\\d"},
      .endcapContainer = "OuterTrackerEndcap",
      .endcapAxes      = "YXZ",
      .endcapPosFilter = std::regex{"layer_pos(\\d)"},
      .endcapNegFilter = std::regex{"layer_neg(\\d)"},
  };

  const auto UngroupedOuterTrackerSpec = TrackerSpec{
      .barrelContainer = "OuterTrackerBarrel",
      .barrelAxes      = "XYZ",
      .barrelFilter    = std::regex{"layer\\d"},
      .endcapContainer = "OuterTrackerEndcap",
      .endcapAxes      = "YXZ",
      .endcapPosFilter = std::regex{"layer(\\d+)_module\\d+_sensor\\d+_pos"},
      .endcapNegFilter = std::regex{"layer(\\d+)_module\\d+_sensor\\d+_neg"},
      .layout          = TrackerSpec::Layout::Ungrouped,
  };

  const auto UngroupedInnerTrackerSpec = TrackerSpec{
      .barrelContainer = "InnerTrackerBarrel",
      .barrelAxes      = "XYZ",
      .barrelFilter    = std::regex{"layer\\d"},
      .endcapContainer = "InnerTrackerEndcap",
      .endcapAxes      = "YXZ",
      .endcapPosFilter = std::regex{"layer(\\d+)_module\\d+_sensor\\d+_pos"},
      .endcapNegFilter = std::regex{"layer(\\d+)_module\\d+_sensor\\d+_neg"},
      .layout          = TrackerSpec::Layout::Ungrouped,
  };

  /// Make the Acts volumes for a regular tracker consisting of a barrel and two
  /// endcaps that can be cleanly stacked along the z-axis without nesting.
  ///
  /// This is the simple case where all endcap layers fit within the z-extent of
  /// the barrel, i.e. no endcap layer protrudes into the radial envelope of the
  /// barrel layers. The barrel and both endcaps are stacked along z inside a
  /// single container node. Dispatches on @p spec.layout for grouped vs.
  /// ungrouped endcap sensor DetElements.
  ///
  /// @param builder     The Blueprint builder that drives the construction
  /// @param spec        The configuration spec defining the barrel and endcap
  ///                    container names, sensor axes, layer filters, and layout
  /// @param trackerName The name of the resulting top-level tracker node
  ///
  /// @returns The tracker blueprint node
  std::shared_ptr<CylinderContainerBlueprintNode> makeRegularTracker(ActsPlugins::DD4hep::BlueprintBuilder& builder,
                                                                     const TrackerSpec&                     spec,
                                                                     const std::string& trackerName) {
    auto tracker = std::make_shared<CylinderContainerBlueprintNode>(trackerName, AxisZ);

    builder.layers()
        .barrel()
        .setSensorAxes(spec.barrelAxes)
        .setLayerFilter(spec.barrelFilter)
        .setContainer(spec.barrelContainer)
        .setEnvelope(kTrackerEnvelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*tracker);

    if (spec.layout == TrackerSpec::Layout::Grouped) {
      addGroupedEndcapSide(builder, *tracker, spec.endcapContainer, spec.endcapNegFilter, spec.endcapAxes,
                           kTrackerEnvelope);
      addGroupedEndcapSide(builder, *tracker, spec.endcapContainer, spec.endcapPosFilter, spec.endcapAxes,
                           kTrackerEnvelope);
    } else {
      addUngroupedEndcapSide(builder, *tracker, spec.endcapContainer, spec.endcapNegFilter, spec.endcapAxes,
                             fmt::format("{}|layer_neg", spec.endcapContainer), kTrackerEnvelope);
      addUngroupedEndcapSide(builder, *tracker, spec.endcapContainer, spec.endcapPosFilter, spec.endcapAxes,
                             fmt::format("{}|layer_pos", spec.endcapContainer), kTrackerEnvelope);
    }
    return tracker;
  }

  /// A simple struct to hold configuration to build a tracker that is nested
  /// such that a simple stacking in z does not work.
  struct NestedInnerTrackerSpec {
    std::string    barrelContainer{"InnerTrackerBarrel"};    ///< Name of the DetElement containing the barrel
    AxisDefinition barrelAxes{"XYZ"};                        ///< The axes directions for the barrel sensors
    std::regex barrelInnerFilter = std::regex{"layer[01]"};  ///< The layer pattern to filter the inner barrel layers
                                                             ///< that enclose the vertex detector
    std::regex barrelOuterFilter = std::regex{"layer2"};     ///< The layer pattern to filter the outer barrel
                                                             ///< layer(s) stacked around the inner barrel
    std::string    endcapContainer{"InnerTrackerEndcap"};    ///< Name of the DetElement containing the endcaps
    AxisDefinition endcapAxes{"YXZ"};                        ///< The axes directions for the endcap sensors
    std::regex     endcapPosInnerFilter = std::regex{"layer_pos0"};  ///< The layer pattern to filter the innermost
                                                                     ///< positive endcap layers that protrude into
                                                                     ///< the barrel radial envelope
    std::regex endcapPosOuterFilter = std::regex{"layer_pos[1-6]"};  ///< The layer pattern to filter the outer
                                                                     ///< positive endcap layers
    std::regex endcapNegInnerFilter = std::regex{"layer_neg0"};      ///< The layer pattern to filter the innermost
                                                                     ///< negative endcap layers that protrude into
                                                                     ///< the barrel radial envelope
    std::regex endcapNegOuterFilter = std::regex{"layer_neg[1-6]"};  ///< The layer pattern to filter the outer
                                                                     ///< negative endcap layers
    enum class Layout { Grouped, Ungrouped };
    Layout layout = Layout::Grouped;
  };

  const auto UngroupedNestedInnerTrackerSpec = NestedInnerTrackerSpec{
      .barrelContainer      = "InnerTrackerBarrel",
      .barrelAxes           = "XYZ",
      .barrelInnerFilter    = std::regex{"layer[01]"},
      .barrelOuterFilter    = std::regex{"layer2"},
      .endcapContainer      = "InnerTrackerEndcap",
      .endcapAxes           = "YXZ",
      .endcapPosInnerFilter = std::regex{"layer(0)_module\\d+_sensor\\d+_pos"},
      .endcapPosOuterFilter = std::regex{"layer([1-6])_module\\d+_sensor\\d+_pos"},
      .endcapNegInnerFilter = std::regex{"layer(0)_module\\d+_sensor\\d+_neg"},
      .endcapNegOuterFilter = std::regex{"layer([1-6])_module\\d+_sensor\\d+_neg"},
      .layout               = NestedInnerTrackerSpec::Layout::Ungrouped,
  };

  /// Make a nested inner tracker that encloses the vertex.
  ///
  /// Nesting in this case means that at least one of the endcap layers
  /// protrudes into the cylinder described by the barrel layers. This makes it
  /// necessary to stack the volumes surrounding the layers in the correct order
  /// in r and z to avoid overlapping volumes.
  ///
  /// For this specific case the tracker can only be nested "once" this means
  /// that it looks something like the following.
  ///
  ///      b                                            b
  ///      a                                            a
  ///      r   endcap(Pos|Neg)OuterFilter               r
  ///      r    ⌄  ⌄  ⌄                       ⌄  ⌄  ⌄   r
  ///      r    |  |  | ───────────────────── |  |  | < e
  ///      e    |  |  | ───────────────────── |  |  | < l
  ///      l >  |  |  |  |  | ───────── |  |  |  |  |   O
  ///      I >  |  |  |  |  | ───────── |  |  |  |  |   u
  ///      n    |  |  |  |  |           |  |  |  |  |   t
  ///      n    |  |  |  |  |    VTX    |  |  |  |  |   e
  ///      e    |  |  |  |  |           |  |  |  |  |   r
  ///      r >  |  |  |  |  | ───────── |  |  |  |  |   F
  ///      F >  |  |  |  |  | ───────── |  |  |  |  |   i
  ///      i    |  |  | ───────────────────── |  |  | < l
  ///      l    |  |  | ───────────────────── |  |  | < t
  ///      t                                            e
  ///      e             ^  ^           ^  ^            r
  ///      r             endcap(Pos|Neg)InnerFilter
  ///
  /// The labels correspond to the members of the NestedInnerTrackerSpec.
  /// Dispatches on @p spec.layout for grouped vs. ungrouped endcap sensors.
  ///
  /// @param builder  The Blueprint builder that drives the construction
  /// @param vertex   The vertex detector blueprint node
  /// @param spec     The spec for defining how the nesting is done specifically
  ///                 for this detector
  ///
  /// @returns The inner tracker blueprint node
  std::shared_ptr<CylinderContainerBlueprintNode> makeNestedInnerTracker(
      ActsPlugins::DD4hep::BlueprintBuilder& builder, std::shared_ptr<CylinderContainerBlueprintNode>&& vertex,
      const NestedInnerTrackerSpec& spec = NestedInnerTrackerSpec{}) {
    // We have to create the inner tracker in several steps, because the inner
    // most endcap layer protrudes into the envelope that is created by the
    // outermost barrel layer. That creates an overlap in z while stacking.
    // Hence, we build it in steps grouping the innermost two layers of the
    // barrel and the innermost layer of the endcap into an "inner" inner
    // tracker (stacking them along z), we then stack the last barrel layer
    // along r, before stacking the remaining endcap layers along z.
    // Additionally, we have to first put the whole vertex detector inside the
    // two innermost InnerTrackerBarrel layers because the outermost vertex
    // layer extends further in r, than the innermost border of the InnerTracker
    // endcaps. Hence, we also need to stack them in the correct order.
    auto innerInnerBarrel = builder.layers()
                                .barrel()
                                .setSensorAxes(spec.barrelAxes)
                                .setLayerFilter(spec.barrelInnerFilter)
                                .setContainer(spec.barrelContainer)
                                .setEnvelope(kTrackerEnvelope)
                                .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                .onLayer(Blueprints::unsetXYCoG)
                                .build();
    innerInnerBarrel->addChild(vertex);

    auto innerInnerTracker = std::make_shared<CylinderContainerBlueprintNode>("InnerInnerTracker", AxisZ);
    innerInnerTracker->addChild(innerInnerBarrel);

    if (spec.layout == NestedInnerTrackerSpec::Layout::Grouped) {
      addGroupedEndcapSide(builder, *innerInnerTracker, spec.endcapContainer, spec.endcapPosInnerFilter,
                           spec.endcapAxes, kTrackerEnvelope);
      addGroupedEndcapSide(builder, *innerInnerTracker, spec.endcapContainer, spec.endcapNegInnerFilter,
                           spec.endcapAxes, kTrackerEnvelope);
    } else {
      addUngroupedEndcapSide(builder, *innerInnerTracker, spec.endcapContainer, spec.endcapPosInnerFilter,
                             spec.endcapAxes, fmt::format("{}|layer_pos", spec.endcapContainer), kTrackerEnvelope);
      addUngroupedEndcapSide(builder, *innerInnerTracker, spec.endcapContainer, spec.endcapNegInnerFilter,
                             spec.endcapAxes, fmt::format("{}|layer_neg", spec.endcapContainer), kTrackerEnvelope);
    }

    auto innerTracker = std::make_shared<CylinderContainerBlueprintNode>("InnerTracker", AxisZ);
    innerTracker->addCylinderContainer("InnerTrackerBarrel", AxisR, [&](auto& innerBarrel) {
      innerBarrel.addChild(innerInnerTracker);
      builder.layers()
          .barrel()
          .setSensorAxes(spec.barrelAxes)
          .setContainer(spec.barrelContainer)
          .setLayerFilter(spec.barrelOuterFilter)
          .setEnvelope(kTrackerEnvelope)
          .onLayer(Blueprints::unsetXYCoG)
          .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
          .addTo(innerBarrel);
    });

    if (spec.layout == NestedInnerTrackerSpec::Layout::Grouped) {
      addGroupedEndcapSide(builder, *innerTracker, spec.endcapContainer, spec.endcapPosOuterFilter, spec.endcapAxes,
                           kTrackerEnvelope);
      addGroupedEndcapSide(builder, *innerTracker, spec.endcapContainer, spec.endcapNegOuterFilter, spec.endcapAxes,
                           kTrackerEnvelope);
    } else {
      addUngroupedEndcapSide(builder, *innerTracker, spec.endcapContainer, spec.endcapPosOuterFilter, spec.endcapAxes,
                             fmt::format("{}|layer_pos", spec.endcapContainer), kTrackerEnvelope);
      addUngroupedEndcapSide(builder, *innerTracker, spec.endcapContainer, spec.endcapNegOuterFilter, spec.endcapAxes,
                             fmt::format("{}|layer_neg", spec.endcapContainer), kTrackerEnvelope);
    }

    return innerTracker;
  }

}  // namespace Blueprints

namespace MuColl {
  namespace MAIA_v0 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder) {
      auto& outer = root.addCylinderContainer(detName, AxisR);
      Blueprints::addCylindricalBeampipe(outer);

      // NOTE: Need to set rather small padding here for the R-direction, because
      // the innermost two layers are a double layer for which the cylindrical
      // volumes are overlapping otherwise
      auto vertexBarrel = builder.layers()
                              .barrel()
                              .setSensorAxes("ZYX")
                              .setLayerFilter("layer_\\d")
                              .setContainer("VertexBarrel")
                              .setEnvelope(Blueprints::kTightBarrelEnvelope)
                              .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                              .onLayer(Blueprints::unsetXYCoG)
                              .build();
      auto vertex = Blueprints::attachEndcaps(builder, std::move(vertexBarrel));

      auto innerTracker = Blueprints::makeNestedInnerTracker(builder, std::move(vertex));
      outer.addChild(innerTracker);

      auto outerTracker = Blueprints::makeRegularTracker(builder, Blueprints::OuterTrackerSpec, "OuterTracker");
      outer.addChild(outerTracker);
    }
  }  // namespace MAIA_v0
}  // namespace MuColl

namespace FCCee {
  namespace ILD_FCCee_v01 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder) {
      auto& outer = root.addCylinderContainer(detName, AxisR);

      Blueprints::addCylindricalBeampipe(outer);

      auto vtxBarrel = Blueprints::makeDoubleLayerBarrel(builder);
      auto vertex    = Blueprints::attachEndcaps(builder, std::move(vtxBarrel), Blueprints::UngroupedVertexSpec);

      auto innerTrackerBarrel = Blueprints::makeBarrel(builder, Blueprints::UngroupedInnerTrackerSpec);
      innerTrackerBarrel->addChild(vertex);

      auto innerTrackerEndcap = Blueprints::attachEndcaps(builder, std::move(innerTrackerBarrel),
                                                          Blueprints::UngroupedInnerTrackerSpec, "InnerTrackerEndcap");
      outer.addChild(innerTrackerEndcap);

      auto set = Blueprints::makeDoubleLayerBarrel(builder, "SET", std::regex{"set_ladder_(\\d)_\\d_\\d+"});
      outer.addChild(set);
    }
  }  // namespace ILD_FCCee_v01

  namespace ILD_FCCee_v02 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder) {
      auto& outer = root.addCylinderContainer(detName, AxisR);

      Blueprints::addCylindricalBeampipe(outer);
      auto vtxBarrel = Blueprints::makeDoubleLayerBarrel(builder);
      auto vertex    = Blueprints::attachEndcaps(builder, std::move(vtxBarrel), Blueprints::UngroupedVertexSpec);

      auto innerTracker =
          Blueprints::makeNestedInnerTracker(builder, std::move(vertex), Blueprints::UngroupedNestedInnerTrackerSpec);
      outer.addChild(innerTracker);
    }
  }  // namespace ILD_FCCee_v02

  namespace CLD_o2_v07 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder) {
      auto& outer = root.addCylinderContainer(detName, AxisR);
      Blueprints::addCylindricalBeampipe(outer);
      auto vtxBarrel = Blueprints::makeDoubleLayerBarrel(builder);
      auto vertex    = Blueprints::attachEndcaps(builder, std::move(vtxBarrel), Blueprints::UngroupedVertexSpec);

      auto innerTracker =
          Blueprints::makeNestedInnerTracker(builder, std::move(vertex), Blueprints::UngroupedNestedInnerTrackerSpec);
      outer.addChild(innerTracker);

      auto outerTracker =
          Blueprints::makeRegularTracker(builder, Blueprints::UngroupedOuterTrackerSpec, "OuterTracker");
      outer.addChild(outerTracker);
    }
  }  // namespace CLD_o2_v07
}  // namespace FCCee

namespace LUXE {
  namespace LUXE_v0 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder) {
      auto& tracker = root.addCuboidContainer(detName, AxisZ);
      auto  envelope =
          Acts::ExtentEnvelope{}.set(AxisZ, {0.4_mm, 0.4_mm}).set(AxisX, {0.4_mm, 0.4_mm}).set(AxisY, {0.4_mm, 0.4_mm});

      builder.layers()
          .planar()
          .setSensorAxes("XYZ")
          .setLayerFilter("layer\\d")
          .setContainer("Tracker")
          .setEnvelope(envelope)
          .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::Gap)
          .addTo(tracker);
    }
  }  // namespace LUXE_v0
}  // namespace LUXE
