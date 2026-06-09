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

using Acts::Experimental::BlueprintNode;
using Acts::Experimental::ContainerBlueprintNode;
using Acts::Experimental::CylinderContainerBlueprintNode;
using Acts::Experimental::LayerBlueprintNode;

using AxisDefinition = ActsPlugins::DD4hep::BlueprintBuilder::AxisDefinition;
using LayerGrouper   = Acts::Experimental::SensorLayerAssembler<ActsPlugins::DD4hep::DD4hepBackend>::LayerGrouper;

using namespace Acts::UnitLiterals;
using enum Acts::AxisDirection;

/// @namespace Blueprints
///
/// This namespace contains some commonly used functionality for populating
/// detector concept blueprints below. Generally the functionality is
/// implemented according to the needs of the geometries that we try to convert.
/// Configuration is placed into "Spec" structs that usually contain
/// - a container name - this is the name of the DD4hep DetElement for a given
///   subdetector)
/// - a filter (regex) - this is the regex that will be used to filter out
///   (layer) DetElements from the subdetector DetElement containing them. These
///   DetElements will be converted to Acts::Surfaces and will be part of the
///   converted geometry
/// - an axis definition - these define the axes directions to use for the
///   transformation from global to local (or back) coordinate systems. (These
///   have been largely identified by trial & error so far.
/// - a layout - this is used to differentiate between DD4hep geometries where
///   the sensor level DetElements have not been grouped into layer DetElements
///   (see e.g. https://github.com/key4hep/k4geo/issues/548 and
///   https://github.com/key4hep/k4geo/issues/550). If the option Ungrouped is
///   used the grouping of these DetElements into layers is done via the
///   functionality to do this provided by Acts
///
/// These properties are usually available once for the barrel config and
/// (partially split) for the endcap configuration of a given detector. It is
/// also possible that it is split even more for (sub)detectors that are nested
/// inside each other.
///
/// In some cases an Ungroued layout is chosen even thoug the DD4hep geometry
/// has layer DetElements. The most common use case for this is to group two
/// layers in the DD4hep geometry into a double layer on the Acts side (e.g.
/// because otherwise the two enclosing cylinders would overlap).
namespace Blueprints {
  /// Commonly used envelopes for blueprint construction below
  const auto kTrackerEnvelope      = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {5_mm, 5_mm});
  const auto kBarrelEnvelope       = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {1_mm, 1_mm});
  const auto kTightBarrelEnvelope  = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {0.4_mm, 0.4_mm});
  const auto kVertexEndcapEnvelope = Acts::ExtentEnvelope{}.set(AxisZ, {1_mm, 1_mm}).set(AxisR, {5_mm, 5_mm});
  const auto kUngroupedVertexEndcapEnvelope =
      Acts::ExtentEnvelope{}.set(AxisZ, {0.5_mm, 0.5_mm}).set(AxisR, {5_mm, 5_mm});

  /// Enum denoting whether a (subdetector) geometry has grouping (layer)
  /// DetElements or whether the grouping into layers has to be done during the
  /// conversoin.
  enum class Layout { Grouped, Ungrouped };

  /// A simple struct to contain the configuration for building a regular
  /// detector where the barrel and the endcaps can be cleanly stacked along the
  /// z-axis
  struct TrackerSpec {
    std::string    barrelContainer;                 ///< Name of the DetElement containing the barrel
    AxisDefinition barrelAxes;                      ///< The axes directions for the barrel sensors
    std::regex     barrelFilter;                    ///< The layer pattern to filter out barrel layers
    std::string    endcapContainer;                 ///< Name of the DetElement containing the endcaps
    AxisDefinition endcapAxes;                      ///< The axes directions for the endcap sensors
    std::regex     endcapPosFilter;                 ///< The layer pattern to filter out positive endcap layers
    std::regex     endcapNegFilter;                 ///< The layer pattern to filter out negative endcap layers
    Layout         endcapLayout = Layout::Grouped;  ///< The layout for endcap construction
    Layout         barrelLayout = Layout::Grouped;  ///< The layout for barrel construction
  };

  const auto DoubleBarrelLayerVertexSpec = TrackerSpec{
      .barrelContainer = "VertexBarrel",
      .barrelAxes      = "ZYX",
      .barrelFilter    = std::regex{"layer_\\d+"},
      .endcapContainer = "VertexEndcap",
      .endcapAxes      = "XZY",
      .endcapPosFilter = std::regex{"layer_pos\\d+"},
      .endcapNegFilter = std::regex{"layer_neg\\d+"},
  };

  const auto UngroupedDoubleBarrelLayerVertexSpec = TrackerSpec{
      .barrelContainer = "VertexBarrel",
      .barrelAxes      = "ZYX",
      .barrelFilter    = std::regex{"VertexBarrel_layer(\\d)_ladder\\d+"},
      .endcapContainer = "VertexEndcap",
      .endcapAxes      = "XZY",
      .endcapPosFilter = std::regex{"layer(\\d+)_module\\d+_sensor\\d+_pos"},
      .endcapNegFilter = std::regex{"layer(\\d+)_module\\d+_sensor\\d+_neg"},
      .endcapLayout    = Layout::Ungrouped,
      .barrelLayout    = Layout::Ungrouped,
  };

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
      .endcapLayout    = Layout::Ungrouped,
  };

  const auto UngroupedInnerTrackerSpec = TrackerSpec{
      .barrelContainer = "InnerTrackerBarrel",
      .barrelAxes      = "XYZ",
      .barrelFilter    = std::regex{"layer\\d"},
      .endcapContainer = "InnerTrackerEndcap",
      .endcapAxes      = "YXZ",
      .endcapPosFilter = std::regex{"layer(\\d+)_module\\d+_sensor\\d+_pos"},
      .endcapNegFilter = std::regex{"layer(\\d+)_module\\d+_sensor\\d+_neg"},
      .endcapLayout    = Layout::Ungrouped,
  };

  const auto SETSpec = TrackerSpec{
      .barrelContainer = "SET",
      .barrelAxes      = "XYZ",
      .barrelFilter    = std::regex{"set_ladder_(\\d)_\\d_\\d+"},
      .endcapContainer = "<unused>",
      .endcapAxes      = "XYZ",
      .endcapPosFilter = {},
      .endcapNegFilter = {},
      .endcapLayout    = Layout::Grouped,
      .barrelLayout    = Layout::Ungrouped,
  };

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
      .layout               = Layout::Ungrouped,
  };

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
  // Return type is the base BlueprintNodePtr (not the derived LayerBlueprintNode)
  // so the callback satisfies acts' OnLayerReturnsNode concept, which requires
  // the result to be exactly std::shared_ptr<BlueprintNode>. `return layer;`
  // upcasts implicitly.
  std::shared_ptr<BlueprintNode> unsetXYCoG(const std::optional<dd4hep::DetElement>&,
                                            std::shared_ptr<LayerBlueprintNode> layer) {
    layer->setUseCenterOfGravity(false, false, true);
    return layer;
  }

  /// Transform functions that are commonly used for the @c makeLayerGrouper
  /// below
  const std::string& identityKey(const std::string& m) { return m; }
  int                doubleLayerKey(const std::string& m) { return std::stoi(m) / 2; }

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
  ///
  /// @returns a closure (lambda) object that can be passed to groupBy
  template <typename TransformF = decltype(identityKey)>
  LayerGrouper makeLayerGrouper(std::regex groupRgx, std::string labelBase, TransformF transformMatch = identityKey) {
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

  /// Add both endcap sides (pos and neg) to @p parent, dispatching on @p layout.
  ///
  /// @param builder   Blueprint builder driving the construction
  /// @param parent    Node to attach the endcap sides to
  /// @param container Name of the DetElement containing the endcap layers/sensors
  /// @param posFilter Regex selecting the positive-side layers or sensors
  /// @param negFilter Regex selecting the negative-side layers or sensors
  /// @param axes      Sensor coordinate axes for the endcap sides
  /// @param layout    Grouped or Ungrouped dispatch
  /// @param envelope  Extent envelope applied to the resulting volumes
  /// @param posLabel  Label prefix for the positive side (Ungrouped path only)
  /// @param negLabel  Label prefix for the negative side (Ungrouped path only)
  /// @param keyXform  Transform applied to capture group 1 (Ungrouped path only)
  template <typename MatchTransformF = decltype(identityKey)>
  void addBothEndcapSides(ActsPlugins::DD4hep::BlueprintBuilder& builder, Acts::Experimental::BlueprintNode& parent,
                          const std::string& container, const std::regex& posFilter, const std::regex& negFilter,
                          AxisDefinition axes, Layout layout, const Acts::ExtentEnvelope& envelope,
                          const std::string& posLabel = "", const std::string& negLabel = "",
                          MatchTransformF keyXform = identityKey) {
    if (layout == Layout::Grouped) {
      addGroupedEndcapSide(builder, parent, container, posFilter, axes, envelope);
      addGroupedEndcapSide(builder, parent, container, negFilter, axes, envelope);
    } else {
      addUngroupedEndcapSide(builder, parent, container, posFilter, axes, posLabel, envelope, keyXform);
      addUngroupedEndcapSide(builder, parent, container, negFilter, axes, negLabel, envelope, keyXform);
    }
  }

  /// Build a grouped barrel blueprint node from a raw builder chain.
  ///
  /// Owns the single repeated builder chain for grouped barrel construction.
  /// All five grouped barrel call sites delegate here.
  ///
  /// @param builder   Blueprint builder driving the construction
  /// @param container Name of the DetElement containing the barrel layers
  /// @param filter    Regex selecting the layer DetElements
  /// @param axes      Sensor coordinate axes for the barrel
  /// @param envelope  Extent envelope applied to the resulting volume
  ///
  /// @returns The barrel blueprint node
  std::shared_ptr<ContainerBlueprintNode> makeGroupedBarrel(ActsPlugins::DD4hep::BlueprintBuilder& builder,
                                                            const std::string& container, const std::regex& filter,
                                                            AxisDefinition axes, const Acts::ExtentEnvelope& envelope) {
    return builder.layers()
        .barrel()
        .setSensorAxes(std::move(axes))
        .setLayerFilter(filter)
        .setContainer(container)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .onLayer(unsetXYCoG)
        .build();
  }

  /// Make a barrel blueprint node for a generic cylindrical detector.
  ///
  /// Uses the pre-grouped layer DetElements directly (via @c layers()), so
  /// the barrel layers must already be organised into dedicated DetElements
  /// matching @p spec.barrelFilter inside @p spec.barrelContainer.
  ///
  /// @param builder  The Blueprint builder that drives the construction
  /// @param spec     Configuration spec (barrelContainer, barrelAxes,
  ///                 barrelFilter used; endcap fields ignored)
  /// @param envelope Extent envelope applied to the resulting volume
  ///
  /// @returns The barrel blueprint node
  template <typename MatchTransformF = decltype(identityKey)>
  std::shared_ptr<ContainerBlueprintNode> makeBarrel(ActsPlugins::DD4hep::BlueprintBuilder& builder,
                                                     const TrackerSpec&                     spec,
                                                     const Acts::ExtentEnvelope&            envelope = kBarrelEnvelope,
                                                     MatchTransformF                        keyXform = identityKey) {
    if (spec.barrelLayout == Layout::Grouped) {
      return makeGroupedBarrel(builder, spec.barrelContainer, spec.barrelFilter, spec.barrelAxes, envelope);
    }

    const auto barrelDetElem    = builder.findDetElementByName(spec.barrelContainer);
    const auto barrelLayerElems = builder.findDetElementByNamePattern(barrelDetElem.value(), spec.barrelFilter);
    const auto doubleLayerName =
        makeLayerGrouper(spec.barrelFilter, fmt::format("{}|doubleLayer", spec.barrelContainer), keyXform);

    return builder.layersFromSensors()
        .barrel()
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .setSensorAxes(spec.barrelAxes)
        .setSensors(std::move(barrelLayerElems))
        .groupBy(doubleLayerName)
        .setContainerName(spec.barrelContainer)
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
      const TrackerSpec& spec, const std::string& containerName,
      std::function<std::string(const std::string&)> keyXform = [](const std::string& m) {
        return std::to_string(std::stoi(m) / 2);
      }) {
    auto node = std::make_shared<CylinderContainerBlueprintNode>(containerName, AxisZ);
    node->addChild(barrel);

    if (spec.endcapLayout == Layout::Grouped) {
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
    tracker->addChild(makeBarrel(builder, spec, kTrackerEnvelope));
    addBothEndcapSides(builder, *tracker, spec.endcapContainer, spec.endcapPosFilter, spec.endcapNegFilter,
                       spec.endcapAxes, spec.endcapLayout, kTrackerEnvelope,
                       fmt::format("{}|layer_pos", spec.endcapContainer),
                       fmt::format("{}|layer_neg", spec.endcapContainer));
    return tracker;
  }

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
    auto innerInnerBarrel =
        makeGroupedBarrel(builder, spec.barrelContainer, spec.barrelInnerFilter, spec.barrelAxes, kTrackerEnvelope);
    innerInnerBarrel->addChild(vertex);

    auto innerInnerTracker = std::make_shared<CylinderContainerBlueprintNode>("InnerInnerTracker", AxisZ);
    innerInnerTracker->addChild(innerInnerBarrel);

    addBothEndcapSides(builder, *innerInnerTracker, spec.endcapContainer, spec.endcapPosInnerFilter,
                       spec.endcapNegInnerFilter, spec.endcapAxes, spec.layout, kTrackerEnvelope,
                       fmt::format("{}|layer_pos", spec.endcapContainer),
                       fmt::format("{}|layer_neg", spec.endcapContainer));

    auto innerTracker = std::make_shared<CylinderContainerBlueprintNode>("InnerTracker", AxisZ);
    innerTracker->addCylinderContainer("InnerTrackerBarrel", AxisR, [&](auto& innerBarrel) {
      innerBarrel.addChild(innerInnerTracker);
      auto outerBarrel =
          makeGroupedBarrel(builder, spec.barrelContainer, spec.barrelOuterFilter, spec.barrelAxes, kTrackerEnvelope);
      innerBarrel.addChild(outerBarrel);
    });

    addBothEndcapSides(builder, *innerTracker, spec.endcapContainer, spec.endcapPosOuterFilter,
                       spec.endcapNegOuterFilter, spec.endcapAxes, spec.layout, kTrackerEnvelope,
                       fmt::format("{}|layer_pos", spec.endcapContainer),
                       fmt::format("{}|layer_neg", spec.endcapContainer));

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
      auto vertexBarrel = Blueprints::makeGroupedBarrel(builder, "VertexBarrel", std::regex{"layer_\\d"}, "ZYX",
                                                        Blueprints::kTightBarrelEnvelope);
      auto vertex = Blueprints::attachEndcaps(builder, std::move(vertexBarrel), Blueprints::DoubleBarrelLayerVertexSpec,
                                              "Vertex");

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

      auto vtxBarrel = Blueprints::makeBarrel(builder, Blueprints::UngroupedDoubleBarrelLayerVertexSpec,
                                              Blueprints::kBarrelEnvelope, Blueprints::doubleLayerKey);
      auto vertex    = Blueprints::attachEndcaps(builder, std::move(vtxBarrel),
                                                 Blueprints::UngroupedDoubleBarrelLayerVertexSpec, "Vertex");

      auto innerTrackerBarrel = Blueprints::makeBarrel(builder, Blueprints::UngroupedInnerTrackerSpec);
      innerTrackerBarrel->addChild(vertex);

      auto innerTrackerEndcap = Blueprints::attachEndcaps(builder, std::move(innerTrackerBarrel),
                                                          Blueprints::UngroupedInnerTrackerSpec, "InnerTrackerEndcap");
      outer.addChild(innerTrackerEndcap);

      // TODO: this is not yet properly working only part of the SET show up in
      // the exporte .obj geometry. This usually indicates some issues with the
      // AxisDirection, but that would mean that there are different
      // AxisDirections in play for the SET geometry
      auto set =
          Blueprints::makeBarrel(builder, Blueprints::SETSpec, Blueprints::kBarrelEnvelope, Blueprints::doubleLayerKey);
      outer.addChild(set);
    }
  }  // namespace ILD_FCCee_v01

  namespace ILD_FCCee_v02 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder) {
      auto& outer = root.addCylinderContainer(detName, AxisR);

      Blueprints::addCylindricalBeampipe(outer);
      auto vtxBarrel = Blueprints::makeBarrel(builder, Blueprints::UngroupedDoubleBarrelLayerVertexSpec,
                                              Blueprints::kBarrelEnvelope, Blueprints::doubleLayerKey);
      auto vertex    = Blueprints::attachEndcaps(builder, std::move(vtxBarrel),
                                                 Blueprints::UngroupedDoubleBarrelLayerVertexSpec, "Vertex");

      auto innerTracker =
          Blueprints::makeNestedInnerTracker(builder, std::move(vertex), Blueprints::UngroupedNestedInnerTrackerSpec);
      outer.addChild(innerTracker);

      // TODO: Add SET (see V01 for caveats)
    }
  }  // namespace ILD_FCCee_v02

  namespace CLD_o2_v07 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder) {
      auto& outer = root.addCylinderContainer(detName, AxisR);
      Blueprints::addCylindricalBeampipe(outer);
      auto vtxBarrel = Blueprints::makeBarrel(builder, Blueprints::UngroupedDoubleBarrelLayerVertexSpec,
                                              Blueprints::kBarrelEnvelope, Blueprints::doubleLayerKey);
      auto vertex    = Blueprints::attachEndcaps(builder, std::move(vtxBarrel),
                                                 Blueprints::UngroupedDoubleBarrelLayerVertexSpec, "Vertex");

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
