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

    auto barrelEnvelope = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {1_mm, 1_mm});
    return builder.layersFromSensors()
        .barrel()
        .setEnvelope(barrelEnvelope)
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
    std::string    barrelContainer;  ///< Name of the DetElement containing the barrel
    AxisDefinition barrelAxes;       ///< The axes directions for the barrel sensors
    std::regex     barrelFilter;     ///< The layer pattern to filter out barrel layers
    std::string    endcapContainer;  ///< Name of the DetElement containing the endcaps
    AxisDefinition endcapAxes;       ///< The axes directions for the endcap sensors
    std::regex     endcapPosFilter;  ///< The layer pattern to filter out positive endcap layers
    std::regex     endcapNegFilter;  ///< The layer pattern to filter out negative endcap layers
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
    auto envelope = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {1_mm, 1_mm});
    return builder.layers()
        .barrel()
        .setSensorAxes(spec.barrelAxes)
        .setLayerFilter(spec.barrelFilter)
        .setContainer(spec.barrelContainer)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .onLayer(unsetXYCoG)
        .build();
  }

  /// Attach endcaps to an existing barrel node to form a complete cylindrical
  /// detector node stacked along the z-axis.
  ///
  /// The layers must already be organised into dedicated DetElements so that
  /// @c builder.layers() can pick them up directly via the filter patterns in
  /// @p spec.
  ///
  /// @param builder The Blueprint builder that drives the construction
  /// @param barrel  The barrel blueprint node to attach the endcaps to
  /// @param spec    Endcap configuration (container name, axes, pos/neg filters);
  ///                barrel fields of the spec are ignored
  std::shared_ptr<CylinderContainerBlueprintNode> attachEndcaps(ActsPlugins::DD4hep::BlueprintBuilder&    builder,
                                                                std::shared_ptr<ContainerBlueprintNode>&& barrel,
                                                                const TrackerSpec& spec = GroupedVertexSpec) {
    auto node = std::make_shared<Acts::Experimental::CylinderContainerBlueprintNode>("Vertex", AxisZ);
    node->addChild(barrel);

    // We use an Endcap envelope with smaller z-padding to accomodate for the double layer structure
    auto envelope = Acts::ExtentEnvelope{}.set(AxisZ, {1_mm, 1_mm}).set(AxisR, {5_mm, 5_mm});
    builder.layers()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainer(spec.endcapContainer)
        .setLayerFilter(spec.endcapPosFilter)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*node);

    builder.layers()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainer(spec.endcapContainer)
        .setLayerFilter(spec.endcapNegFilter)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*node);

    return node;
  }

  /// Attach endcaps to an existing barrel node to form a complete cylindrical
  /// detector node stacked along the z-axis.
  ///
  /// Use this variant when the individual sensors have not been placed into
  /// dedicated layer DetElements. The function collects all matching sensor
  /// DetElements and groups them into layers internally via @c layersFromSensors.
  ///
  /// @param builder       The Blueprint builder that drives the construction
  /// @param barrel        The barrel blueprint node to attach the endcaps to
  /// @param spec          Endcap configuration (container name, axes, pos/neg filters);
  ///                      barrel fields of the spec are ignored
  /// @param containerName Name of the resulting top-level cylinder container node
  std::shared_ptr<CylinderContainerBlueprintNode> attachUngroupedEndcaps(
      ActsPlugins::DD4hep::BlueprintBuilder& builder, std::shared_ptr<ContainerBlueprintNode>&& barrel,
      const TrackerSpec& spec = UngroupedVertexSpec, const std::string& containerName = "Vertex") {
    auto node = std::make_shared<Acts::Experimental::CylinderContainerBlueprintNode>(containerName, AxisZ);
    node->addChild(barrel);

    auto envelope = Acts::ExtentEnvelope{}.set(AxisZ, {0.5_mm, 0.5_mm}).set(AxisR, {5_mm, 5_mm});

    const auto endcapDetElem = builder.findDetElementByName(spec.endcapContainer);

    const auto posEndcapDetElems = builder.findDetElementByNamePattern(endcapDetElem.value(), spec.endcapPosFilter);
    const auto posLayerGrouper =
        makeLayerGrouper(spec.endcapPosFilter, fmt::format("{}|doubleLayer_pos", spec.endcapContainer),
                         [](const auto& m) { return std::stoi(m) / 2; });
    builder.layersFromSensors()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainerName(spec.endcapContainer)
        .groupBy(posLayerGrouper)
        .setSensors(std::move(posEndcapDetElems))
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*node);

    const auto negEndcapDetElems = builder.findDetElementByNamePattern(endcapDetElem.value(), spec.endcapNegFilter);
    const auto negLayerGrouper =
        makeLayerGrouper(spec.endcapNegFilter, fmt::format("{}|doubleLayer_neg", spec.endcapContainer),
                         [](const auto& m) { return std::stoi(m) / 2; });

    builder.layersFromSensors()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainerName(spec.endcapContainer)
        .setSensors(std::move(negEndcapDetElems))
        .groupBy(negLayerGrouper)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*node);

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
  };

  const auto InnerTrackerSpec = TrackerSpec{
      .barrelContainer = "InnerTrackerBarrel",
      .barrelAxes      = "XYZ",
      .barrelFilter    = std::regex{"layer\\d"},
      .endcapContainer = "InnerTrackerEndcap",
      .endcapAxes      = "YXZ",
      .endcapPosFilter = std::regex{"layer_pos(\\d)"},
      .endcapNegFilter = std::regex{"layer_neg(\\d)"},
  };

  const auto UngroupedInnerTrackerSpec = TrackerSpec{
      .barrelContainer = "InnerTrackerBarrel",
      .barrelAxes      = "XYZ",
      .barrelFilter    = std::regex{"layer\\d"},
      .endcapContainer = "InnerTrackerEndcap",
      .endcapAxes      = "YXZ",
      .endcapPosFilter = std::regex{"layer(\\d+)_module\\d+_sensor\\d+_pos"},
      .endcapNegFilter = std::regex{"layer(\\d+)_module\\d+_sensor\\d+_neg"},
  };

  /// Make the Acts volumes for a regular tracker consisting of a barrel and two
  /// endcaps that can be cleanly stacked along the z-axis without nesting.
  ///
  /// This is the simple case where all endcap layers fit within the z-extent of
  /// the barrel, i.e. no endcap layer protrudes into the radial envelope of the
  /// barrel layers. The barrel and both endcaps are stacked along z inside a
  /// single container node.
  ///
  /// @param builder     The Blueprint builder that drives the construction
  /// @param spec        The configuration spec defining the barrel and endcap
  ///                    container names, sensor axes, and layer filters
  /// @param trackerName The name of the resulting top-level tracker node
  ///
  /// @returns The tracker blueprint node
  std::shared_ptr<CylinderContainerBlueprintNode> makeRegularTracker(ActsPlugins::DD4hep::BlueprintBuilder& builder,
                                                                     const TrackerSpec&                     spec,
                                                                     const std::string& trackerName) {
    auto envelope = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {5_mm, 5_mm});
    auto tracker  = std::make_shared<CylinderContainerBlueprintNode>(trackerName, AxisZ);

    builder.layers()
        .barrel()
        .setSensorAxes(spec.barrelAxes)
        .setLayerFilter(spec.barrelFilter)
        .setContainer(spec.barrelContainer)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*tracker);
    builder.layers()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainer(spec.endcapContainer)
        .setLayerFilter(spec.endcapNegFilter)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*tracker);
    builder.layers()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainer(spec.endcapContainer)
        .setLayerFilter(spec.endcapPosFilter)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*tracker);

    return tracker;
  }

  /// Make the Acts volumes for a regular tracker consisting of a barrel and two
  /// endcaps that can be cleanly stacked along the z-axis without nesting. In
  /// this case the sensors are not placed into DetElements and have to be
  /// grouped first.
  ///
  /// This is the simple case where all endcap layers fit within the z-extent of
  /// the barrel, i.e. no endcap layer protrudes into the radial envelope of the
  /// barrel layers. The barrel and both endcaps are stacked along z inside a
  /// single container node.
  ///
  /// @param builder     The Blueprint builder that drives the construction
  /// @param spec        The configuration spec defining the barrel and endcap
  ///                    container names, sensor axes, and layer filters
  /// @param trackerName The name of the resulting top-level tracker node
  ///
  /// @returns The tracker blueprint node
  std::shared_ptr<CylinderContainerBlueprintNode> makeRegularTrackerUngroupedEndcap(
      ActsPlugins::DD4hep::BlueprintBuilder& builder, const TrackerSpec& spec, const std::string& trackerName) {
    auto envelope = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {5_mm, 5_mm});
    auto tracker  = std::make_shared<CylinderContainerBlueprintNode>(trackerName, AxisZ);

    // Barrel can just be done normally
    builder.layers()
        .barrel()
        .setSensorAxes(spec.barrelAxes)
        .setLayerFilter(spec.barrelFilter)
        .setContainer(spec.barrelContainer)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*tracker);

    const auto endcapDetElem = builder.findDetElementByName(spec.endcapContainer);

    const auto posEndcapDetElems = builder.findDetElementByNamePattern(endcapDetElem.value(), spec.endcapPosFilter);
    const auto posLayerGrouper =
        makeLayerGrouper(spec.endcapPosFilter, fmt::format("{}|layer_pos", spec.endcapContainer));
    builder.layersFromSensors()
        .endcap()
        .setSensors(std::move(posEndcapDetElems))
        .groupBy(posLayerGrouper)
        .setSensorAxes(spec.endcapAxes)
        .setContainerName(spec.endcapContainer)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*tracker);

    const auto negEndcapDetElems = builder.findDetElementByNamePattern(endcapDetElem.value(), spec.endcapNegFilter);
    const auto negLayerGrouper =
        makeLayerGrouper(spec.endcapNegFilter, fmt::format("{}|layer_neg", spec.endcapContainer));
    builder.layersFromSensors()
        .endcap()
        .setSensors(std::move(negEndcapDetElems))
        .groupBy(negLayerGrouper)
        .setSensorAxes(spec.endcapAxes)
        .setContainerName(spec.endcapContainer)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*tracker);

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
    auto envelope         = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {5_mm, 5_mm});
    auto innerInnerBarrel = builder.layers()
                                .barrel()
                                .setSensorAxes(spec.barrelAxes)
                                .setLayerFilter(spec.barrelInnerFilter)
                                .setContainer(spec.barrelContainer)
                                .setEnvelope(envelope)
                                .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                .onLayer(Blueprints::unsetXYCoG)
                                .build();
    innerInnerBarrel->addChild(vertex);

    auto innerInnerTracker = std::make_shared<CylinderContainerBlueprintNode>("InnerInnerTracker", AxisZ);
    innerInnerTracker->addChild(innerInnerBarrel);
    builder.layers()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainer(spec.endcapContainer)
        .setLayerFilter(spec.endcapPosInnerFilter)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*innerInnerTracker);
    builder.layers()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainer(spec.endcapContainer)
        .setLayerFilter(spec.endcapNegInnerFilter)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*innerInnerTracker);

    auto innerTracker = std::make_shared<CylinderContainerBlueprintNode>("InnerTracker", AxisZ);
    innerTracker->addCylinderContainer("InnerTrackerBarrel", AxisR, [&](auto& innerBarrel) {
      innerBarrel.addChild(innerInnerTracker);
      builder.layers()
          .barrel()
          .setSensorAxes(spec.barrelAxes)
          .setContainer(spec.barrelContainer)
          .setLayerFilter(spec.barrelOuterFilter)
          .setEnvelope(envelope)
          .onLayer(Blueprints::unsetXYCoG)
          .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
          .addTo(innerBarrel);
    });
    // Then add the (rest of the) two endcaps
    builder.layers()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainer(spec.endcapContainer)
        .setLayerFilter(spec.endcapPosOuterFilter)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*innerTracker);
    builder.layers()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainer(spec.endcapContainer)
        .setLayerFilter(spec.endcapNegOuterFilter)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*innerTracker);

    return innerTracker;
  }

  /// Make a nested inner tracker that encloses the vertex.
  ///
  /// This version uses a DD4hep detector geometry where the individual layers
  /// have not been put into dedicated DetElements. Instead this conversion will
  /// pick up all DetElements of the sensors and group them internally.
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
  ///
  /// @param builder  The Blueprint builder that drives the construction
  /// @param vertex   The vertex detector blueprint node
  /// @param spec     The spec for defining how the nesting is done specifically
  ///                 for this detector
  ///
  /// @returns The inner tracker blueprint node
  std::shared_ptr<CylinderContainerBlueprintNode> makeNestedInnerTrackerUngroupedEndcaps(
      ActsPlugins::DD4hep::BlueprintBuilder& builder, std::shared_ptr<CylinderContainerBlueprintNode>&& vertex,
      const NestedInnerTrackerSpec& spec = UngroupedNestedInnerTrackerSpec) {
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
    auto envelope         = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {5_mm, 5_mm});
    auto innerInnerBarrel = builder.layers()
                                .barrel()
                                .setSensorAxes(spec.barrelAxes)
                                .setLayerFilter(spec.barrelInnerFilter)
                                .setContainer(spec.barrelContainer)
                                .setEnvelope(envelope)
                                .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
                                .onLayer(Blueprints::unsetXYCoG)
                                .build();
    innerInnerBarrel->addChild(vertex);

    auto innerInnerTracker = std::make_shared<CylinderContainerBlueprintNode>("InnerInnerTracker", AxisZ);
    innerInnerTracker->addChild(innerInnerBarrel);

    auto endcapDetElem     = builder.findDetElementByName(spec.endcapContainer);
    auto posEndcapDetElems = builder.findDetElementByNamePattern(endcapDetElem.value(), spec.endcapPosInnerFilter);
    auto posLayerGrouper =
        makeLayerGrouper(spec.endcapPosInnerFilter, fmt::format("{}|layer_pos", spec.endcapContainer));
    builder.layersFromSensors()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainerName(spec.endcapContainer)
        .setSensors(std::move(posEndcapDetElems))
        .groupBy(posLayerGrouper)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*innerInnerTracker);

    auto negEndcapDetElems = builder.findDetElementByNamePattern(endcapDetElem.value(), spec.endcapNegInnerFilter);
    auto negLayerGrouper =
        makeLayerGrouper(spec.endcapNegInnerFilter, fmt::format("{}|layer_neg", spec.endcapContainer));

    builder.layersFromSensors()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainerName(spec.endcapContainer)
        .setSensors(std::move(negEndcapDetElems))
        .groupBy(negLayerGrouper)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*innerInnerTracker);

    auto innerTracker = std::make_shared<CylinderContainerBlueprintNode>("InnerTracker", AxisZ);
    innerTracker->addCylinderContainer("InnerTrackerBarrel", AxisR, [&](auto& innerBarrel) {
      innerBarrel.addChild(innerInnerTracker);
      builder.layers()
          .barrel()
          .setSensorAxes(spec.barrelAxes)
          .setContainer(spec.barrelContainer)
          .setLayerFilter(spec.barrelOuterFilter)
          .setEnvelope(envelope)
          .onLayer(Blueprints::unsetXYCoG)
          .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
          .addTo(innerBarrel);
    });

    // Then add the (rest of the) two endcaps
    posEndcapDetElems = builder.findDetElementByNamePattern(endcapDetElem.value(), spec.endcapPosOuterFilter);
    posLayerGrouper   = makeLayerGrouper(spec.endcapPosOuterFilter, fmt::format("{}|layer_pos", spec.endcapContainer));
    builder.layersFromSensors()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainerName(spec.endcapContainer)
        .setSensors(std::move(posEndcapDetElems))
        .groupBy(posLayerGrouper)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*innerTracker);

    negEndcapDetElems = builder.findDetElementByNamePattern(endcapDetElem.value(), spec.endcapNegOuterFilter);
    negLayerGrouper   = makeLayerGrouper(spec.endcapNegOuterFilter, fmt::format("{}|layer_neg", spec.endcapContainer));
    builder.layersFromSensors()
        .endcap()
        .setSensorAxes(spec.endcapAxes)
        .setContainerName(spec.endcapContainer)
        .setSensors(std::move(negEndcapDetElems))
        .groupBy(negLayerGrouper)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .addTo(*innerTracker);

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
      auto barrelEnvelope = Acts::ExtentEnvelope{}.set(AxisZ, {5_mm, 5_mm}).set(AxisR, {0.4_mm, 0.4_mm});
      auto vertexBarrel   = builder.layers()
                              .barrel()
                              .setSensorAxes("ZYX")
                              .setLayerFilter("layer_\\d")
                              .setContainer("VertexBarrel")
                              .setEnvelope(barrelEnvelope)
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
      auto vertex    = Blueprints::attachUngroupedEndcaps(builder, std::move(vtxBarrel));

      auto innerTrackerBarrel = Blueprints::makeBarrel(builder, Blueprints::UngroupedInnerTrackerSpec);
      innerTrackerBarrel->addChild(vertex);

      auto innerTrackerEndcap = Blueprints::attachUngroupedEndcaps(
          builder, std::move(innerTrackerBarrel), Blueprints::UngroupedInnerTrackerSpec, "InnerTrackerEndcap");
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
      auto vertex    = Blueprints::attachUngroupedEndcaps(builder, std::move(vtxBarrel));

      auto innerTracker = Blueprints::makeNestedInnerTrackerUngroupedEndcaps(builder, std::move(vertex));
      outer.addChild(innerTracker);
    }
  }  // namespace ILD_FCCee_v02

  namespace CLD_o2_v07 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder) {
      auto& outer = root.addCylinderContainer(detName, AxisR);
      Blueprints::addCylindricalBeampipe(outer);
      auto vtxBarrel = Blueprints::makeDoubleLayerBarrel(builder);
      auto vertex    = Blueprints::attachUngroupedEndcaps(builder, std::move(vtxBarrel));

      auto innerTracker = Blueprints::makeNestedInnerTrackerUngroupedEndcaps(builder, std::move(vertex));
      outer.addChild(innerTracker);

      auto outerTracker =
          Blueprints::makeRegularTrackerUngroupedEndcap(builder, Blueprints::UngroupedOuterTrackerSpec, "OuterTracker");
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
