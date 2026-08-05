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

#include <Acts/Definitions/Algebra.hpp>
#include <Acts/Definitions/Units.hpp>
#include <Acts/Geometry/Blueprint.hpp>
#include <Acts/Geometry/BlueprintBuilder.hpp>
#include <Acts/Geometry/BlueprintNode.hpp>
#include <Acts/Geometry/ContainerBlueprintNode.hpp>
#include <Acts/Geometry/CuboidVolumeBounds.hpp>
#include <Acts/Geometry/CylinderVolumeBounds.hpp>
#include <Acts/Geometry/Extent.hpp>
#include <Acts/Geometry/LayerBlueprintNode.hpp>
#include <Acts/Geometry/MaterialDesignatorBlueprintNode.hpp>
#include <Acts/Geometry/NavigationPolicyFactory.hpp>
#include <Acts/Geometry/StaticBlueprintNode.hpp>
#include <Acts/Geometry/TrackingVolume.hpp>
#include <Acts/Geometry/VolumeAttachmentStrategy.hpp>
#include <Acts/Navigation/TryAllNavigationPolicy.hpp>
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/Utilities/AxisDefinitions.hpp>
#include <Acts/Utilities/ProtoAxis.hpp>
#include <ActsPlugins/DD4hep/BlueprintBuilder.hpp>
#include <ActsPlugins/Root/TGeoAxes.hpp>

#include <DD4hep/DD4hepUnits.h>
#include <DD4hep/Shapes.h>
#include <DD4hep/Volumes.h>

#include <TGeoTube.h>

#include <fmt/format.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <regex>
#include <stdexcept>
#include <utility>

using Acts::BlueprintNode;
using Acts::ContainerBlueprintNode;
using Acts::CylinderContainerBlueprintNode;
using Acts::LayerBlueprintNode;

using AxisDefinition  = ActsPlugins::DD4hep::BlueprintBuilder::AxisDefinition;
using LayerGrouper    = Acts::SensorLayerAssembler<ActsPlugins::DD4hep::DD4hepBackend>::LayerGrouper;
using LayerCustomizer = Acts::ElementLayerAssembler<ActsPlugins::DD4hep::DD4hepBackend>::LayerCustomizer;

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

  /// @name Material designation
  ///
  /// Acts projects the material of the full simulation geometry onto surfaces
  /// that are explicitly marked up while the blueprint tree is built, using
  /// @c Acts::MaterialDesignatorBlueprintNode. This replaces the markup JSON of
  /// the Gen1 (TGeo) workflow: the receiving surfaces are designated here, in
  /// code, instead of being addressed by geometry identifier.
  ///
  /// Two Acts rules constrain which faces may be designated:
  /// - a portal *shared* by two adjacent volumes (fused while stacking) may
  ///   carry material from only one of the two sides, and
  /// - a face that a container has to *merge* while stacking its children may
  ///   not carry material at all; construction aborts if it does.
  ///
  /// In a container stacked along r the children's cylinder faces are fused and
  /// their disc faces are merged; in a container stacked along z it is the
  /// other way around. The scheme below therefore designates, on every layer
  /// volume, only the face pointing *towards* the interaction point: the inner
  /// cylinder of a barrel layer, and the disc facing the IP for an endcap
  /// layer. That face is the portal the layer shares with its inward
  /// neighbour, so exactly one side of it is designated. The exception is a
  /// layer whose IP-facing face is the *extreme* face of its container rather
  /// than an interior one: that face becomes the container's own face and is
  /// merged higher up the tree -- see @c LayerMaterial::skipInnermost.
  /// @{

  /// Binning of the material projected onto a cylindrical face. The proto axes
  /// use auto-range equidistant binning, so only the bin counts matter here.
  const auto kCylinderMaterialBinning = std::pair{Acts::DirectedProtoAxis{AxisRPhi, Acts::AxisBoundaryType::Closed, 20},
                                                  Acts::DirectedProtoAxis{AxisZ, Acts::AxisBoundaryType::Bound, 20}};

  /// Binning of the material projected onto a disc face
  const auto kDiscMaterialBinning = std::pair{Acts::DirectedProtoAxis{AxisR, Acts::AxisBoundaryType::Bound, 50},
                                              Acts::DirectedProtoAxis{AxisPhi, Acts::AxisBoundaryType::Closed, 20}};

  /// Wrap @p child in a material designator marking @p face of the child's
  /// volume as a receiver for the projected material.
  ///
  /// The binning follows from the face: cylindrical faces are binned in
  /// (rphi, z), disc faces in (r, phi).
  ///
  /// @param child The node whose volume face should receive material
  /// @param name  Name of the designator node (used for debugging only)
  /// @param face  The face to designate
  ///
  /// @returns The designator node, to be added in place of @p child
  std::shared_ptr<BlueprintNode> withMaterial(std::shared_ptr<BlueprintNode> child, const std::string& name,
                                              Acts::CylinderVolumeBounds::Face face) {
    using enum Acts::CylinderVolumeBounds::Face;
    const auto& [loc0, loc1] =
        (face == InnerCylinder || face == OuterCylinder) ? kCylinderMaterialBinning : kDiscMaterialBinning;

    auto designator = std::make_shared<Acts::MaterialDesignatorBlueprintNode>(name);
    designator->configureFace(face, loc0, loc1);
    designator->addChild(std::move(child));
    return designator;
  }

  /// Material designation for the layer volumes of a (sub)detector
  struct LayerMaterial {
    /// Layers whose name matches this regex are left without material. Use it
    /// for a layer whose IP-facing face is the *extreme* face of its container
    /// rather than a portal shared with an inward neighbour, because that face
    /// is merged higher up the tree and would abort the construction.
    ///
    /// Note this is not simply "layer 0": whether the innermost layer's inner
    /// cylinder is the container extreme depends on what else sits in the same
    /// radial stack. In MAIA the outer-tracker and vertex barrels are plain
    /// layer stacks and do skip their innermost layer, while the inner-tracker
    /// barrel encloses the whole vertex detector, which makes its layer0 inner
    /// cylinder an interior (fused) portal that is safe to designate.
    ///
    /// The default (an empty regex) matches no layer name, so every layer is
    /// designated.
    std::regex skipInnermost{};
  };

  /// Build an `onLayer` customizer that designates @p face of every layer
  /// volume as a material receiver, on top of whatever @p customize does.
  ///
  /// Layers are matched against @c LayerMaterial::skipInnermost by their
  /// DD4hep DetElement name where one exists (the Grouped layouts), and by the
  /// blueprint node name otherwise (the Ungrouped layouts, where the layers are
  /// assembled from sensors and carry the group label from @c makeLayerGrouper).
  ///
  /// @param material  The designation config; if unset the returned customizer
  ///                  only forwards to @p customize
  /// @param face      The (IP-facing) face to designate
  /// @param label     Prefix for the designator node names
  /// @param customize Layer customization to apply first (e.g. @c unsetXYCoG)
  LayerCustomizer designateLayers(const std::optional<LayerMaterial>& material, Acts::CylinderVolumeBounds::Face face,
                                  const std::string& label, LayerCustomizer customize = {}) {
    return [=, customize = std::move(customize)](const std::optional<dd4hep::DetElement>& elem,
                                                 std::shared_ptr<LayerBlueprintNode>      layer) {
      const std::string layerName = elem.has_value() ? std::string{elem->name()} : layer->name();

      std::shared_ptr<BlueprintNode> node;
      if (customize) {
        node = customize(elem, std::move(layer));
      } else {
        node = std::move(layer);
      }

      if (!material.has_value() || std::regex_match(layerName, material->skipInnermost)) {
        return node;
      }
      return withMaterial(std::move(node), fmt::format("{}_{}_Material", label, layerName), face);
    };
  }

  /// @}

  /// Add a cylindrical beampipe to the passed node using the measures passed as arguments.
  ///
  /// We use this to enclose our actual beampipe because that is not a sipmle
  /// cylinder. However, we mainly need a surface / volume to attach the
  /// material of the beampipe, for which we use this cylindrical volume here.
  ///
  /// @param node The Blueprint container to which the beampipe should be added
  /// @param designateMaterial Whether the outer cylinder of the beampipe volume
  ///                          should receive material. This is the innermost
  ///                          portal of the whole tracker, shared all the way
  ///                          down to the innermost barrel layer, so it must be
  ///                          designated here and nowhere else.
  /// @param rMax The (initial) max radius of the cylinder
  /// @param halfZ the half-length in z of this cylinder
  void addCylindricalBeampipe(ContainerBlueprintNode& node, bool designateMaterial = false, double rMax = 10_mm,
                              double halfZ = 1000_mm) {
    std::shared_ptr<BlueprintNode> beampipe =
        std::make_shared<Acts::StaticBlueprintNode>(std::make_unique<Acts::TrackingVolume>(
            Acts::Transform3::Identity(), std::make_shared<Acts::CylinderVolumeBounds>(0_mm, rMax, halfZ), "Beampipe"));
    if (designateMaterial) {
      beampipe =
          withMaterial(std::move(beampipe), "Beampipe_Material", Acts::CylinderVolumeBounds::Face::OuterCylinder);
    }
    node.addChild(std::move(beampipe));

    // We want to pull the next volume in towards the beampipe to map material to
    // the correct places in the end. We need to ensure that the enclosing
    // cylinder contains the beampipe entirely.
    node.setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First);
  }

  /// Look up the inner radius of the solenoid coil, in Acts units.
  ///
  /// Read from the envelope shape of the @c Solenoid DetElement rather than
  /// hard-coded, because detectors sharing a blueprint can place their coil very
  /// differently (MAIA_v0 has it at 1500 mm, inside the calorimeter, while
  /// MuSIC_v2 has it at 2055 mm, outside).
  ///
  /// @param builder Blueprint builder driving the construction
  ///
  /// @returns The inner radius, or nothing if the detector has no @c Solenoid
  ///          DetElement or its envelope is not a tube
  std::optional<double> findSolenoidInnerRadius(ActsPlugins::DD4hep::BlueprintBuilder& builder) {
    const auto detElem = builder.findDetElementByName("Solenoid");
    if (!detElem.has_value() || !detElem->placement().isValid()) {
      return std::nullopt;
    }
    const auto* tube = dynamic_cast<const TGeoTube*>(detElem->volume().solid().ptr());
    if (tube == nullptr) {
      return std::nullopt;
    }
    // DD4hep native lengths are cm, Acts works in mm
    return tube->GetRmin() * (Acts::UnitConstants::cm / dd4hep::cm);
  }

  /// Add a cylindrical volume covering the solenoid to the passed (radial)
  /// container node.
  ///
  /// Where the solenoid sits between the tracker and the calorimeter, every
  /// extrapolation to the calorimeter face crosses its coil -- several hundred
  /// mm of vacuum tank and conductor. Without a volume of its own that material
  /// has nowhere sensible to go: the radial stack stretches the outer tracker
  /// volume all the way out to the calorimeter, so the outer tracker's own outer
  /// cylinder ends up hard against the calorimeter face and the coil is left
  /// straddling a single volume with no boundary anywhere inside it.
  ///
  /// Inserting this volume splits that region in two, so that the outer
  /// tracker's outer cylinder lands where the tracker actually ends (at the
  /// coil's inner face) and this volume's outer cylinder marks the coil's outer
  /// face. Designating one of them on each side brackets the coil, which lets
  /// the mapping distribute the coil material across its thickness instead of
  /// lumping all of it at one edge.
  ///
  /// Only @p rMin is physical: the volume is created with a nominal thickness
  /// and half-length, and the enclosing radial stack expands it to meet whatever
  /// comes next (the calorimeter barrel), the same way the beampipe volume above
  /// is handled.
  ///
  /// @param node The Blueprint container to which the solenoid should be added
  /// @param rMin Inner radius of the solenoid coil
  /// @param designateMaterial Whether the outer cylinder of this volume should
  ///                          receive material
  void addCylindricalSolenoid(ContainerBlueprintNode& node, double rMin, bool designateMaterial = false) {
    std::shared_ptr<BlueprintNode> solenoid =
        std::make_shared<Acts::StaticBlueprintNode>(std::make_unique<Acts::TrackingVolume>(
            Acts::Transform3::Identity(), std::make_shared<Acts::CylinderVolumeBounds>(rMin, rMin + 1_mm, 1000_mm),
            "Solenoid"));
    if (designateMaterial) {
      solenoid =
          withMaterial(std::move(solenoid), "Solenoid_Material", Acts::CylinderVolumeBounds::Face::OuterCylinder);
    }
    node.addChild(std::move(solenoid));
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
  /// @param onLayer   Optional per-layer customization, e.g. from @c designateLayers
  void addGroupedEndcapSide(ActsPlugins::DD4hep::BlueprintBuilder& builder, Acts::BlueprintNode& parent,
                            const std::string& container, const std::regex& filter, AxisDefinition axes,
                            const Acts::ExtentEnvelope& envelope, LayerCustomizer onLayer = {}) {
    builder.layers()
        .endcap()
        .setSensorAxes(std::move(axes))
        .setContainer(container)
        .setLayerFilter(filter)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .onLayer(std::move(onLayer))
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
  /// @param onLayer     Optional per-layer customization, e.g. from @c designateLayers
  void addUngroupedEndcapSide(
      ActsPlugins::DD4hep::BlueprintBuilder& builder, Acts::BlueprintNode& parent, const std::string& container,
      const std::regex& filter, AxisDefinition axes, const std::string& labelPrefix,
      const Acts::ExtentEnvelope&                    envelope,
      std::function<std::string(const std::string&)> keyXform = [](const std::string& m) { return m; },
      LayerCustomizer                                onLayer  = {}) {
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
        .onLayer(std::move(onLayer))
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
  /// @param material  If set, designate the IP-facing disc of every endcap
  ///                  layer as a material receiver. That is the negative disc
  ///                  on the positive side and vice versa.
  template <typename MatchTransformF = decltype(identityKey)>
  void addBothEndcapSides(ActsPlugins::DD4hep::BlueprintBuilder& builder, Acts::BlueprintNode& parent,
                          const std::string& container, const std::regex& posFilter, const std::regex& negFilter,
                          AxisDefinition axes, Layout layout, const Acts::ExtentEnvelope& envelope,
                          const std::string& posLabel = "", const std::string& negLabel = "",
                          MatchTransformF                     keyXform = identityKey,
                          const std::optional<LayerMaterial>& material = std::nullopt) {
    using enum Acts::CylinderVolumeBounds::Face;
    auto onPos = designateLayers(material, NegativeDisc, container);
    auto onNeg = designateLayers(material, PositiveDisc, container);

    if (layout == Layout::Grouped) {
      addGroupedEndcapSide(builder, parent, container, posFilter, axes, envelope, std::move(onPos));
      addGroupedEndcapSide(builder, parent, container, negFilter, axes, envelope, std::move(onNeg));
    } else {
      addUngroupedEndcapSide(builder, parent, container, posFilter, axes, posLabel, envelope, keyXform,
                             std::move(onPos));
      addUngroupedEndcapSide(builder, parent, container, negFilter, axes, negLabel, envelope, keyXform,
                             std::move(onNeg));
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
  /// @param material  If set, designate the inner cylinder of every barrel
  ///                  layer as a material receiver
  ///
  /// @returns The barrel blueprint node
  std::shared_ptr<ContainerBlueprintNode> makeGroupedBarrel(
      ActsPlugins::DD4hep::BlueprintBuilder& builder, const std::string& container, const std::regex& filter,
      AxisDefinition axes, const Acts::ExtentEnvelope& envelope,
      const std::optional<LayerMaterial>& material = std::nullopt) {
    return builder.layers()
        .barrel()
        .setSensorAxes(std::move(axes))
        .setLayerFilter(filter)
        .setContainer(container)
        .setEnvelope(envelope)
        .setAttachmentStrategy(Acts::VolumeAttachmentStrategy::First)
        .onLayer(designateLayers(material, Acts::CylinderVolumeBounds::Face::InnerCylinder, container, unsetXYCoG))
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
  /// @param material If set, designate the inner cylinder of every barrel layer
  ///                 as a material receiver
  ///
  /// @returns The barrel blueprint node
  template <typename MatchTransformF = decltype(identityKey)>
  std::shared_ptr<ContainerBlueprintNode> makeBarrel(ActsPlugins::DD4hep::BlueprintBuilder& builder,
                                                     const TrackerSpec&                     spec,
                                                     const Acts::ExtentEnvelope&            envelope = kBarrelEnvelope,
                                                     MatchTransformF                        keyXform = identityKey,
                                                     const std::optional<LayerMaterial>&    material = std::nullopt) {
    if (spec.barrelLayout == Layout::Grouped) {
      return makeGroupedBarrel(builder, spec.barrelContainer, spec.barrelFilter, spec.barrelAxes, envelope, material);
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
        .onLayer(designateLayers(material, Acts::CylinderVolumeBounds::Face::InnerCylinder, spec.barrelContainer,
                                 unsetXYCoG))
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
  /// @param material      If set, designate the IP-facing disc of every endcap
  ///                      layer as a material receiver
  /// @param keyXform      Transform applied to capture group 1 of the filter regex
  ///                      to derive the layer-group key (Ungrouped path only)
  std::shared_ptr<CylinderContainerBlueprintNode> attachEndcaps(
      ActsPlugins::DD4hep::BlueprintBuilder& builder, std::shared_ptr<ContainerBlueprintNode>&& barrel,
      const TrackerSpec& spec, const std::string& containerName,
      const std::optional<LayerMaterial>&            material = std::nullopt,
      std::function<std::string(const std::string&)> keyXform = [](const std::string& m) {
        return std::to_string(std::stoi(m) / 2);
      }) {
    using enum Acts::CylinderVolumeBounds::Face;
    auto node = std::make_shared<CylinderContainerBlueprintNode>(containerName, AxisZ);
    node->addChild(barrel);

    auto onPos = designateLayers(material, NegativeDisc, spec.endcapContainer);
    auto onNeg = designateLayers(material, PositiveDisc, spec.endcapContainer);

    if (spec.endcapLayout == Layout::Grouped) {
      addGroupedEndcapSide(builder, *node, spec.endcapContainer, spec.endcapPosFilter, spec.endcapAxes,
                           kVertexEndcapEnvelope, std::move(onPos));
      addGroupedEndcapSide(builder, *node, spec.endcapContainer, spec.endcapNegFilter, spec.endcapAxes,
                           kVertexEndcapEnvelope, std::move(onNeg));
    } else {
      addUngroupedEndcapSide(builder, *node, spec.endcapContainer, spec.endcapPosFilter, spec.endcapAxes,
                             fmt::format("{}|doubleLayer_pos", spec.endcapContainer), kUngroupedVertexEndcapEnvelope,
                             keyXform, std::move(onPos));
      addUngroupedEndcapSide(builder, *node, spec.endcapContainer, spec.endcapNegFilter, spec.endcapAxes,
                             fmt::format("{}|doubleLayer_neg", spec.endcapContainer), kUngroupedVertexEndcapEnvelope,
                             std::move(keyXform), std::move(onNeg));
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
  /// @param barrelMaterial If set, designate the inner cylinder of every barrel
  ///                       layer as a material receiver
  /// @param endcapMaterial If set, designate the IP-facing disc of every endcap
  ///                       layer as a material receiver
  ///
  /// @returns The tracker blueprint node
  std::shared_ptr<CylinderContainerBlueprintNode> makeRegularTracker(
      ActsPlugins::DD4hep::BlueprintBuilder& builder, const TrackerSpec& spec, const std::string& trackerName,
      const std::optional<LayerMaterial>& barrelMaterial = std::nullopt,
      const std::optional<LayerMaterial>& endcapMaterial = std::nullopt) {
    auto tracker = std::make_shared<CylinderContainerBlueprintNode>(trackerName, AxisZ);
    tracker->addChild(makeBarrel(builder, spec, kTrackerEnvelope, identityKey, barrelMaterial));
    addBothEndcapSides(builder, *tracker, spec.endcapContainer, spec.endcapPosFilter, spec.endcapNegFilter,
                       spec.endcapAxes, spec.endcapLayout, kTrackerEnvelope,
                       fmt::format("{}|layer_pos", spec.endcapContainer),
                       fmt::format("{}|layer_neg", spec.endcapContainer), identityKey, endcapMaterial);
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
  /// @param barrelMaterial If set, designate the inner cylinder of every barrel
  ///                       layer as a material receiver
  /// @param endcapMaterial If set, designate the IP-facing disc of every endcap
  ///                       layer as a material receiver
  ///
  /// @returns The inner tracker blueprint node
  std::shared_ptr<CylinderContainerBlueprintNode> makeNestedInnerTracker(
      ActsPlugins::DD4hep::BlueprintBuilder& builder, std::shared_ptr<CylinderContainerBlueprintNode>&& vertex,
      const NestedInnerTrackerSpec&       spec           = NestedInnerTrackerSpec{},
      const std::optional<LayerMaterial>& barrelMaterial = std::nullopt,
      const std::optional<LayerMaterial>& endcapMaterial = std::nullopt) {
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
    auto innerInnerBarrel = makeGroupedBarrel(builder, spec.barrelContainer, spec.barrelInnerFilter, spec.barrelAxes,
                                              kTrackerEnvelope, barrelMaterial);
    innerInnerBarrel->addChild(vertex);

    auto innerInnerTracker = std::make_shared<CylinderContainerBlueprintNode>("InnerInnerTracker", AxisZ);
    innerInnerTracker->addChild(innerInnerBarrel);

    addBothEndcapSides(builder, *innerInnerTracker, spec.endcapContainer, spec.endcapPosInnerFilter,
                       spec.endcapNegInnerFilter, spec.endcapAxes, spec.layout, kTrackerEnvelope,
                       fmt::format("{}|layer_pos", spec.endcapContainer),
                       fmt::format("{}|layer_neg", spec.endcapContainer), identityKey, endcapMaterial);

    auto innerTracker = std::make_shared<CylinderContainerBlueprintNode>("InnerTracker", AxisZ);
    innerTracker->addCylinderContainer("InnerTrackerBarrel", AxisR, [&](auto& innerBarrel) {
      innerBarrel.addChild(innerInnerTracker);
      auto outerBarrel = makeGroupedBarrel(builder, spec.barrelContainer, spec.barrelOuterFilter, spec.barrelAxes,
                                           kTrackerEnvelope, barrelMaterial);
      innerBarrel.addChild(outerBarrel);
    });

    addBothEndcapSides(builder, *innerTracker, spec.endcapContainer, spec.endcapPosOuterFilter,
                       spec.endcapNegOuterFilter, spec.endcapAxes, spec.layout, kTrackerEnvelope,
                       fmt::format("{}|layer_pos", spec.endcapContainer),
                       fmt::format("{}|layer_neg", spec.endcapContainer), identityKey, endcapMaterial);

    return innerTracker;
  }

  /// Navigation policy factory for the passive calo volumes. Each calo volume
  /// holds only a handful of explicitly added surfaces (the polygon barrel
  /// faces, or a single endcap disc). A TryAll policy (portals plus all passive
  /// surfaces) is the simplest robust choice here: with so few surfaces there is
  /// no benefit to a binned SurfaceArray, and TryAll needs no binning
  /// configuration that could be mis-set.
  std::shared_ptr<Acts::NavigationPolicyFactory> makeCaloNavigationPolicyFactory() {
    return std::make_shared<Acts::NavigationPolicyFactory>(
        Acts::NavigationPolicyFactory{}.add<Acts::TryAllNavigationPolicy>(Acts::TryAllNavigationPolicy::Config{}));
  }

  /// Add the calorimeter barrel as a passive static volume to @p parent (the
  /// radial container around the tracker). The volume is a cylinder enclosing
  /// the regular-polygon inner face, with one planar surface per polygon side.
  ///
  /// A barrel of planar surfaces would normally be a Cylinder layer (a
  /// cylindrical layer volume whose modules are binned into a SurfaceArray, as
  /// the tracker barrels are). Here the face is just a few polygon planes, so a
  /// hand-built static volume navigated with a TryAll policy is simpler and
  /// avoids picking a SurfaceArray binning for a non-cylindrical polygon; it is
  /// not a workaround for any missing layer type.
  void addCaloBarrel(BlueprintNode& parent, const IActsGeoSvc::CaloFaceSurfaces& calo) {
    constexpr double pad    = 1_mm;
    auto             bounds = std::make_shared<Acts::CylinderVolumeBounds>(std::max(0.0, calo.barrelRMin - pad),
                                                                           calo.barrelRMax + pad, calo.barrelHalfZ + pad);
    auto vol = std::make_unique<Acts::TrackingVolume>(Acts::Transform3::Identity(), std::move(bounds), "CaloBarrel");
    for (const auto& face : calo.barrelFaces) {
      vol->addSurface(face);
    }
    parent.addStaticVolume(std::move(vol)).setNavigationPolicyFactory(makeCaloNavigationPolicyFactory());
  }

  /// Add one calorimeter endcap disc as a passive static volume to @p parent
  /// (the top-level z container). The volume abuts the central region in z and
  /// extends out beyond the disc face; it shares the central radial extent so
  /// it stacks cleanly along z.
  void addCaloEndcap(BlueprintNode& parent, const IActsGeoSvc::CaloFaceSurfaces& calo, bool positive) {
    const auto& disc = positive ? calo.endcapPos : calo.endcapNeg;
    if (!disc) {
      return;
    }
    constexpr double pad = 1_mm;
    // The central region (which holds the calo barrel) reaches barrelHalfZ plus
    // the barrel volume's z-padding. Start the endcap just beyond that so the
    // top-level z-stack sees a small gap rather than an overlap. The endcap
    // disc (at endcapZ, which the surface builder keeps clear of the barrel)
    // then sits comfortably inside the volume.
    const double zInner = calo.barrelHalfZ + 2 * pad;
    const double zOuter = calo.endcapZ + 10_mm;  // beyond the disc face
    const double halfZ  = std::max(5_mm, (zOuter - zInner) / 2.0);
    const double zc     = (zInner + zOuter) / 2.0;

    // Span the full radius (0 .. barrel circumradius) so the volume shares the
    // central radial extent and the z-stack does not need radial gap shells.
    auto bounds = std::make_shared<Acts::CylinderVolumeBounds>(0.0, calo.barrelRMax + pad, halfZ);

    Acts::Transform3 transform = Acts::Transform3::Identity();
    transform.translation()    = Acts::Vector3{0, 0, positive ? zc : -zc};
    auto vol                   = std::make_unique<Acts::TrackingVolume>(transform, std::move(bounds),
                                                      positive ? "CaloEndcapPos" : "CaloEndcapNeg");
    vol->addSurface(disc);
    parent.addStaticVolume(std::move(vol)).setNavigationPolicyFactory(makeCaloNavigationPolicyFactory());
  }

  /// Add the single rectangular calorimeter face of a telescope geometry (e.g.
  /// LUXE) as a passive static cuboid volume to @p parent (a cuboid container
  /// stacked along z). The volume is the calorimeter slab's bounding box and
  /// holds one planar surface at its upstream face.
  ///
  /// This is the telescope analogue of @c addCaloBarrel / @c addCaloEndcap: the
  /// cylindrical detectors wrap the tracker with polygon/disc calo volumes,
  /// whereas a telescope simply stacks a cuboid calo volume behind the tracker
  /// layers along the beam (z) axis.
  void addCaloPlanarFace(BlueprintNode& parent, const IActsGeoSvc::CaloFaceSurfaces& calo) {
    if (!calo.planarFace) {
      return;
    }
    constexpr double pad = 1_mm;
    // The parent cuboid container stacks its children along z, which requires
    // every child to share the same transverse (x, y) center. The tracker
    // layers are forced onto the beam axis (x = y = 0, via unsetXYCoG), so the
    // calo volume is centered there too and made wide enough to still enclose
    // the (transversely offset) calo slab. Only the z position follows the
    // slab.
    const double cx     = calo.planarVolumeCenter[0];
    const double cy     = calo.planarVolumeCenter[1];
    const double cz     = calo.planarVolumeCenter[2];
    const double halfX  = std::abs(cx) + calo.planarVolumeHalfLen[0] + pad;
    const double halfY  = std::abs(cy) + calo.planarVolumeHalfLen[1] + pad;
    const double halfZ  = calo.planarVolumeHalfLen[2] + pad;
    auto         bounds = std::make_shared<Acts::CuboidVolumeBounds>(halfX, halfY, halfZ);

    Acts::Transform3 transform = Acts::Transform3::Identity();
    transform.translation()    = Acts::Vector3{0, 0, cz};
    auto vol                   = std::make_unique<Acts::TrackingVolume>(transform, std::move(bounds), "CaloFace");
    vol->addSurface(calo.planarFace);
    parent.addStaticVolume(std::move(vol)).setNavigationPolicyFactory(makeCaloNavigationPolicyFactory());
  }

}  // namespace Blueprints

namespace MuColl {
  namespace MAIA_v0 {
    void populateBlueprint(const std::string& detName, Acts::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder, const IActsGeoSvc::CaloFaceSurfaces& calo) {
      // Material receivers for MAIA. See the "Material designation" block in the
      // Blueprints namespace for why only the IP-facing face of each layer is
      // designated. The three barrels differ in whether their innermost layer
      // has to be left out, because that depends on what else sits in the same
      // radial stack:
      //
      // - VertexBarrel and OuterTrackerBarrel are plain layer stacks, so the
      //   inner cylinder of their layer 0 *is* the container's inner cylinder.
      //   Both containers are then children of a z-stack (Vertex and
      //   OuterTracker), which merges its children's cylinder faces, so those
      //   layers are skipped. Designating them aborts the construction with
      //   "Material is designated on portal faces that are merged when stacking
      //   child volumes in AxisZ direction".
      // - InnerTrackerBarrel encloses the whole vertex detector
      //   (makeNestedInnerTracker adds it as a radial child), so its layer0
      //   inner cylinder is an interior portal fused with Vertex's outer
      //   cylinder, which designates nothing. It is safe, and designated.
      //
      // Nothing is lost by skipping: each excluded face survives as part of a
      // receiver that is designated. VertexBarrel's is Beampipe.OuterCylinder
      // (the same portal, shared through every enclosing container), and
      // OuterTrackerBarrel's is InnerTracker.OuterCylinder.
      const auto vertexBarrelMaterial = Blueprints::LayerMaterial{.skipInnermost = std::regex{"layer_0"}};
      const auto innerBarrelMaterial  = Blueprints::LayerMaterial{};
      const auto outerBarrelMaterial  = Blueprints::LayerMaterial{.skipInnermost = std::regex{"layer0"}};
      const auto endcapMaterial       = Blueprints::LayerMaterial{};

      // Build the tracker detectors as radial children of the supplied
      // container.
      auto buildTrackers = [&](ContainerBlueprintNode& outer) {
        Blueprints::addCylindricalBeampipe(outer, /*designateMaterial=*/true);

        // NOTE: Need to set rather small padding here for the R-direction,
        // because the innermost two layers are a double layer for which the
        // cylindrical volumes are overlapping otherwise
        auto vertexBarrel = Blueprints::makeGroupedBarrel(builder, "VertexBarrel", std::regex{"layer_\\d"}, "ZYX",
                                                          Blueprints::kTightBarrelEnvelope, vertexBarrelMaterial);
        auto vertex       = Blueprints::attachEndcaps(builder, std::move(vertexBarrel),
                                                      Blueprints::DoubleBarrelLayerVertexSpec, "Vertex", endcapMaterial);

        auto innerTracker = Blueprints::makeNestedInnerTracker(
            builder, std::move(vertex), Blueprints::NestedInnerTrackerSpec{}, innerBarrelMaterial, endcapMaterial);
        // The inner tracker's outer cylinder is fused with the outer tracker's
        // inner cylinder, which carries no material (the innermost outer-tracker
        // barrel layer is skipped), so it is free to receive the material of the
        // services between the two subdetectors.
        outer.addChild(Blueprints::withMaterial(innerTracker, "InnerTracker_Material",
                                                Acts::CylinderVolumeBounds::Face::OuterCylinder));

        auto outerTracker = Blueprints::makeRegularTracker(builder, Blueprints::OuterTrackerSpec, "OuterTracker",
                                                           outerBarrelMaterial, endcapMaterial);
        outer.addChild(Blueprints::withMaterial(outerTracker, "OuterTracker_Material",
                                                Acts::CylinderVolumeBounds::Face::OuterCylinder));

        // In MAIA_v0 the solenoid sits between the outer tracker and the
        // calorimeter (coil at r = 1500..1857 mm, ECAL barrel face at 1857 mm),
        // so every extrapolation to the calorimeter face crosses it. Give it a
        // volume of its own: without one the radial stack stretches the outer
        // tracker across the whole coil and the designation above ends up hard
        // against the calorimeter face, with nothing anywhere near the coil.
        //
        // With the volume in place the two designations bracket the coil --
        // OuterTracker_Material lands at its inner face and Solenoid_Material at
        // its outer face -- so the mapping can spread the coil material over its
        // thickness rather than lumping it at one edge. A single surface inside
        // the coil would be better still; that is the next refinement.
        //
        // Guarded, not unconditional: MuSIC_v2 shares this blueprint but puts
        // its coil at r = 2055 mm, outside its calorimeter (1690..1960 mm),
        // where it is irrelevant for the extrapolation and a volume here would
        // not fit the radial stack at all.
        if (const auto solenoidRMin = Blueprints::findSolenoidInnerRadius(builder);
            !calo.empty() && solenoidRMin.has_value() && *solenoidRMin < calo.barrelRMin) {
          Blueprints::addCylindricalSolenoid(outer, *solenoidRMin, /*designateMaterial=*/true);
        }
      };

      if (calo.empty()) {
        // No calorimeter face: keep the original purely-radial layout.
        auto& outer = root.addCylinderContainer(detName, AxisR);
        buildTrackers(outer);
        return;
      }

      // The calorimeter wraps the tracker: its endcaps reach to small radius at
      // large |z| where the tracker does not extend. This is expressed as a
      // top-level z-stack [calo -endcap | central (tracker + calo barrel) |
      // calo +endcap], with the calo barrel as the outermost radial child of
      // the central region.
      auto& world = root.addCylinderContainer(detName, AxisZ);
      Blueprints::addCaloEndcap(world, calo, /*positive=*/false);
      auto& central = world.addCylinderContainer(detName + "Central", AxisR);
      buildTrackers(central);
      if (!calo.barrelFaces.empty()) {
        Blueprints::addCaloBarrel(central, calo);
      }
      Blueprints::addCaloEndcap(world, calo, /*positive=*/true);
    }
  }  // namespace MAIA_v0
}  // namespace MuColl

namespace FCCee {
  namespace ILD_FCCee_v01 {
    void populateBlueprint(const std::string& detName, Acts::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder, const IActsGeoSvc::CaloFaceSurfaces& calo) {
      // Build the tracker detectors as radial children of the supplied
      // container.
      auto buildTrackers = [&](ContainerBlueprintNode& outer) {
        Blueprints::addCylindricalBeampipe(outer);

        auto vtxBarrel = Blueprints::makeBarrel(builder, Blueprints::UngroupedDoubleBarrelLayerVertexSpec,
                                                Blueprints::kBarrelEnvelope, Blueprints::doubleLayerKey);
        auto vertex    = Blueprints::attachEndcaps(builder, std::move(vtxBarrel),
                                                   Blueprints::UngroupedDoubleBarrelLayerVertexSpec, "Vertex");

        auto innerTrackerBarrel = Blueprints::makeBarrel(builder, Blueprints::UngroupedInnerTrackerSpec);
        innerTrackerBarrel->addChild(vertex);

        auto innerTrackerEndcap = Blueprints::attachEndcaps(
            builder, std::move(innerTrackerBarrel), Blueprints::UngroupedInnerTrackerSpec, "InnerTrackerEndcap");
        outer.addChild(innerTrackerEndcap);

        // TODO: this is not yet properly working only part of the SET show up in
        // the exporte .obj geometry. This usually indicates some issues with the
        // AxisDirection, but that would mean that there are different
        // AxisDirections in play for the SET geometry
        auto set = Blueprints::makeBarrel(builder, Blueprints::SETSpec, Blueprints::kBarrelEnvelope,
                                          Blueprints::doubleLayerKey);
        outer.addChild(set);
      };

      if (calo.empty()) {
        // No calorimeter face: keep the original purely-radial layout.
        auto& outer = root.addCylinderContainer(detName, AxisR);
        buildTrackers(outer);
        return;
      }

      // The calorimeter wraps the tracker, so the top level is a z-stack
      // [calo -endcap | central (tracker + calo barrel) | calo +endcap] (see
      // MAIA_v0 for details).
      auto& world = root.addCylinderContainer(detName, AxisZ);
      Blueprints::addCaloEndcap(world, calo, /*positive=*/false);
      auto& central = world.addCylinderContainer(detName + "Central", AxisR);
      buildTrackers(central);
      if (!calo.barrelFaces.empty()) {
        Blueprints::addCaloBarrel(central, calo);
      }
      Blueprints::addCaloEndcap(world, calo, /*positive=*/true);
    }
  }  // namespace ILD_FCCee_v01

  namespace ILD_FCCee_v02 {
    void populateBlueprint(const std::string& detName, Acts::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder, const IActsGeoSvc::CaloFaceSurfaces& calo) {
      // Build the tracker detectors as radial children of the supplied
      // container.
      auto buildTrackers = [&](ContainerBlueprintNode& outer) {
        Blueprints::addCylindricalBeampipe(outer);
        auto vtxBarrel = Blueprints::makeBarrel(builder, Blueprints::UngroupedDoubleBarrelLayerVertexSpec,
                                                Blueprints::kBarrelEnvelope, Blueprints::doubleLayerKey);
        auto vertex    = Blueprints::attachEndcaps(builder, std::move(vtxBarrel),
                                                   Blueprints::UngroupedDoubleBarrelLayerVertexSpec, "Vertex");

        auto innerTracker =
            Blueprints::makeNestedInnerTracker(builder, std::move(vertex), Blueprints::UngroupedNestedInnerTrackerSpec);
        outer.addChild(innerTracker);

        // TODO: Add SET (see V01 for caveats)
      };

      if (calo.empty()) {
        // No calorimeter face: keep the original purely-radial layout.
        auto& outer = root.addCylinderContainer(detName, AxisR);
        buildTrackers(outer);
        return;
      }

      // The calorimeter wraps the tracker, so the top level is a z-stack
      // [calo -endcap | central (tracker + calo barrel) | calo +endcap] (see
      // MAIA_v0 for details).
      auto& world = root.addCylinderContainer(detName, AxisZ);
      Blueprints::addCaloEndcap(world, calo, /*positive=*/false);
      auto& central = world.addCylinderContainer(detName + "Central", AxisR);
      buildTrackers(central);
      if (!calo.barrelFaces.empty()) {
        Blueprints::addCaloBarrel(central, calo);
      }
      Blueprints::addCaloEndcap(world, calo, /*positive=*/true);
    }
  }  // namespace ILD_FCCee_v02

  namespace CLD_o2_v07 {
    void populateBlueprint(const std::string& detName, Acts::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder, const IActsGeoSvc::CaloFaceSurfaces& calo) {
      // Build the tracker detectors as radial children of the supplied
      // container.
      auto buildTrackers = [&](ContainerBlueprintNode& outer) {
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
      };

      if (calo.empty()) {
        // No calorimeter face: keep the original purely-radial layout.
        auto& outer = root.addCylinderContainer(detName, AxisR);
        buildTrackers(outer);
        return;
      }

      // The calorimeter wraps the tracker, so the top level is a z-stack
      // [calo -endcap | central (tracker + calo barrel) | calo +endcap] (see
      // MAIA_v0 for details).
      auto& world = root.addCylinderContainer(detName, AxisZ);
      Blueprints::addCaloEndcap(world, calo, /*positive=*/false);
      auto& central = world.addCylinderContainer(detName + "Central", AxisR);
      buildTrackers(central);
      if (!calo.barrelFaces.empty()) {
        Blueprints::addCaloBarrel(central, calo);
      }
      Blueprints::addCaloEndcap(world, calo, /*positive=*/true);
    }
  }  // namespace CLD_o2_v07
}  // namespace FCCee

namespace LUXE {
  namespace LUXE_v0 {
    void populateBlueprint(const std::string& detName, Acts::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder, const IActsGeoSvc::CaloFaceSurfaces& calo) {
      // LUXE has a telescope-like geometry: the tracker planar layers are
      // stacked along the beam (z) axis, and the electromagnetic calorimeter is
      // a single rectangular slab downstream. Both live in a cuboid container
      // stacked along z (the calo cuboid sits behind the tracker layers).
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
          // Force the layer volumes onto the beam axis (x = y = 0) so they share
          // a common transverse center with the calo volume for the z-stack.
          .onLayer(Blueprints::unsetXYCoG)
          .addTo(tracker);

      // Insert the ECAL front face as an extrapolation target, if it was built.
      Blueprints::addCaloPlanarFace(tracker, calo);
    }
  }  // namespace LUXE_v0
}  // namespace LUXE
