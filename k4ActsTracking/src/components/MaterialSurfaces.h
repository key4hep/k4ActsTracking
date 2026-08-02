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
#ifndef K4ACTSTRACKING_MATERIALSURFACES_H
#define K4ACTSTRACKING_MATERIALSURFACES_H

#include <Acts/Geometry/Portal.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/Geometry/TrackingGeometryVisitor.hpp>
#include <Acts/Geometry/TrackingVolume.hpp>
#include <Acts/Material/ISurfaceMaterial.hpp>
#include <Acts/Material/ProtoSurfaceMaterial.hpp>
#include <Acts/Surfaces/Surface.hpp>

#include <unordered_set>
#include <vector>

/// Helpers shared between the geometry service, which designates the material
/// receivers, and the material mapping algorithm, which projects material onto
/// them. Both need the same notion of "this surface is a designated receiver".
namespace MaterialSurfaces {

  /// Whether @p material is one of the proto-material placeholders that
  /// @c Acts::MaterialDesignatorBlueprintNode attaches to a designated face.
  ///
  /// A placeholder carries the binning chosen during the blueprint construction
  /// but no actual material, and is replaced by the real thing when a mapped
  /// material map is loaded. Its presence is therefore the marker for "this
  /// surface is a material receiver that has not been filled in yet".
  ///
  /// @param material The surface material to test, may be nullptr
  ///
  /// @returns True if @p material is a proto-material placeholder
  inline bool isProtoMaterial(const Acts::ISurfaceMaterial* material) {
    return dynamic_cast<const Acts::ProtoGridSurfaceMaterial*>(material) != nullptr ||
           dynamic_cast<const Acts::ProtoSurfaceMaterial*>(material) != nullptr;
  }

  /// Collect the surfaces of @p trackingGeometry that carry a proto-material
  /// placeholder, i.e. the receivers designated by the blueprint.
  ///
  /// These are the surfaces the material mapping projects onto. Portals that are
  /// shared (fused) between two volumes are visited once per volume, so they are
  /// de-duplicated by address.
  ///
  /// @param trackingGeometry The constructed tracking geometry
  ///
  /// @returns The designated material receivers, in traversal order
  inline std::vector<const Acts::Surface*> collectProtoMaterialSurfaces(
      const Acts::TrackingGeometry& trackingGeometry) {
    std::vector<const Acts::Surface*>        surfaces{};
    std::unordered_set<const Acts::Surface*> seen{};

    // Acts dispatches the visitor on the argument type, so portals (which hold
    // most of the receivers) and plain surfaces need separate overloads.
    struct Collector {
      std::vector<const Acts::Surface*>&        surfaces;
      std::unordered_set<const Acts::Surface*>& seen;

      void add(const Acts::Surface& surface) const {
        if (!isProtoMaterial(surface.surfaceMaterial()) || !seen.insert(&surface).second) {
          return;
        }
        surfaces.push_back(&surface);
      }

      void operator()(const Acts::Portal& portal) const { add(portal.surface()); }
      void operator()(const Acts::Surface& surface) const { add(surface); }
    };
    trackingGeometry.apply(Collector{surfaces, seen});

    return surfaces;
  }

}  // namespace MaterialSurfaces

#endif  // K4ACTSTRACKING_MATERIALSURFACES_H
