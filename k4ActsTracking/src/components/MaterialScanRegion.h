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
#ifndef K4ACTSTRACKING_MATERIALSCANREGION_H
#define K4ACTSTRACKING_MATERIALSCANREGION_H

#include "k4ActsTracking/IActsGeoSvc.h"

#include <Acts/Definitions/Algebra.hpp>
#include <Acts/Geometry/CylinderVolumeBounds.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/Geometry/TrackingVolume.hpp>
#include <Acts/Material/MaterialInteraction.hpp>
#include <Acts/Material/MaterialSlab.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

/// The region whose material is recorded and mapped: r < r, |z| < halfZ.
///
/// By default it ends at the ECAL inner face (the cylinder inscribed in the
/// barrel polygon, and the endcap discs). Material behind it has to stay out of
/// the map: the mapper hands every recorded step to the nearest receiver and
/// would pile the calorimeter onto the outermost one. A detector without a
/// calorimeter face falls back to the extent of the tracking world.
namespace MaterialScanRegion {

struct Cylinder {
  double r = 0.;
  double halfZ = 0.;

  bool contains(const Acts::Vector3& position) const {
    return position.head<2>().norm() < r && std::abs(position.z()) < halfZ;
  }

  /// Path length from @p position along the unit vector @p direction to the
  /// boundary, or a negative value if @p position is outside.
  double pathToBoundary(const Acts::Vector3& position, const Acts::Vector3& direction) const {
    if (!contains(position)) {
      return -1.;
    }
    constexpr double inf = std::numeric_limits<double>::infinity();
    // Barrel: |position + s * direction|_T = r, the positive root.
    double sR = inf;
    const double a = direction.head<2>().squaredNorm();
    if (a > 0.) {
      const double b = position.head<2>().dot(direction.head<2>());
      const double c = position.head<2>().squaredNorm() - r * r;
      sR = (-b + std::sqrt(b * b - a * c)) / a;
    }
    // Endcaps.
    double sZ = inf;
    if (direction.z() != 0.) {
      sZ = ((direction.z() > 0. ? halfZ : -halfZ) - position.z()) / direction.z();
    }
    return std::min(sR, sZ);
  }

  /// Drop the recorded steps that start outside and cut the one that leaves
  /// at the boundary. A step is taken to start at its recorded position, as
  /// in ACTS' Geant4 recording and in @c MaterialRecordingAlg.
  ///
  /// @returns the x/X0 that was removed
  double clip(Acts::RecordedMaterialTrack& track) const {
    auto& steps = track.second.materialInteractions;
    double removed = 0.;
    std::vector<Acts::MaterialInteraction> kept;
    kept.reserve(steps.size());
    for (auto& step : steps) {
      const double inside = pathToBoundary(step.position, step.direction.normalized());
      if (inside <= 0.) {
        removed += step.materialSlab.thicknessInX0();
        continue;
      }
      if (step.materialSlab.thickness() > inside) {
        const double before = step.materialSlab.thicknessInX0();
        step.materialSlab = Acts::MaterialSlab(step.materialSlab.material(), static_cast<float>(inside));
        removed += before - step.materialSlab.thicknessInX0();
      }
      kept.push_back(std::move(step));
    }
    steps = std::move(kept);
    return removed;
  }
};

/// The default region of @p geo, with each positive override (in mm) taking
/// precedence. Empty if a value is neither given nor derivable.
inline std::optional<Cylinder> fromGeometry(const IActsGeoSvc& geo, double rOverride, double zOverride) {
  const auto& calo = geo.caloFaceSurfaces();
  double r = calo.barrelFaces.empty() ? 0. : calo.barrelRMin;
  double z = calo.endcapPos != nullptr ? calo.endcapZ : (calo.barrelFaces.empty() ? 0. : calo.barrelHalfZ);

  const auto* world = geo.trackingGeometry()->highestTrackingVolume();
  if (const auto* bounds = dynamic_cast<const Acts::CylinderVolumeBounds*>(&world->volumeBounds())) {
    if (r <= 0.) {
      r = bounds->get(Acts::CylinderVolumeBounds::eMaxR);
    }
    if (z <= 0.) {
      z = bounds->get(Acts::CylinderVolumeBounds::eHalfLengthZ);
    }
  }
  if (rOverride > 0.) {
    r = rOverride;
  }
  if (zOverride > 0.) {
    z = zOverride;
  }
  if (r <= 0. || z <= 0.) {
    return std::nullopt;
  }
  return Cylinder{r, z};
}

} // namespace MaterialScanRegion

#endif
