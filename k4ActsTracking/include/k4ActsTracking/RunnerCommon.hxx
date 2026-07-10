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
#pragma once

// Shared ACTS type aliases and small helpers used by both the combinatorial
// (CKFRunner) and standalone-fit (KFRunner) track runners.

// k4ActsTracking
#include "k4ActsTracking/CKFTracking.hxx"
#include "k4ActsTracking/IActsGeoSvc.h"
#include "k4ActsTracking/Measurement.hxx"
#include "k4ActsTracking/SourceLink.hxx"

// edm4hep
#include <edm4hep/TrackerHit.h>
#include <edm4hep/TrackerHitPlaneCollection.h>

// podio
#include <podio/ObjectID.h>

// ACTS
#include <Acts/Definitions/Units.hpp>
#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/EventData/ParticleHypothesis.hpp>
#include <Acts/EventData/SourceLink.hpp>
#include <Acts/EventData/TrackContainer.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>
#include <Acts/EventData/VectorTrackContainer.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Geometry/GeometryIdentifier.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Propagator/EigenStepper.hpp>
#include <Acts/Propagator/Navigator.hpp>
#include <Acts/Propagator/Propagator.hpp>
#include <Acts/Seeding/EstimateTrackParamsFromSeed.hpp>
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/Utilities/Result.hpp>

// TBB
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>

// Standard
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ACTSTracking {

  /// Track container shared by the CKF and Kalman-fit runners.
  using CKFTrackContainer =
      Acts::TrackContainer<Acts::VectorTrackContainer, Acts::VectorMultiTrajectory, std::shared_ptr>;

  /// Stepper / navigator / propagator shared by the runners.
  using CKFStepper    = Acts::EigenStepper<>;
  using CKFNavigator  = Acts::Navigator;
  using CKFPropagator = Acts::Propagator<CKFStepper, CKFNavigator>;

  /// Build a propagator over the tracking geometry. @p resolvePassive selects
  /// whether passive surfaces (e.g. calorimeter faces) are resolved during
  /// navigation. The result depends only on the geometry and field, so callers
  /// can build the propagators they need once and reuse them across events.
  inline CKFPropagator makePropagator(const IActsGeoSvc& geo, bool resolvePassive) {
    CKFNavigator::Config navigatorCfg{geo.trackingGeometry()};
    navigatorCfg.resolvePassive   = resolvePassive;
    navigatorCfg.resolveMaterial  = true;
    navigatorCfg.resolveSensitive = true;
    return CKFPropagator(CKFStepper(geo.magneticField()), CKFNavigator(navigatorCfg));
  }

  /// Stable key for matching an edm4hep tracker hit across collections.
  inline std::uint64_t trackerHitKey(const edm4hep::TrackerHit& hit) {
    const podio::ObjectID id = hit.getObjectID();
    return (static_cast<std::uint64_t>(id.collectionID) << 32) | static_cast<std::uint32_t>(id.index);
  }

  /**
   * @brief Build the ACTS measurements and source links for a set of tracker hits.
   *
   * Shared hit-conversion loop of the tracking algorithms:
   *  - looks up the ACTS surface for each hit's cellID,
   *  - sorts hits by geometry identifier for efficient multiset insertion,
   *  - converts global to local coordinates and creates a Measurement + SourceLink.
   *
   * For each converted hit the @p hitSink callback is invoked with the hit, its
   * SourceLink and the geometry needed to build seeding space points, so each
   * algorithm can decide what to keep (a per-hit source-link map, seed grid
   * inputs, ...).
   *
   * The @p hits container is filled in lockstep with @p measurements: entry i is
   * the edm4hep hit of the source link whose index() is i, so the compact
   * SourceLink can recover its hit via hits[sourceLink.index()].
   *
   * @tparam Alg     Owning Gaudi algorithm (used only for level-aware logging).
   * @tparam HitSink Callback (const edm4hep::TrackerHitPlane&, const SourceLink&,
   *                 const Acts::Vector3& globalPos, const Acts::Surface&,
   *                 const Acts::SquareMatrix2& localCov).
   */
  template <class Alg, class HitSink>
  void prepareTrackerHits(const Alg& alg, const IActsGeoSvc& geo, const Acts::GeometryContext& geoCtx,
                          const edm4hep::TrackerHitPlaneCollection& trackerHits,
                          ACTSTracking::MeasurementContainer&       measurements,
                          ACTSTracking::SourceLinkContainer& sourceLinks, ACTSTracking::HitContainer& hits,
                          int numThreads, HitSink&& hitSink) {
    const auto& cellIdToSurface = geo.cellIdToSurfaceMap();

    std::vector<std::pair<Acts::GeometryIdentifier, edm4hep::TrackerHitPlane>> sortedHits;
    sortedHits.reserve(trackerHits.size());

    for (const auto& hit : trackerHits) {
      auto it = cellIdToSurface.find(hit.getCellID());
      if (it == cellIdToSurface.end()) {
        alg.warning() << "No surface found for cellID " << hit.getCellID() << ". skipping hit for tracking." << endmsg;
        continue;
      }
      sortedHits.push_back({it->second->geometryId(), hit});
    }
    alg.debug() << "Working with " << sortedHits.size() << " hits." << endmsg;

    // Sort hits by geometry ID for efficient SourceLink multiset insertion
    auto compare = [](const auto& a, const auto& b) { return a.first < b.first; };
    if (numThreads > 1) {
      tbb::task_arena arena(numThreads);
      arena.execute([&] { tbb::parallel_sort(sortedHits.begin(), sortedHits.end(), compare); });
    } else {
      std::sort(sortedHits.begin(), sortedHits.end(), compare);
    }

    sourceLinks.reserve(sortedHits.size());
    hits.reserve(sortedHits.size());

    for (const auto& hitPair : sortedHits) {
      const Acts::Surface* surface = geo.trackingGeometry()->findSurface(hitPair.first);
      if (surface == nullptr) {
        alg.warning() << "Surface with geoID " << hitPair.first
                      << " not found in tracking geometry. Skipping hit for tracking." << endmsg;
        continue;
      }

      const edm4hep::Vector3d& edmGlobalPos = hitPair.second.getPosition();
      Acts::Vector3            globalPos    = {edmGlobalPos.x, edmGlobalPos.y, edmGlobalPos.z};

      Acts::Result<Acts::Vector2> lpResult =
          surface->globalToLocal(geoCtx, globalPos, {0, 0, 0}, 0.5 * Acts::UnitConstants::um);
      if (!lpResult.ok()) {
        alg.warning() << "Global to local transformation did not succeed for hit. Skipping it in tracking." << endmsg;
        continue;
      }

      Acts::Vector2 loc = lpResult.value();

      Acts::SquareMatrix2 localCov = Acts::SquareMatrix2::Zero();
      localCov(0, 0)               = std::pow(hitPair.second.getDu() * Acts::UnitConstants::mm, 2);
      localCov(1, 1)               = std::pow(hitPair.second.getDv() * Acts::UnitConstants::mm, 2);

      ACTSTracking::SourceLink  sourceLink(surface->geometryId(), measurements.size());
      Acts::SourceLink          srcWrap{sourceLink};
      ACTSTracking::Measurement meas =
          ACTSTracking::makeMeasurement(srcWrap, loc, localCov, Acts::eBoundLoc0, Acts::eBoundLoc1);

      measurements.push_back(meas);
      hits.push_back(hitPair.second);
      sourceLinks.emplace_hint(sourceLinks.end(), sourceLink);

      hitSink(hitPair.second, sourceLink, globalPos, *surface, localCov);
    }
  }

  /**
   * @brief Estimate initial bound track parameters from a three-point seed.
   *
   * Evaluates the magnetic field at the bottom position, calls
   * Acts::estimateTrackParamsFromSeed on the bottom/middle/top positions and
   * builds a diagonal initial covariance.
   *
   * @return The estimated parameters, or std::nullopt if the estimation fails. A
   *         field-lookup failure throws, matching the tracking algorithms.
   */
  template <class Alg>
  std::optional<Acts::BoundTrackParameters> estimateSeedParameters(
      const Alg& alg, const IActsGeoSvc& geo, const Acts::GeometryContext& geoCtx, const Acts::Surface& bottomSurface,
      const Acts::Vector3& bottomPos, const Acts::Vector3& middlePos, const Acts::Vector3& topPos, double t0,
      Acts::MagneticFieldProvider::Cache& magCache, double errPos, double errPhi, double errLambda, double errRelP,
      double errTime) {
    // Magnetic field at the seed (bottom space point) position
    Acts::Result<Acts::Vector3> seedField = geo.magneticField()->getField(bottomPos, magCache);
    if (!seedField.ok()) {
      throw std::runtime_error("Field lookup error: " + std::to_string(seedField.error().value()));
    }

    Acts::Result<Acts::BoundVector> optParams =
        Acts::estimateTrackParamsFromSeed(geoCtx, bottomSurface, bottomPos, t0, middlePos, topPos, *seedField);
    if (!optParams.ok()) {
      alg.debug() << "Failed estimation of track parameters for seed." << endmsg;
      return std::nullopt;
    }

    const Acts::BoundVector& params = *optParams;
    float                    p      = std::abs(1.f / params[Acts::eBoundQOverP]);

    Acts::BoundMatrix cov = ACTSTracking::makeInitialCovariance(p, errPos, errPhi, errLambda, errRelP, errTime);

    return Acts::BoundTrackParameters(bottomSurface.getSharedPtr(), params, cov, Acts::ParticleHypothesis::pion());
  }

}  // namespace ACTSTracking
