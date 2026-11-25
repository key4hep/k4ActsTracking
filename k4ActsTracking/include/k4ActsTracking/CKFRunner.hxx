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

// k4ActsTracking
#include "k4ActsTracking/CKFTracking.hxx"
#include "k4ActsTracking/Helpers.hxx"
#include "k4ActsTracking/IActsGeoSvc.h"
#include "k4ActsTracking/Measurement.hxx"
#include "k4ActsTracking/RunnerCommon.hxx"
#include "k4ActsTracking/SourceLink.hxx"

// edm4hep
#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackState.h>
#include <edm4hep/TrackerHit.h>

// Gaudi
#include <GaudiKernel/CommonMessaging.h>

// ACTS
#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>
#include <Acts/TrackFinding/MeasurementSelector.hpp>
#include <Acts/TrackFinding/TrackSelector.hpp>
#include <Acts/Utilities/CalibrationContext.hpp>

// ActsPlugins: centralised ACTS -> EDM4hep conversion
#include <ActsPlugins/EDM4hep/EDM4hepUtil.hpp>

// Standard
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace ACTSTracking {

/// The CKF type owned by CKFRunner (shared runner types live in RunnerCommon.hxx).
using CombKalmanFilter = Acts::CombinatorialKalmanFilter<CKFPropagator, CKFTrackContainer>;

/**
 * @brief Convert estimated seed parameters into an edm4hep seed TrackState.
 *
 * @param location AtFirstHit for seeds at the bottom (inner) SP; AtLastHit for
 *                 outside-in seeds, whose parameters are at the top (outer) SP.
 */
template <class Alg>
edm4hep::TrackState makeSeedTrackState(const Alg& /*alg*/, const IActsGeoSvc& geo, const Acts::GeometryContext& geoCtx,
                                       const Acts::BoundTrackParameters& paramseed,
                                       Acts::MagneticFieldProvider::Cache& magCache,
                                       edm4hep::TrackState::Location location = edm4hep::TrackState::AtFirstHit) {
  // The centralised converter re-expresses the seed parameters at an ad-hoc
  // perigee and evaluates the local field at their position itself.
  return ActsPlugins::EDM4hepUtil::writeTrackState(geoCtx, location, paramseed, *geo.magneticField(), magCache);
}

/**
 * @brief Append a seed track (its tracker hits + seed track state) to @p seedCollection.
 *
 * Centralises the locked create/add pattern shared by the seeding algorithms:
 * the lock serialises the podio collection mutation, then a new track is created
 * with the given hits and seed track state. @p trackerHits is any range of
 * edm4hep::TrackerHit - e.g. a std::array for a fixed triplet seed, or a
 * transformed view for a variable-size candidate.
 */
template <class HitRange>
void appendSeedTrack(edm4hep::TrackCollection& seedCollection, std::mutex& seedMutex,
                     const edm4hep::TrackState& seedTrackState, const HitRange& trackerHits) {
  std::lock_guard<std::mutex> lock(seedMutex);
  auto seedTrack = seedCollection.create();
  for (const auto& hit : trackerHits) {
    seedTrack.addToTrackerHits(hit);
  }
  seedTrack.addToTrackStates(seedTrackState);
}

/**
 * @brief Owns the ACTS Combinatorial Kalman Filter and runs it over seeds.
 *
 * Owns the event-independent propagators and CKF (built once) and runs the
 * per-seed track finding/smoothing/conversion loop shared by the CKF tracking
 * algorithms. Each found track is smoothed, extrapolated back to the perigee
 * at the IP so its AtIP edm4hep track state (in particular D0/Z0) is well
 * defined and, when enabled, extrapolated to the calorimeter face to add an
 * AtCalorimeter track state.
 *
 * Build one instance (e.g. in the algorithm's initialize()) and reuse it: a
 * single instance is shared by concurrent findTracks() calls, each of which
 * binds its own event-local measurements/source links and uses its own local
 * track containers; the shared propagators/CKF are read-only, and writes to
 * the output collection are serialised by the caller's mutex. Non-copyable and
 * non-movable (it holds the propagators by unique_ptr).
 *
 * The implementation lives in CKFRunner.cpp, compiled once into the
 * k4ActsTracking library, so the CKF is not instantiated again in every
 * algorithm that uses it.
 */
class CKFRunner {
public:
  struct Config {
    double chi2CutOff = 15;
    std::int32_t numMeasurementsCutOff = 10;
    double chi2CutOffOutlier = std::numeric_limits<double>::max();

    /// Run the CKF backward (outside-in): from the OUTER seed SP inward,
    /// through the seed's middle and inner SPs towards the beamline. When
    /// true, the caller must build seeds whose bound parameters live on the
    /// TOP (outer) SP's surface -- see estimateSeedParameters(..., propagateBackward).
    /// The found tracks are reversed after smoothing, so the output is
    /// ordered inside-out (AtFirstHit = innermost hit) in either direction.
    bool propagateBackward = false;
    bool extrapolateToCalo = false;
    std::size_t maxSteps = kDefaultMaxPropagationSteps;

    /// Whether a track that crosses both calorimeter sections gets an
    /// AtCalorimeter state for each of them. False (the default) keeps one
    /// state per track, at the first calo face the track reaches. See
    /// CaloStateAppender::addEndcapStateAfterBarrel.
    bool addEndcapCaloState = false;

    // Branch stopper: optionally terminate CKF branches early on too many
    // holes/outliers or low pT. Disabled by default (useBranchStopper = false).
    bool useBranchStopper = false;
    int bsMaxHoles = 2;
    int bsMaxOutliers = 2;
    int bsMinMeasurements = 6;
    double bsPtMin = 0.0; ///< GeV; <= 0 disables the pT branch stop
    int bsPtMinMeasurements = 3;

    /// Two-way CKF: after smoothing the first pass, run a second CKF pass in
    /// the opposite direction from the smoothed innermost/outermost state and
    /// splice its states onto the first-pass chain. The stitched track then
    /// goes through the same reference-surface extrapolation as the single-
    /// pass output. Disabled by default.
    bool doTwoWayCKF = false;

    /// Inflate the covariance passed to the second pass so the acceptance
    /// window is not artificially tight after smoothing. Ignored when
    /// doTwoWayCKF is false.
    bool inflateCovarianceTwoWay = true;

    /// Multiplicative inflation applied to the second-pass covariance when
    /// inflateCovarianceTwoWay is true.
    double twoWayInflateCovarianceFactor = 100.0;

    /// Surface the fitted tracks are extrapolated to for the AtIP track state.
    /// When null (the default) a PerigeeSurface at the origin is used, i.e. the
    /// standard collider beamline parameterisation (d0/z0). A telescope client
    /// can instead pass a plane perpendicular to the beam so that beam-parallel
    /// tracks - which never approach the z-axis line - still have a reachable,
    /// well-defined reference.
    std::shared_ptr<const Acts::Surface> referenceSurface{nullptr};

    /// Optional ACTS track selection, applied to each track after the
    /// extrapolation to the reference surface.
    std::optional<Acts::TrackSelector::Config> trackSelector{};
  };

  /// Builds the event-independent propagators and CKF once. The event-local
  /// measurements and source links are bound per call in findTracks().
  ///
  /// The CKF propagator, the reference-surface extrapolator and (optionally)
  /// the calorimeter-face propagator depend only on the tracking geometry and
  /// field, so they are constructed here and reused read-only across events
  /// and threads. Without the separate reference extrapolator the CKF tracks
  /// would keep their parameters at a measurement surface and the AtIP track
  /// state would come out degenerate (omega=nan, D0=Z0=0). The reference is a
  /// PerigeeSurface at the origin unless Config::referenceSurface overrides it
  /// (e.g. a plane perpendicular to the beam for a telescope geometry).
  CKFRunner(const IActsGeoSvc& geo, const Config& cfg);
  ~CKFRunner();

  CKFRunner(const CKFRunner&) = delete;
  CKFRunner(CKFRunner&&) = delete;
  CKFRunner& operator=(const CKFRunner&) = delete;
  CKFRunner& operator=(CKFRunner&&) = delete;

  /// Two-way CKF summary for the algorithm's finalize(): how many first-pass
  /// tracks got a second pass stitched on, and how many kept the single-pass
  /// track (the per-track reasons are debug messages in findTracks()).
  std::string twoWaySummary() const;

  /// Run the CKF over @p paramseeds using the event-local @p measurements and
  /// @p sourceLinks, appending found tracks to @p trackCollection.
  /// @param msg Messaging of the calling component (usually the algorithm),
  ///            used for the debug/warning output of the track loop.
  /// @param caloMonitor Optional counters for the calorimeter-face extrapolation.
  void findTracks(const CommonMessagingBase& msg, const ACTSTracking::MeasurementContainer& measurements,
                  const ACTSTracking::SourceLinkContainer& sourceLinks, const ACTSTracking::HitContainer& hits,
                  const std::vector<Acts::BoundTrackParameters>& paramseeds,
                  Acts::MagneticFieldProvider::Cache& magCache, edm4hep::TrackCollection& trackCollection,
                  std::mutex& trackMutex, const CaloExtrapMonitor* caloMonitor = nullptr) const;

private:
  static Acts::MeasurementSelector::Config makeSelectorConfig(const Config& cfg);

  /// CKF branch stopper: drop a branch whose |pT| falls below threshold, or
  /// that has too many holes/outliers (keeping it if it already has enough
  /// measurements). Only connected when Config::useBranchStopper is set.
  Acts::CombinatorialKalmanFilterBranchStopperResult
  branchStopper(const CKFTrackContainer::TrackProxy& track, const CKFTrackContainer::TrackStateProxy& trackState) const;

  const IActsGeoSvc& m_geo;
  Acts::GeometryContext m_geoCtx;
  Acts::MagneticFieldContext m_magCtx{};
  Acts::CalibrationContext m_calCtx{};
  std::size_t m_maxSteps = kDefaultMaxPropagationSteps;
  bool m_propagateBackward = false;
  bool m_useBranchStopper = false;
  int m_bsMaxHoles = 2;
  int m_bsMaxOutliers = 2;
  int m_bsMinMeasurements = 6;
  double m_bsPtMin = 0.0;
  int m_bsPtMinMeasurements = 3;
  bool m_doTwoWayCKF = false;
  bool m_inflateCovarianceTwoWay = true;
  double m_twoWayInflateCovarianceFactor = 100.0;
  Acts::MeasurementSelector::Config m_measSelConfig;

  std::unique_ptr<CombKalmanFilter> m_trackFinder;
  std::shared_ptr<const Acts::Surface> m_referenceSurface;
  std::unique_ptr<CKFPropagator> m_extrapolator;
  CaloStateAppender m_caloAppender;
  std::optional<Acts::TrackSelector> m_trackSelector;

  // Two-way CKF counters: findTracks() is const and runs concurrently.
  mutable std::atomic<std::size_t> m_nTwoWayStitched{0};
  mutable std::atomic<std::size_t> m_nTwoWayFallbacks{0};
};

} // namespace ACTSTracking
