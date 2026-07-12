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

// k4ActsTracking
#include "k4ActsTracking/CKFRunner.hxx"
#include "k4ActsTracking/IActsGeoSvc.h"
#include "k4ActsTracking/Measurement.hxx"
#include "k4ActsTracking/SourceLink.hxx"

// k4FWCore
#include <k4FWCore/GaudiChecks.h>
#include <k4FWCore/Transformer.h>

// edm4hep
#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackerHit.h>
#include <edm4hep/TrackerHitPlaneCollection.h>

// podio
#include <podio/ObjectID.h>

// Gaudi
#include <Gaudi/Property.h>
#include <GaudiKernel/SmartIF.h>

// ACTS
#include <Acts/Definitions/Algebra.hpp>
#include <Acts/Definitions/Units.hpp>
#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/Utilities/CalibrationContext.hpp>

// TBB
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

#include <fmt/ostream.h>

// Standard
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <ranges>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief CKF tracking seeded from an existing user-defined track container.
 *
 * Closely follows CKFTrackingAlg, but instead of running the internal ACTS
 * seed finder it builds the seed track parameters from an input
 * edm4hep::TrackCollection (e.g. track candidates produced by an upstream
 * pattern-recognition stage). Each input track defines a group of tracker hits;
 * the innermost / middle / outermost of those hits (ordered by radius) form a
 * three-point seed from which initial Acts::BoundTrackParameters are estimated.
 * Those parameters then feed the same Combinatorial Kalman Filter used by
 * CKFTrackingAlg (shared via ACTSTracking::CKFRunner).
 */
struct CKFTrackingFromSeedsAlg final
    : k4FWCore::MultiTransformer<std::tuple<edm4hep::TrackCollection, edm4hep::TrackCollection>(
          const edm4hep::TrackerHitPlaneCollection&, const edm4hep::TrackCollection&)> {
  CKFTrackingFromSeedsAlg(const std::string& name, ISvcLocator* svcLoc);

  StatusCode initialize() override;

  StatusCode finalize() override;

  std::tuple<edm4hep::TrackCollection, edm4hep::TrackCollection> operator()(
      const edm4hep::TrackerHitPlaneCollection& trackerHitCollection,
      const edm4hep::TrackCollection&           seedTrackCollection) const override;

private:
  /// @name Run control
  ///@{
  Gaudi::Property<bool> m_propagateBackward{this, "PropagateBackward", false, "Extrapolates tracks towards beamline."};
  Gaudi::Property<bool> m_extrapolateToCalo{
      this, "ExtrapolateToCalo", true,
      "Extrapolate fitted tracks to the calorimeter face and add an AtCalorimeter track state."};
  Gaudi::Property<unsigned int> m_minSeedHits{this, "MinSeedHits", 3,
                                              "Minimum number of hits an input track must have to seed the CKF."};
  ///@}

  /// @name Track-fit initial error estimates
  ///@{
  Gaudi::Property<double>  m_initialTrackError_pos{this, "InitialTrackError_Pos", 10 * Acts::UnitConstants::um,
                                                  "Initial track error for local position."};
  Gaudi::Property<double>  m_initialTrackError_phi{this, "InitialTrackError_Phi", 1 * Acts::UnitConstants::degree,
                                                  "Initial track error for phi."};
  Gaudi::Property<double>  m_initialTrackError_relP{this, "InitialTrackError_RelP", 0.25,
                                                   "Initial track error for momentum (relative)."};
  Gaudi::Property<double>  m_initialTrackError_lambda{this, "InitialTrackError_Lambda", 1 * Acts::UnitConstants::degree,
                                                     "Initial track error for lambda."};
  Gaudi::Property<double>  m_initialTrackError_time{this, "InitialTrackError_Time", 100 * Acts::UnitConstants::ns,
                                                   "Initial track error for time."};
  Gaudi::Property<double>  m_CKF_chi2CutOff{this, "CKF_Chi2CutOff", 15, "Maximum local chi2 contribution."};
  Gaudi::Property<int32_t> m_CKF_numMeasurementsCutOff{this, "CKF_NumMeasurementsCutOff", 10,
                                                       "Maximum measurements on a single surface."};
  ///@}

  /// @name Multi-threading
  ///@{
  Gaudi::Property<int> m_numThreads{this, "NumThreads", 1, "Number of threads for internal TBB parallelism."};
  ///@}

  SmartIF<IActsGeoSvc> m_actsGeoSvc;

  // Shared Combinatorial Kalman Filter, built once in initialize() and reused
  // read-only across events and threads.
  std::optional<ACTSTracking::CKFRunner> m_ckfRunner{};

  // Calorimeter-face extrapolation monitoring, updated by the shared CKFRunner
  // and summarised in finalize().
  ACTSTracking::CaloExtrapMonitor m_caloMonitor{};

  mutable std::mutex m_seedMutex{};
  mutable std::mutex m_trackMutex{};
};

DECLARE_COMPONENT(CKFTrackingFromSeedsAlg)

// ---------------------------------------------------------------------------
// Constructor / initialize
// ---------------------------------------------------------------------------

CKFTrackingFromSeedsAlg::CKFTrackingFromSeedsAlg(const std::string& name, ISvcLocator* svcLoc)
    : MultiTransformer(name, svcLoc,
                       {KeyValue("InputTrackerHitCollection", "TrackerHits"),
                        KeyValue("InputSeedTrackCollection", "SeedTrackCandidates")},
                       {KeyValue("OutputSeedCollection", "SeedTracks"), KeyValue("OutputTrackCollection", "Tracks")}) {}

StatusCode CKFTrackingFromSeedsAlg::initialize() {
  m_actsGeoSvc = svcLoc()->service<IActsGeoSvc>("ActsGeoSvc");
  K4_GAUDI_CHECK(m_actsGeoSvc);

  if (m_extrapolateToCalo && m_actsGeoSvc->caloSurfaceGeoIds().empty()) {
    warning() << "ExtrapolateToCalo requested but ActsGeoSvc provides no calorimeter-face surfaces; "
                 "no AtCalorimeter track states will be produced."
              << endmsg;
  }

  m_ckfRunner.emplace(*m_actsGeoSvc,
                      ACTSTracking::CKFRunner::Config{.chi2CutOff            = m_CKF_chi2CutOff,
                                                      .numMeasurementsCutOff = m_CKF_numMeasurementsCutOff,
                                                      .propagateBackward     = m_propagateBackward,
                                                      .extrapolateToCalo     = m_extrapolateToCalo});

  return StatusCode::SUCCESS;
}

StatusCode CKFTrackingFromSeedsAlg::finalize() {
  if (m_extrapolateToCalo) {
    info() << m_caloMonitor.summary() << endmsg;
  }
  return StatusCode::SUCCESS;
}

// ---------------------------------------------------------------------------
// Event processing
// ---------------------------------------------------------------------------

std::tuple<edm4hep::TrackCollection, edm4hep::TrackCollection> CKFTrackingFromSeedsAlg::operator()(
    const edm4hep::TrackerHitPlaneCollection& trackerHitCollection,
    const edm4hep::TrackCollection&           seedTrackCollection) const {
  edm4hep::TrackCollection seedCollection;
  edm4hep::TrackCollection trackCollection;

  // Default-construct ACTS contexts
  const Acts::GeometryContext      geoCtx = Acts::GeometryContext::dangerouslyDefaultConstruct();
  const Acts::MagneticFieldContext magCtx{};

  ACTSTracking::SourceLinkContainer  sourceLinks;
  ACTSTracking::MeasurementContainer measurements;
  ACTSTracking::HitContainer         hitContainer;

  // Build measurements + source links for all hits, and keep the source link of
  // each hit so the input track candidates can be turned into seeds.
  std::unordered_map<std::uint64_t, ACTSTracking::SourceLink> slByHit;
  slByHit.reserve(trackerHitCollection.size());

  ACTSTracking::prepareTrackerHits(
      *this, *m_actsGeoSvc, geoCtx, trackerHitCollection, measurements, sourceLinks, hitContainer, m_numThreads.value(),
      [&](const edm4hep::TrackerHitPlane& hit, const ACTSTracking::SourceLink& sl, const Acts::Vector3& /*globalPos*/,
          const Acts::Surface& /*surface*/,
          const Acts::SquareMatrix2& /*localCov*/) { slByHit.emplace(ACTSTracking::trackerHitKey(hit), sl); });

  debug() << fmt::format("Created {} sourceLinks from {} hits", sourceLinks.size(), trackerHitCollection.size())
          << endmsg;

  // Per-hit seed info: global position + transverse radius, resolved once per
  // usable hit so the seed triplet can be ordered by radius without recomputing
  // positions inside the sort comparator.
  struct SeedHit {
    Acts::Vector3            pos;
    double                   r;
    ACTSTracking::SourceLink sl;
  };

  // Parallelise over the input seed tracks: each task turns a contiguous chunk
  // of candidates into three-point seeds + initial parameters, then runs the CKF
  // once over that chunk (so the ACTS track containers are reused across the
  // chunk's seeds). Shared inputs (slByHit, measurements, sourceLinks) are
  // read-only here; the podio output collections are guarded by the mutexes.
  auto seedAndTrack = [&](const tbb::blocked_range<std::size_t>& range) {
    // Per-task magnetic-field cache: the cache is mutable state and must not be
    // shared between threads.
    auto magCacheLocal = m_actsGeoSvc->magneticField()->makeCache(magCtx);

    std::vector<Acts::BoundTrackParameters> paramseeds;
    paramseeds.reserve(range.size());

    for (std::size_t i = range.begin(); i != range.end(); ++i) {
      const auto seedTrack = seedTrackCollection[i];

      std::vector<SeedHit> hits;
      hits.reserve(seedTrack.trackerHits_size());
      for (const auto& hit : seedTrack.getTrackerHits()) {
        auto it = slByHit.find(ACTSTracking::trackerHitKey(hit));
        if (it == slByHit.end()) {
          continue;
        }
        const edm4hep::Vector3d p = hitContainer[it->second.index()].getPosition();
        hits.push_back({Acts::Vector3(p.x, p.y, p.z), std::hypot(p.x, p.y), it->second});
      }

      if (hits.size() < m_minSeedHits) {
        debug() << "Skipping input track with " << hits.size() << " usable hits (< " << m_minSeedHits.value() << ")."
                << endmsg;
        continue;
      }

      // Order by radius and pick innermost / middle / outermost as the seed triplet.
      std::sort(hits.begin(), hits.end(), [](const SeedHit& a, const SeedHit& b) { return a.r < b.r; });

      const SeedHit& bottom = hits.front();
      const SeedHit& middle = hits[hits.size() / 2];
      const SeedHit& top    = hits.back();

      const Acts::Surface* bottomSurface = m_actsGeoSvc->trackingGeometry()->findSurface(bottom.sl.geometryId());
      if (bottomSurface == nullptr) {
        warning() << "Surface with geoID " << bottom.sl.geometryId() << " not found in tracking geometry" << endmsg;
        continue;
      }

      std::optional<Acts::BoundTrackParameters> paramseed = ACTSTracking::estimateSeedParameters(
          *this, *m_actsGeoSvc, geoCtx, *bottomSurface, bottom.pos, middle.pos, top.pos,
          hitContainer[bottom.sl.index()].getTime(), magCacheLocal, m_initialTrackError_pos, m_initialTrackError_phi,
          m_initialTrackError_lambda, m_initialTrackError_relP, m_initialTrackError_time);
      if (!paramseed) {
        continue;
      }
      paramseeds.push_back(*paramseed);

      auto seedTrackState = ACTSTracking::makeSeedTrackState(*this, *m_actsGeoSvc, geoCtx, *paramseed, magCacheLocal);

      ACTSTracking::appendSeedTrack(
          seedCollection, m_seedMutex, seedTrackState,
          hits | std::views::transform([&](const SeedHit& h) { return hitContainer[h.sl.index()]; }));
    }

    // One CKF pass for the whole chunk.
    m_ckfRunner->findTracks(*this, measurements, sourceLinks, hitContainer, paramseeds, magCacheLocal, trackCollection,
                            m_trackMutex, &m_caloMonitor);
  };

  const tbb::blocked_range<std::size_t> fullRange(0, seedTrackCollection.size());
  if (m_numThreads.value() > 1) {
    tbb::task_arena arena(m_numThreads.value());
    arena.execute([&] { tbb::parallel_for(fullRange, seedAndTrack); });
  } else {
    seedAndTrack(fullRange);
  }

  debug() << "Track Collection Size: " << trackCollection.size() << endmsg;
  return std::make_tuple(std::move(seedCollection), std::move(trackCollection));
}
