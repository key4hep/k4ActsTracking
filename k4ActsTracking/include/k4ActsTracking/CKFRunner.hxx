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
#include "k4ActsTracking/MeasurementCalibrator.hxx"
#include "k4ActsTracking/RunnerCommon.hxx"
#include "k4ActsTracking/SourceLink.hxx"

// edm4hep
#include <edm4hep/MutableTrack.h>
#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackState.h>
#include <edm4hep/TrackerHit.h>
#include <edm4hep/TrackerHitPlaneCollection.h>
#include <edm4hep/Vector3f.h>

// podio
#include <podio/ObjectID.h>

// ACTS
#include <Acts/Definitions/Units.hpp>
#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/EventData/ParticleHypothesis.hpp>
#include <Acts/EventData/TrackStateType.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>
#include <Acts/EventData/VectorTrackContainer.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Propagator/EigenStepper.hpp>
#include <Acts/Propagator/Navigator.hpp>
#include <Acts/Propagator/Propagator.hpp>
#include <Acts/Seeding/EstimateTrackParamsFromSeed.hpp>
#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/Surfaces/PlaneSurface.hpp>
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>
#include <Acts/TrackFinding/MeasurementSelector.hpp>
#include <Acts/TrackFinding/TrackStateCreator.hpp>
#include <Acts/TrackFitting/GainMatrixUpdater.hpp>
#include <Acts/Utilities/TrackHelpers.hpp>

// TBB
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>

#include <fmt/format.h>

// Standard
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ACTSTracking {

  /// CKF-specific type aliases (shared runner types live in RunnerCommon.hxx).
  using CKFTrackFinderOptions = Acts::CombinatorialKalmanFilterOptions<CKFTrackContainer>;
  using CombKalmanFilter      = Acts::CombinatorialKalmanFilter<CKFPropagator, CKFTrackContainer>;

  /**
   * @brief Thread-safe counters for the calorimeter-face extrapolation.
   *
   * Shared by the CKF tracking algorithms: CKFRunner::findTracks updates the
   * counters (through a const pointer, hence the mutable atomics) and each
   * algorithm reports summary() in finalize().
   */
  struct CaloExtrapMonitor {
    mutable std::atomic<std::size_t> attempts{0};      ///< tracks with a usable start state
    mutable std::atomic<std::size_t> noStartState{0};  ///< tracks without a measured smoothed state
    mutable std::atomic<std::size_t> notReached{0};    ///< propagation did not reach a calo face
    mutable std::atomic<std::size_t> propFailed{0};    ///< propagation itself failed
    mutable std::atomic<std::size_t> ok{0};            ///< reached a calo face

    std::string summary() const {
      const std::size_t a        = attempts.load();
      const std::size_t o        = ok.load();
      const std::size_t nr       = notReached.load();
      const std::size_t pf       = propFailed.load();
      const std::size_t ns       = noStartState.load();
      const std::size_t failed   = nr + pf;
      const double      failRate = a > 0 ? static_cast<double>(failed) / static_cast<double>(a) : 0.0;
      return fmt::format(
          "Calorimeter-face extrapolation summary: {} attempts, {} reached the face, {} failed "
          "({:.2f}%: {} not reached, {} propagation errors); {} tracks had no measured smoothed start state.",
          a, o, failed, 100.0 * failRate, nr, pf, ns);
    }
  };

  /**
   * @brief Build the ACTS measurements and source links for a set of tracker hits.
   *
   * Shared hit-conversion loop of the CKF tracking algorithms:
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

  /**
   * @brief Convert estimated seed parameters into an edm4hep seed TrackState.
   */
  template <class Alg>
  edm4hep::TrackState makeSeedTrackState(const Alg& /*alg*/, const IActsGeoSvc& geo,
                                         const Acts::GeometryContext&        geoCtx,
                                         const Acts::BoundTrackParameters&   paramseed,
                                         Acts::MagneticFieldProvider::Cache& magCache) {
    const Acts::Surface&     surface = paramseed.referenceSurface();
    const Acts::BoundVector& params  = paramseed.parameters();

    Acts::Vector3 globalPos =
        surface.localToGlobal(geoCtx, {params[Acts::eBoundLoc0], params[Acts::eBoundLoc1]}, {0, 0, 0});
    Acts::Result<Acts::Vector3> hitField = geo.magneticField()->getField(globalPos, magCache);
    if (!hitField.ok()) {
      throw std::runtime_error("Field lookup error: " + std::to_string(hitField.error().value()));
    }

    return ACTSTracking::ACTS2edm4hep_trackState(edm4hep::TrackState::AtFirstHit, geoCtx, paramseed,
                                                 (*hitField)[2] / Acts::UnitConstants::T);
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
   */
  class CKFRunner {
  public:
    struct Config {
      double       chi2CutOff            = 15;
      std::int32_t numMeasurementsCutOff = 10;
      double       chi2CutOffOutlier     = std::numeric_limits<double>::max();
      bool         propagateBackward     = false;
      bool         extrapolateToCalo     = false;
      std::size_t  maxSteps              = 10000;

      // Branch stopper: optionally terminate CKF branches early on too many
      // holes/outliers or low pT. Disabled by default (useBranchStopper = false).
      bool   useBranchStopper    = false;
      int    bsMaxHoles          = 2;
      int    bsMaxOutliers       = 2;
      int    bsMinMeasurements   = 6;
      double bsPtMin             = 0.0;  ///< GeV; <= 0 disables the pT branch stop
      int    bsPtMinMeasurements = 3;

      /// Surface the fitted tracks are extrapolated to for the AtIP track state.
      /// When null (the default) a PerigeeSurface at the origin is used, i.e. the
      /// standard collider beamline parameterisation (d0/z0). A telescope client
      /// can instead pass a plane perpendicular to the beam so that beam-parallel
      /// tracks - which never approach the z-axis line - still have a reachable,
      /// well-defined reference.
      std::shared_ptr<const Acts::Surface> referenceSurface{nullptr};

      /// Telescope (field-free tracker) clients only. When true, the otherwise
      /// unconstrained q/p of each fitted track is determined from the physics
      /// constraint that the track originates at the target (the reference-surface
      /// centre): q/p is solved so the field-aware back-extrapolation lands on the
      /// target in the bending plane (local x = eBoundLoc0). A downstream tracker
      /// measures the position and direction but not the momentum (no curvature in
      /// the field-free region), so without this the seed's nominal q/p survives
      /// and the back-extrapolation mis-lands. Default false keeps the collider
      /// behaviour (q/p from the seed/CKF fit) byte-identical.
      bool constrainMomentumToTarget = false;

      /// Telescope clients only, and only meaningful with constrainMomentumToTarget.
      /// z [mm] of a plane inside the bending magnet where a momentum-carrying
      /// track state (edm4hep AtOther) is added so the reconstructed p is
      /// recoverable: p [GeV] = 0.3 * |B| [T] / (|omega| [1/mm] * 1e3). This is
      /// necessary because the AtIP omega is degenerate wherever the field
      /// vanishes (e.g. the LUXE reference plane at z=0). <= 0 disables it.
      double momentumReportZ = 0.0;
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
    CKFRunner(const IActsGeoSvc& geo, const Config& cfg)
        : m_geo(geo),
          m_geoCtx(Acts::GeometryContext::dangerouslyDefaultConstruct()),
          m_maxSteps(cfg.maxSteps),
          m_propagateBackward(cfg.propagateBackward),
          m_useBranchStopper(cfg.useBranchStopper),
          m_bsMaxHoles(cfg.bsMaxHoles),
          m_bsMaxOutliers(cfg.bsMaxOutliers),
          m_bsMinMeasurements(cfg.bsMinMeasurements),
          m_bsPtMin(cfg.bsPtMin),
          m_bsPtMinMeasurements(cfg.bsPtMinMeasurements),
          m_measSelConfig(makeSelectorConfig(cfg)),
          m_trackFinder(std::make_unique<CombKalmanFilter>(makePropagator(geo, false))),
          m_referenceSurface(cfg.referenceSurface
                                 ? cfg.referenceSurface
                                 : Acts::Surface::makeShared<Acts::PerigeeSurface>(Acts::Vector3::Zero())),
          m_constrainMomentum(cfg.constrainMomentumToTarget),
          m_extrapolator(std::make_unique<CKFPropagator>(makePropagator(geo, false))) {
      // In-magnet plane where the momentum-carrying track state is added (only
      // for telescope clients that both constrain the momentum and request it).
      if (m_constrainMomentum && cfg.momentumReportZ > 0.0) {
        Acts::Transform3 reportTransform = Acts::Transform3::Identity();
        reportTransform.translation()    = Acts::Vector3(0, 0, cfg.momentumReportZ * Acts::UnitConstants::mm);
        m_momentumReportSurface          = Acts::Surface::makeShared<Acts::PlaneSurface>(reportTransform);
      }
      // The calorimeter inner-face surfaces are passive surfaces of the tracking
      // geometry, so the calo propagator's navigator must resolve passive
      // surfaces. Only built when requested and when the geometry provides them.
      if (cfg.extrapolateToCalo && !geo.caloSurfaceGeoIds().empty()) {
        m_caloPropagator = std::make_unique<ACTSTracking::CaloFacePropagator>(makePropagator(geo, true));
      }
    }

    CKFRunner(const CKFRunner&)            = delete;
    CKFRunner(CKFRunner&&)                 = delete;
    CKFRunner& operator=(const CKFRunner&) = delete;
    CKFRunner& operator=(CKFRunner&&)      = delete;

    /// Run the CKF over @p paramseeds using the event-local @p measurements and
    /// @p sourceLinks, appending found tracks to @p trackCollection.
    /// @param caloMonitor Optional counters for the calorimeter-face extrapolation.
    template <class Alg>
    void findTracks(const Alg& alg, const ACTSTracking::MeasurementContainer& measurements,
                    const ACTSTracking::SourceLinkContainer& sourceLinks, const ACTSTracking::HitContainer& hits,
                    const std::vector<Acts::BoundTrackParameters>& paramseeds,
                    Acts::MagneticFieldProvider::Cache& magCache, edm4hep::TrackCollection& trackCollection,
                    std::mutex& trackMutex, const CaloExtrapMonitor* caloMonitor = nullptr) const {
      alg.debug() << "Starting CKF track finding with " << paramseeds.size() << " seeds." << endmsg;

      // Bind the event-local measurements / source links. These, and the
      // extensions/options that point at them, are per-call locals so concurrent
      // findTracks() calls stay independent; the propagators, CKF and selector
      // config are shared read-only state.
      Acts::GainMatrixUpdater             kfUpdater;
      Acts::MeasurementSelector           measSel{m_measSelConfig};
      ACTSTracking::MeasurementCalibrator measCal{measurements};
      ACTSTracking::SourceLinkAccessor    slAccessor;
      slAccessor.container = &sourceLinks;

      TrackStateCreatorType trackStateCreator;
      trackStateCreator.sourceLinkAccessor.template connect<&ACTSTracking::SourceLinkAccessor::range>(&slAccessor);
      trackStateCreator.calibrator.template connect<&ACTSTracking::MeasurementCalibrator::calibrate>(&measCal);
      trackStateCreator.measurementSelector
          .template connect<&Acts::MeasurementSelector::select<Acts::VectorMultiTrajectory>>(&measSel);

      Acts::CombinatorialKalmanFilterExtensions<CKFTrackContainer> extensions;
      extensions.updater.connect<&Acts::GainMatrixUpdater::operator()<Acts::VectorMultiTrajectory>>(&kfUpdater);
      extensions.createTrackStates.template connect<&TrackStateCreatorType::createTrackStates>(&trackStateCreator);
      if (m_useBranchStopper) {
        extensions.branchStopper.template connect<&CKFRunner::branchStopper>(this);
      }

      Acts::PropagatorPlainOptions pOptions{m_geoCtx, m_magCtx};
      pOptions.maxSteps = m_maxSteps;
      if (m_propagateBackward) {
        pOptions.direction = Acts::Direction::Backward();
      }
      const CKFTrackFinderOptions ckfOptions(m_geoCtx, m_magCtx, m_calCtx, extensions, pOptions);

      auto              trackContainer      = std::make_shared<Acts::VectorTrackContainer>();
      auto              trackStateContainer = std::make_shared<Acts::VectorMultiTrajectory>();
      CKFTrackContainer tracks(trackContainer, trackStateContainer);

      // Access the track finder as const so concurrent findTracks() calls use
      // the read-only overload (the finder and propagators are shared state).
      const CombKalmanFilter& trackFinder = *m_trackFinder;

      for (std::size_t iseed = 0; iseed < paramseeds.size(); ++iseed) {
        tracks.clear();
        auto result = trackFinder.findTracks(paramseeds.at(iseed), ckfOptions, tracks);
        if (result.ok()) {
          const auto& fitOutput = result.value();
          for (const CKFTrackContainer::TrackProxy& trackItem : fitOutput) {
            auto trackTip = tracks.makeTrack();
            trackTip.copyFrom(trackItem);
            auto smoothResult = Acts::smoothTrack(m_geoCtx, trackTip);
            if (!smoothResult.ok()) {
              alg.warning() << "Track smoothing error: " << smoothResult.error() << endmsg;
              continue;
            }

            // Telescope clients: the field-free tracker fixes the track's
            // position and direction but not its momentum (no curvature), so the
            // seed's nominal q/p survives the fit. Solve q/p from the requirement
            // that the field-aware back-extrapolation lands on the target (the
            // reference-surface centre) in the bending plane, and write it into
            // the smoothed states so the reference extrapolation below carries the
            // constrained momentum. Gated on Config::constrainMomentumToTarget;
            // the collider path is byte-identical when it is false.
            std::optional<Acts::BoundTrackParameters> innermost;
            std::optional<double>                     solvedQOverP;
            if (m_constrainMomentum) {
              innermost = innermostMeasuredParams(trackTip);
              if (innermost) {
                solvedQOverP = solveMomentumToTarget(*innermost);
                if (solvedQOverP) {
                  constrainTrackMomentum(trackTip, *solvedQOverP);
                } else {
                  alg.debug() << "Momentum constraint did not converge; keeping the seed q/p for this track." << endmsg;
                }
              }
            }

            // Extrapolate the smoothed track to the reference surface (the IP
            // perigee by default, or a beam-perpendicular plane for telescope
            // clients) so its track-level parameters (and hence the AtIP edm4hep
            // TrackState) are defined there, not at the innermost measurement
            // surface. Must happen before ACTS2edm4hep_track, which fills the
            // AtIP state.
            typename CKFPropagator::template Options<> exOptions(m_geoCtx, m_magCtx);
            exOptions.maxSteps                = m_maxSteps;
            const CKFPropagator& extrapolator = *m_extrapolator;
            auto                 exResult     = Acts::extrapolateTrackToReferenceSurface(
                trackTip, *m_referenceSurface, extrapolator, exOptions, Acts::TrackExtrapolationStrategy::firstOrLast);
            if (!exResult.ok()) {
              alg.warning() << "Reference-surface extrapolation error: " << exResult.error() << endmsg;
              continue;
            }

            auto track = ACTSTracking::ACTS2edm4hep_track(m_geoCtx, trackTip, hits, m_geo.magneticField(), magCache);

            addCaloState(alg, trackTip, track, magCache, caloMonitor);

            // Momentum-carrying in-magnet track state (telescope reporting): the
            // AtIP omega is degenerate where the field vanishes, so add a state
            // inside the bending magnet whose omega encodes the reconstructed p.
            if (m_momentumReportSurface && innermost && solvedQOverP) {
              addMomentumReportState(alg, *innermost, *solvedQOverP, track, magCache);
            }

            {
              std::lock_guard lock{trackMutex};
              trackCollection.push_back(track);
            }
          }
        } else {
          alg.warning() << "Track fit error: " << result.error() << endmsg;
        }
      }
    }

  private:
    using TrackStateCreatorType =
        Acts::TrackStateCreator<ACTSTracking::SourceLinkAccessor::Iterator, CKFTrackContainer>;

    static Acts::MeasurementSelector::Config makeSelectorConfig(const Config& cfg) {
      return {{Acts::GeometryIdentifier(),
               {{}, {cfg.chi2CutOff}, {static_cast<std::size_t>(cfg.numMeasurementsCutOff)}, {cfg.chi2CutOffOutlier}}}};
    }

    // ---- Telescope momentum constraint (only used when m_constrainMomentum) ----

    /// Bound parameters of the innermost measured, smoothed state (the hit closest
    /// to the target), used as the fixed position+direction ray whose momentum is
    /// solved for. Empty if the track has no usable measured state.
    template <class TrackProxy>
    std::optional<Acts::BoundTrackParameters> innermostMeasuredParams(const TrackProxy& trackTip) const {
      std::optional<Acts::BoundTrackParameters> params;
      // trackStatesReversed() runs outermost -> innermost, so keep overwriting to
      // end on the innermost measured state.
      for (const auto& state : trackTip.trackStatesReversed()) {
        const auto flags = state.typeFlags();
        if (state.hasSmoothed() && flags.test(Acts::TrackStateFlag::HasMeasurement) &&
            !flags.test(Acts::TrackStateFlag::IsOutlier)) {
          params.emplace(state.referenceSurface().getSharedPtr(), state.smoothed(), state.smoothedCovariance(),
                         trackTip.particleHypothesis());
        }
      }
      return params;
    }

    /// Propagate @p start, reparameterised with @p qOverP but the same position and
    /// direction, to @p target using the field-aware extrapolator. Returns the
    /// bound parameters on @p target, or nullopt on propagation failure.
    std::optional<Acts::BoundTrackParameters> propagateWithQOverP(const Acts::BoundTrackParameters& start,
                                                                  double qOverP, const Acts::Surface& target) const {
      Acts::BoundVector pars   = start.parameters();
      pars[Acts::eBoundQOverP] = qOverP;
      const Acts::BoundTrackParameters trial(start.referenceSurface().getSharedPtr(), pars, start.covariance(),
                                             start.particleHypothesis());

      typename CKFPropagator::template Options<> opts(m_geoCtx, m_magCtx);
      opts.maxSteps  = m_maxSteps;
      opts.direction = Acts::Direction::Backward();  // target is upstream of the tracker

      auto result = m_extrapolator->propagate(trial, target, opts);
      if (!result.ok()) {
        return std::nullopt;
      }
      const auto& output = result.value();
      if (!output.endParameters.has_value()) {
        return std::nullopt;
      }
      return output.endParameters.value();
    }

    /// Solve q/p so the field-aware back-extrapolation of @p start lands on the
    /// target (the reference-surface centre) in the bending plane (loc0 -> 0). The
    /// bending-plane miss is ~linear in q/p (deflection ∝ ∫B·dl / p), so a few
    /// secant iterations converge. |p| is clamped to a sane range. Returns the
    /// solved q/p [1/GeV], or nullopt if a propagation failed.
    std::optional<double> solveMomentumToTarget(const Acts::BoundTrackParameters& start) const {
      constexpr int    kMaxIter = 12;
      constexpr double kTolMm   = 0.02;  // sub-resolution landing accuracy
      const double     seedQ    = start.parameters()[Acts::eBoundQOverP];
      const double     qSign    = (seedQ >= 0.0) ? 1.0 : -1.0;
      const double     qAbsMin  = 1.0 / 200.0;  // |p| <= 200 GeV
      const double     qAbsMax  = 1.0 / 0.05;   // |p| >= 0.05 GeV
      auto             clampQ   = [&](double q) { return qSign * std::clamp(std::abs(q), qAbsMin, qAbsMax); };
      auto             missX    = [&](double q) -> std::optional<double> {
        auto p = propagateWithQOverP(start, q, *m_referenceSurface);
        if (!p) {
          return std::nullopt;
        }
        return p->parameters()[Acts::eBoundLoc0];
      };

      double q0 = clampQ(seedQ != 0.0 ? seedQ : qSign * qAbsMin);
      auto   m0 = missX(q0);
      if (!m0) {
        return std::nullopt;
      }
      double f0 = *m0;
      if (std::abs(f0) < kTolMm) {
        return q0;
      }

      double q1 = clampQ(q0 * 1.5);
      if (q1 == q0) {
        q1 = clampQ(q0 + qSign * 0.02);
      }
      auto m1 = missX(q1);
      if (!m1) {
        return std::nullopt;
      }
      double f1 = *m1;

      for (int it = 0; it < kMaxIter && std::abs(f1) >= kTolMm; ++it) {
        const double denom = f1 - f0;
        if (std::abs(denom) < 1e-12) {
          break;
        }
        const double q2 = clampQ(q1 - f1 * (q1 - q0) / denom);
        auto         m2 = missX(q2);
        if (!m2) {
          return std::nullopt;
        }
        q0 = q1;
        f0 = f1;
        q1 = q2;
        f1 = *m2;
      }
      return q1;
    }

    /// Write the solved q/p into every state of the track. The tracker is
    /// field-free, so q/p is constant along it; setting it on all states makes the
    /// reference extrapolation (firstOrLast) and any later use carry the
    /// constrained momentum regardless of which end it starts from.
    template <class TrackProxy> void constrainTrackMomentum(TrackProxy& trackTip, double qOverP) const {
      for (auto state : trackTip.trackStatesReversed()) {
        if (state.hasSmoothed()) {
          state.smoothed()[Acts::eBoundQOverP] = qOverP;
        }
        if (state.hasFiltered()) {
          state.filtered()[Acts::eBoundQOverP] = qOverP;
        }
        if (state.hasPredicted()) {
          state.predicted()[Acts::eBoundQOverP] = qOverP;
        }
      }
    }

    /// Add a momentum-carrying edm4hep track state inside the bending magnet, where
    /// the field is non-zero, so the reconstructed p is recoverable even though the
    /// AtIP omega is degenerate at the (field-free) reference plane. The state's
    /// omega encodes p via |omega| [1/mm] = 0.3 * |B| [T] / p [GeV] * 1e-3, so
    /// p [GeV] = 0.3 * |B| / (|omega| * 1e3) with |B| the local dipole field.
    template <class Alg>
    void addMomentumReportState(const Alg& alg, const Acts::BoundTrackParameters& innermost, double qOverP,
                                edm4hep::MutableTrack& track, Acts::MagneticFieldProvider::Cache& magCache) const {
      auto atField = propagateWithQOverP(innermost, qOverP, *m_momentumReportSurface);
      if (!atField) {
        alg.debug() << "Momentum-report extrapolation into the magnet failed; no momentum state added." << endmsg;
        return;
      }
      const Acts::Vector3 posv     = atField->position(m_geoCtx);
      auto                fieldRes = m_geo.magneticField()->getField(posv, magCache);
      if (!fieldRes.ok()) {
        return;
      }
      const double Btesla = (*fieldRes).norm() / Acts::UnitConstants::T;
      const double absP   = (qOverP != 0.0) ? 1.0 / std::abs(qOverP) : 0.0;  // native q/p is 1/GeV
      if (!(absP > 0.0) || !(Btesla > 0.0)) {
        alg.debug() << "Momentum-report state skipped: no local field or degenerate q/p." << endmsg;
        return;
      }

      const Acts::Vector3 dir   = atField->direction();
      const double        omega = std::copysign(1.0, qOverP) * 0.3 * Btesla / absP * 1e-3;
      const double        phi   = std::atan2(dir.y(), dir.x());
      const double        tanL  = dir.z() / std::hypot(dir.x(), dir.y());

      edm4hep::TrackState state{};
      state.location  = edm4hep::TrackState::AtOther;
      state.D0        = 0.0f;
      state.Z0        = 0.0f;
      state.phi       = static_cast<float>(phi);
      state.omega     = static_cast<float>(omega);
      state.tanLambda = static_cast<float>(tanL);
      state.time      = 0.0f;
      state.referencePoint =
          edm4hep::Vector3f(static_cast<float>(posv.x()), static_cast<float>(posv.y()), static_cast<float>(posv.z()));
      track.addToTrackStates(state);
    }

    /// CKF branch stopper: drop a branch whose |pT| falls below threshold, or
    /// that has too many holes/outliers (keeping it if it already has enough
    /// measurements). Only connected when Config::useBranchStopper is set.
    Acts::CombinatorialKalmanFilterBranchStopperResult branchStopper(
        const CKFTrackContainer::TrackProxy& track, const CKFTrackContainer::TrackStateProxy& trackState) const {
      using Result = Acts::CombinatorialKalmanFilterBranchStopperResult;

      const int nMeas = static_cast<int>(track.nMeasurements());

      // pT branch stop: once the branch has enough measurements for the momentum
      // estimate to be reliable, drop it if |pT| has fallen below threshold.
      if (m_bsPtMin > 0.0 && nMeas >= m_bsPtMinMeasurements) {
        const auto&  params = trackState.hasFiltered() ? trackState.filtered() : trackState.predicted();
        const double theta  = params[Acts::eBoundTheta];
        const double qOverP = params[Acts::eBoundQOverP];
        if (qOverP != 0.0) {
          const double pt = std::abs(std::sin(theta) / qOverP);  // native momentum units
          if (pt < m_bsPtMin * Acts::UnitConstants::GeV) {
            return Result::StopAndDrop;
          }
        }
      }

      // Hole / outlier branch stop.
      const bool tooManyHoles    = static_cast<int>(track.nHoles()) > m_bsMaxHoles;
      const bool tooManyOutliers = static_cast<int>(track.nOutliers()) > m_bsMaxOutliers;
      if (!(tooManyHoles || tooManyOutliers)) {
        return Result::Continue;
      }
      // Keep the branch if it already has enough measurements, otherwise drop it.
      return (nMeas >= m_bsMinMeasurements) ? Result::StopAndKeep : Result::StopAndDrop;
    }

    /// Extrapolate the smoothed track to the calorimeter face and, on success,
    /// append an AtCalorimeter track state to @p track. No-op when calo
    /// extrapolation is disabled. Starts from the outermost smoothed state that
    /// carries a real measurement (closest to the calorimeter).
    template <class Alg, class TrackProxy>
    void addCaloState(const Alg& alg, const TrackProxy& trackTip, edm4hep::MutableTrack& track,
                      Acts::MagneticFieldProvider::Cache& magCache, const CaloExtrapMonitor* caloMonitor) const {
      if (!m_caloPropagator) {
        return;
      }

      std::optional<Acts::BoundTrackParameters> startParams;
      for (const auto& state : trackTip.trackStatesReversed()) {
        const auto flags = state.typeFlags();
        if (state.hasSmoothed() && flags.test(Acts::TrackStateFlag::HasMeasurement) &&
            !flags.test(Acts::TrackStateFlag::IsOutlier)) {
          startParams.emplace(state.referenceSurface().getSharedPtr(), state.smoothed(), state.smoothedCovariance(),
                              trackTip.particleHypothesis());
          break;
        }
      }

      if (!startParams) {
        if (caloMonitor) {
          ++caloMonitor->noStartState;
        }
        alg.debug() << "No measured smoothed state available; no AtCalorimeter state added for this track." << endmsg;
        return;
      }

      if (caloMonitor) {
        ++caloMonitor->attempts;
      }
      const ACTSTracking::CaloFacePropagator& caloPropagator = *m_caloPropagator;
      const auto caloResult = ACTSTracking::extrapolateToCaloFace(caloPropagator, *startParams,
                                                                  m_geo.caloSurfaceGeoIds(), m_geoCtx, m_magCtx);

      using ACTSTracking::CaloExtrapolationStatus;
      switch (caloResult.status) {
        case CaloExtrapolationStatus::Ok: {
          if (caloMonitor) {
            ++caloMonitor->ok;
          }
          const Acts::Vector3 caloPos  = caloResult.params->position(m_geoCtx);
          auto                fieldRes = m_geo.magneticField()->getField(caloPos, magCache);
          const double        Bz       = fieldRes.ok() ? (*fieldRes)[2] / Acts::UnitConstants::T : 0.0;
          // The calo-face parameters are local to the target surface;
          // ACTS2edm4hep_trackState re-expresses them at an ad-hoc perigee at the
          // calo-face position and sets the referencePoint accordingly.
          auto caloState = ACTSTracking::ACTS2edm4hep_trackState(edm4hep::TrackState::AtCalorimeter, m_geoCtx,
                                                                 *caloResult.params, Bz);
          track.addToTrackStates(caloState);
          break;
        }
        case CaloExtrapolationStatus::NotReached:
        case CaloExtrapolationStatus::NoSurfaces:
          if (caloMonitor) {
            ++caloMonitor->notReached;
          }
          alg.debug() << "Extrapolation to the calorimeter face did not reach a surface; "
                         "no AtCalorimeter state added for this track."
                      << endmsg;
          break;
        case CaloExtrapolationStatus::PropagationError:
          if (caloMonitor) {
            ++caloMonitor->propFailed;
          }
          alg.debug() << "Extrapolation to the calorimeter face failed during propagation; "
                         "no AtCalorimeter state added for this track."
                      << endmsg;
          break;
      }
    }

    const IActsGeoSvc&                m_geo;
    Acts::GeometryContext             m_geoCtx;
    Acts::MagneticFieldContext        m_magCtx{};
    Acts::CalibrationContext          m_calCtx{};
    std::size_t                       m_maxSteps            = 10000;
    bool                              m_propagateBackward   = false;
    bool                              m_useBranchStopper    = false;
    int                               m_bsMaxHoles          = 2;
    int                               m_bsMaxOutliers       = 2;
    int                               m_bsMinMeasurements   = 6;
    double                            m_bsPtMin             = 0.0;
    int                               m_bsPtMinMeasurements = 3;
    Acts::MeasurementSelector::Config m_measSelConfig;

    std::unique_ptr<CombKalmanFilter>                 m_trackFinder;
    std::shared_ptr<const Acts::Surface>              m_referenceSurface;
    bool                                              m_constrainMomentum = false;
    std::unique_ptr<CKFPropagator>                    m_extrapolator;
    std::shared_ptr<const Acts::Surface>              m_momentumReportSurface;
    std::unique_ptr<ACTSTracking::CaloFacePropagator> m_caloPropagator;
  };

}  // namespace ACTSTracking
