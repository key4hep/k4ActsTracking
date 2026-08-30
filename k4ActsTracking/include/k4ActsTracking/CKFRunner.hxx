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
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>
#include <Acts/TrackFinding/MeasurementSelector.hpp>
#include <Acts/TrackFinding/TrackStateCreator.hpp>
#include <Acts/TrackFitting/GainMatrixUpdater.hpp>
#include <Acts/Utilities/TrackHelpers.hpp>

// ActsPlugins: centralised ACTS -> EDM4hep conversion
#include <ActsPlugins/EDM4hep/EDM4hepUtil.hpp>

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
   * @brief Convert estimated seed parameters into an edm4hep seed TrackState.
   */
  template <class Alg>
  edm4hep::TrackState makeSeedTrackState(const Alg& /*alg*/, const IActsGeoSvc& geo,
                                         const Acts::GeometryContext&        geoCtx,
                                         const Acts::BoundTrackParameters&   paramseed,
                                         Acts::MagneticFieldProvider::Cache& magCache) {
    // The centralised converter re-expresses the seed parameters at an ad-hoc
    // perigee and evaluates the local field at their position itself.
    return ActsPlugins::EDM4hepUtil::writeTrackState(geoCtx, edm4hep::TrackState::AtFirstHit, paramseed,
                                                     *geo.magneticField(), magCache);
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
    auto                        seedTrack = seedCollection.create();
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
   */
  class CKFRunner {
  public:
    struct Config {
      double       chi2CutOff            = 15;
      std::int32_t numMeasurementsCutOff = 10;
      double       chi2CutOffOutlier     = std::numeric_limits<double>::max();
      bool         propagateBackward     = false;
      bool         extrapolateToCalo     = false;
      std::size_t  maxSteps              = kDefaultMaxPropagationSteps;

      /// Whether a track that crosses both calorimeter sections gets an
      /// AtCalorimeter state for each of them. False (the default) keeps one
      /// state per track, at the first calo face the track reaches. See
      /// CaloStateAppender::addEndcapStateAfterBarrel.
      bool addEndcapCaloState = false;

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
          m_extrapolator(std::make_unique<CKFPropagator>(makePropagator(geo, false))),
          m_caloAppender(
              geo, m_geoCtx, m_magCtx,
              {.enabled = cfg.extrapolateToCalo, .addEndcapState = cfg.addEndcapCaloState, .maxSteps = cfg.maxSteps}) {}

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

            auto track = ACTSTracking::ACTS2edm4hep_track(m_geoCtx, m_magCtx, trackTip, hits, m_geo.magneticField());

            m_caloAppender.addCaloState(alg, trackTip, track, magCache, caloMonitor);

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

    const IActsGeoSvc&                m_geo;
    Acts::GeometryContext             m_geoCtx;
    Acts::MagneticFieldContext        m_magCtx{};
    Acts::CalibrationContext          m_calCtx{};
    std::size_t                       m_maxSteps            = kDefaultMaxPropagationSteps;
    bool                              m_propagateBackward   = false;
    bool                              m_useBranchStopper    = false;
    int                               m_bsMaxHoles          = 2;
    int                               m_bsMaxOutliers       = 2;
    int                               m_bsMinMeasurements   = 6;
    double                            m_bsPtMin             = 0.0;
    int                               m_bsPtMinMeasurements = 3;
    Acts::MeasurementSelector::Config m_measSelConfig;

    std::unique_ptr<CombKalmanFilter>    m_trackFinder;
    std::shared_ptr<const Acts::Surface> m_referenceSurface;
    std::unique_ptr<CKFPropagator>       m_extrapolator;
    CaloStateAppender                    m_caloAppender;
  };

}  // namespace ACTSTracking
