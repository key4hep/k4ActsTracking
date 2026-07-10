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

    return ACTSTracking::ACTS2edm4hep_trackState(edm4hep::TrackState::AtFirstHit, paramseed,
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
    };

    /// Builds the event-independent propagators and CKF once. The event-local
    /// measurements and source links are bound per call in findTracks().
    ///
    /// The CKF propagator, the IP-perigee extrapolator and (optionally) the
    /// calorimeter-face propagator depend only on the tracking geometry and
    /// field, so they are constructed here and reused read-only across events
    /// and threads. Without the separate IP extrapolator the CKF tracks would
    /// keep their parameters at a measurement surface and the AtIP track state
    /// would come out degenerate (omega=nan, D0=Z0=0).
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
          m_perigee(Acts::Surface::makeShared<Acts::PerigeeSurface>(Acts::Vector3::Zero())),
          m_extrapolator(std::make_unique<CKFPropagator>(makePropagator(geo, false))) {
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

            // Extrapolate the smoothed track to the perigee at the IP so its
            // track-level parameters (and hence the AtIP edm4hep TrackState) are
            // defined there, not at the innermost measurement surface. Must
            // happen before ACTS2edm4hep_track, which fills the AtIP state.
            typename CKFPropagator::template Options<> exOptions(m_geoCtx, m_magCtx);
            exOptions.maxSteps                = m_maxSteps;
            const CKFPropagator& extrapolator = *m_extrapolator;
            auto exResult = Acts::extrapolateTrackToReferenceSurface(trackTip, *m_perigee, extrapolator, exOptions,
                                                                     Acts::TrackExtrapolationStrategy::firstOrLast);
            if (!exResult.ok()) {
              alg.warning() << "IP extrapolation error: " << exResult.error() << endmsg;
              continue;
            }

            auto track = ACTSTracking::ACTS2edm4hep_track(trackTip, hits, m_geo.magneticField(), magCache);

            addCaloState(alg, trackTip, track, magCache, caloMonitor);

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
          auto                caloState =
              ACTSTracking::ACTS2edm4hep_trackState(edm4hep::TrackState::AtCalorimeter, *caloResult.params, Bz);
          // The calo-face parameters are local to the target surface, so the
          // edm4hep D0/Z0 from the generic conversion are not meaningful here.
          // Express the state at the impact point instead: set the reference
          // point to the global calo-face position (D0 = Z0 = 0 there).
          caloState.referencePoint = edm4hep::Vector3f(caloPos.x(), caloPos.y(), caloPos.z());
          caloState.D0             = 0;
          caloState.Z0             = 0;
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
    std::shared_ptr<Acts::PerigeeSurface>             m_perigee;
    std::unique_ptr<CKFPropagator>                    m_extrapolator;
    std::unique_ptr<ACTSTracking::CaloFacePropagator> m_caloPropagator;
  };

}  // namespace ACTSTracking
