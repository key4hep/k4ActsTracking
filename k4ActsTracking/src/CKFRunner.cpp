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

#include "k4ActsTracking/CKFRunner.hxx"

// k4ActsTracking
#include "k4ActsTracking/MeasurementCalibrator.hxx"

// edm4hep
#include <edm4hep/MutableTrack.h>

// Gaudi
#include <GaudiKernel/MsgStream.h>

// ACTS
#include <Acts/Definitions/Units.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>
#include <Acts/EventData/VectorTrackContainer.hpp>
#include <Acts/Propagator/ActorList.hpp>
#include <Acts/Propagator/DefaultActor.hpp>
#include <Acts/Propagator/MaterialInteractor.hpp>
#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/TrackFinding/TrackStateCreator.hpp>
#include <Acts/TrackFitting/GainMatrixUpdater.hpp>
#include <Acts/Utilities/TrackHelpers.hpp>

#include <fmt/format.h>

// Standard
#include <cmath>
#include <string>
#include <utility>

namespace ACTSTracking {

namespace {

  using CKFTrackFinderOptions = Acts::CombinatorialKalmanFilterOptions<CKFTrackContainer>;
  using TrackStateCreatorType = Acts::TrackStateCreator<ACTSTracking::SourceLinkAccessor::Iterator, CKFTrackContainer>;

  /**
   * @brief Anchor of the two-way CKF: the first (stem) state of @p track.
   *
   * Walks the track forward (stem to tip), so the anchor is where the first CKF
   * pass started: the innermost state for an inside-out first pass, the
   * outermost for an outside-in one. The second pass runs from there in the
   * opposite direction. The track must be forward-linked (copyFrom() does that).
   *
   * @return The anchor state, or std::nullopt if the track has no states.
   */
  template <class track_proxy_t>
  std::optional<typename track_proxy_t::ConstTrackStateProxy> findTwoWayAnchor(const track_proxy_t& track) {
    auto states = track.trackStates();
    if (states.begin() == states.end()) {
      return std::nullopt;
    }
    return typename track_proxy_t::ConstTrackStateProxy{*states.begin()};
  }

} // namespace

CKFRunner::CKFRunner(const IActsGeoSvc& geo, const Config& cfg)
    : m_geo(geo), m_geoCtx(Acts::GeometryContext::dangerouslyDefaultConstruct()), m_maxSteps(cfg.maxSteps),
      m_propagateBackward(cfg.propagateBackward), m_useBranchStopper(cfg.useBranchStopper),
      m_bsMaxHoles(cfg.bsMaxHoles), m_bsMaxOutliers(cfg.bsMaxOutliers), m_bsMinMeasurements(cfg.bsMinMeasurements),
      m_bsPtMin(cfg.bsPtMin), m_bsPtMinMeasurements(cfg.bsPtMinMeasurements), m_doTwoWayCKF(cfg.doTwoWayCKF),
      m_inflateCovarianceTwoWay(cfg.inflateCovarianceTwoWay),
      m_twoWayInflateCovarianceFactor(cfg.twoWayInflateCovarianceFactor), m_measSelConfig(makeSelectorConfig(cfg)),
      m_trackFinder(std::make_unique<CombKalmanFilter>(makePropagator(geo, false))),
      m_referenceSurface(cfg.referenceSurface ? cfg.referenceSurface
                                              : Acts::Surface::makeShared<Acts::PerigeeSurface>(Acts::Vector3::Zero())),
      m_extrapolator(std::make_unique<CKFPropagator>(makePropagator(geo, false))),
      m_caloAppender(
          geo, m_geoCtx, m_magCtx,
          {.enabled = cfg.extrapolateToCalo, .addEndcapState = cfg.addEndcapCaloState, .maxSteps = cfg.maxSteps}) {
  if (cfg.trackSelector) {
    m_trackSelector.emplace(*cfg.trackSelector);
  }
}

CKFRunner::~CKFRunner() = default;

std::string CKFRunner::twoWaySummary() const {
  return fmt::format("Two-way CKF: {} tracks extended by the second pass, {} kept single-pass (no anchor, second "
                     "pass failed or empty, no new measurement, or smoothing error).",
                     m_nTwoWayStitched.load(), m_nTwoWayFallbacks.load());
}

void CKFRunner::findTracks(const CommonMessagingBase& msg, const ACTSTracking::MeasurementContainer& measurements,
                           const ACTSTracking::SourceLinkContainer& sourceLinks, const ACTSTracking::HitContainer& hits,
                           const std::vector<Acts::BoundTrackParameters>& paramseeds,
                           Acts::MagneticFieldProvider::Cache& magCache, edm4hep::TrackCollection& trackCollection,
                           std::mutex& trackMutex, const CaloExtrapMonitor* caloMonitor) const {
  msg.debug() << "Starting CKF track finding with " << paramseeds.size() << " seeds." << endmsg;

  // Bind the event-local measurements / source links. These, and the
  // extensions/options that point at them, are per-call locals so concurrent
  // findTracks() calls stay independent; the propagators, CKF and selector
  // config are shared read-only state.
  Acts::GainMatrixUpdater kfUpdater;
  Acts::MeasurementSelector measSel{m_measSelConfig};
  ACTSTracking::MeasurementCalibrator measCal{measurements};
  ACTSTracking::SourceLinkAccessor slAccessor;
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
  // Backward (outside-in): from the outer seed SP inward.
  if (m_propagateBackward) {
    pOptions.direction = Acts::Direction::Backward();
  }
  CKFTrackFinderOptions ckfOptions(m_geoCtx, m_magCtx, m_calCtx, extensions, pOptions);
  // Stop the inward pass at the reference surface, so it does not run on
  // into the opposite side of the vertex detector.
  ckfOptions.targetSurface = m_propagateBackward ? m_referenceSurface.get() : nullptr;

  // Two-way CKF: second pass propagates opposite to the first pass.
  //   Inside-out first pass  -> backward second pass (toward perigee).
  //   Outside-in first pass  -> forward second pass  (into outer tracker).
  // skipPrePropagationUpdate: the second pass re-seeds from the smoothed
  // anchor of the first pass, so the CKF must not re-apply the pre-
  // propagation Kalman update.
  Acts::PropagatorPlainOptions secondPOptions{m_geoCtx, m_magCtx};
  secondPOptions.maxSteps = m_maxSteps;
  secondPOptions.direction = m_propagateBackward ? Acts::Direction::Forward() : Acts::Direction::Backward();
  CKFTrackFinderOptions secondOptions(m_geoCtx, m_magCtx, m_calCtx, extensions, secondPOptions);
  secondOptions.targetSurface = m_propagateBackward ? nullptr : m_referenceSurface.get();
  secondOptions.skipPrePropagationUpdate = true;

  auto trackContainer = std::make_shared<Acts::VectorTrackContainer>();
  auto trackStateContainer = std::make_shared<Acts::VectorMultiTrajectory>();
  CKFTrackContainer tracks(trackContainer, trackStateContainer);

  // Access the track finder as const so concurrent findTracks() calls use
  // the read-only overload (the finder and propagators are shared state).
  const CombKalmanFilter& trackFinder = *m_trackFinder;

  for (std::size_t iseed = 0; iseed < paramseeds.size(); ++iseed) {
    tracks.clear();
    auto result = trackFinder.findTracks(paramseeds.at(iseed), ckfOptions, tracks);
    if (!result.ok()) {
      msg.warning() << "Track fit error: " << result.error() << endmsg;
      continue;
    }

    for (const CKFTrackContainer::TrackProxy& trackItem : result.value()) {
      auto smoothed = tracks.makeTrack();
      smoothed.copyFrom(trackItem);
      auto smoothResult = Acts::smoothTrack(m_geoCtx, smoothed);
      if (!smoothResult.ok()) {
        msg.warning() << "Track smoothing error: " << smoothResult.error() << endmsg;
        continue;
      }

      // ---------------------------------------------------------------
      // Optional two-way CKF: run a second pass in the opposite
      // direction from the smoothed innermost/outermost state, then
      // splice its states onto the first-pass chain so a single
      // .previous() walk covers both passes. The stitched track goes
      // through the same reference-surface extrapolation as the
      // single-pass path.
      //   Inside-out (forward first pass): anchor = innermost state;
      //     second (backward) pass extends the chain toward perigee.
      //   Outside-in (backward first pass): anchor = outermost state;
      //     second (forward) pass extends the chain into the outer tracker,
      //     and the stitched track is smoothed.
      // ---------------------------------------------------------------
      std::optional<CKFTrackContainer::TrackProxy> stitchedOpt;
      if (m_doTwoWayCKF) {
        using ConstTP = CKFTrackContainer::ConstTrackProxy;

        ConstTP constSmoothed(smoothed);
        const auto anchor = findTwoWayAnchor(constSmoothed);

        if (!anchor.has_value()) {
          msg.debug() << "TwoWayCKF: no anchor state found, falling back to single-pass output." << endmsg;
        } else {
          const auto anchorIdx = anchor->index();

          Acts::BoundTrackParameters params2 = constSmoothed.createParametersFromState(*anchor);
          if (m_inflateCovarianceTwoWay) {
            auto cov2 = *params2.covariance();
            cov2 *= m_twoWayInflateCovarianceFactor;
            params2 = Acts::BoundTrackParameters(params2.referenceSurface().getSharedPtr(), params2.parameters(), cov2,
                                                 params2.particleHypothesis());
          }

          // Start the second pass from a root branch carrying the first-pass
          // summary (nMeasurements/nHoles/nOutliers/chi2/nDoF) but no states:
          // the CKF then increments these counters, so the stitched track
          // ends up with the totals for both passes, and the branch stopper
          // sees the full track rather than just the second-pass part
          auto secondRoot = tracks.makeTrack();
          secondRoot.copyFromWithoutStates(smoothed);
          auto secondResult = trackFinder.findTracks(params2, secondOptions, tracks, secondRoot);
          // The second pass can also return a track with no states at all
          // (first seen once endcap-disk seeding was enabled). Copying it
          // leaves secondTrack without a stem index, so trackStates() below
          // throws "Track has no stem index" and the whole event aborts;
          // now treating it like an empty result instead. Additionally, if
          // the CKF returns  multiple branches, choose the best one
          // explicitly: most measurements, then lowest chi2.
          const CKFTrackContainer::TrackProxy* bestSecondTrack = nullptr;
          if (secondResult.ok()) {
            const auto& secondCandidates = secondResult.value();
            for (std::size_t candidateIndex = 0; candidateIndex < secondCandidates.size(); ++candidateIndex) {
              const auto& candidate = secondCandidates[candidateIndex];
              if (candidate.nTrackStates() == 0) {
                continue;
              }
              if (bestSecondTrack == nullptr || candidate.nMeasurements() > bestSecondTrack->nMeasurements() ||
                  (candidate.nMeasurements() == bestSecondTrack->nMeasurements() &&
                   candidate.chi2() < bestSecondTrack->chi2())) {
                bestSecondTrack = &candidate;
              }
            }
          }
          const bool secondEmpty = !secondResult.ok() || bestSecondTrack == nullptr;
          if (secondEmpty) {
            msg.debug() << "TwoWayCKF: second pass "
                        << (!secondResult.ok()             ? std::string("FAILED: ") + secondResult.error().message()
                            : secondResult.value().empty() ? std::string("returned EMPTY")
                                                           : std::string("returned no tracks with states"))
                        << ", falling back to single-pass output." << endmsg;
          } else if (bestSecondTrack->nMeasurements() <= smoothed.nMeasurements()) {
            msg.debug() << "TwoWayCKF: second pass found no new measurement, falling back to single-pass output."
                        << endmsg;
          } else {
            auto secondTrack = tracks.makeTrack();
            secondTrack.copyFrom(*bestSecondTrack);

            // Same stitching for both directions. The CKF creates no state
            // on the anchor surface, so after reverseTrackStates() tipIndex()
            // is the second-pass state next to the anchor: link the anchor's
            // .previous() to it and keep the first-pass tip. A .previous()
            // walk from the tip then runs through the first pass, the anchor
            // and the second pass: outermost -> innermost for inside-out,
            // innermost -> outermost for outside-in.
            auto junctionJacobian = secondTrack.reverseTrackStates(true);
            auto anchorState = tracks.trackStateContainer().getTrackState(anchorIdx);
            const auto anchorPrevious = anchorState.previous();
            const Acts::BoundMatrix anchorJacobian = anchorState.jacobian();
            anchorState.previous() = secondTrack.tipIndex();
            if (junctionJacobian) {
              anchorState.jacobian() = *junctionJacobian;
            }
            secondTrack.tipIndex() = smoothed.tipIndex();
            secondTrack.linkForward();

            // Outside-in: the forward second pass found the outer-tracker
            // hits, where the calo extrapolation starts, so smooth the
            // stitched track to give them smoothed parameters, as the ACTS
            // examples do for reverse search. Inside-out needs no smoothing:
            // its second-pass states are the inner ones, and the perigee
            // target already sets the IP parameters.
            const Acts::Result<void> stitchedSmoothResult =
                m_propagateBackward ? Acts::smoothTrack(m_geoCtx, secondTrack) : Acts::Result<void>::success();
            if (!stitchedSmoothResult.ok()) {
              msg.warning() << "TwoWayCKF: stitched-track smoothing error: " << stitchedSmoothResult.error()
                            << ", falling back to single-pass output." << endmsg;
              anchorState.previous() = anchorPrevious;
              anchorState.jacobian() = anchorJacobian;
            } else {
              stitchedOpt = secondTrack;
            }
          }
        }
        // Counted here rather than reported as error per track; see twoWaySummary().
        ++(stitchedOpt ? m_nTwoWayStitched : m_nTwoWayFallbacks);
      }

      // Pick the track to emit: stitched (if two-way succeeded), else
      // smoothed. Both go through the same reference-surface extrapolation
      // + calo-face append + edm4hep conversion below, so the caller sees
      // the same output shape regardless of two-way mode.
      CKFTrackContainer::TrackProxy trackTip = stitchedOpt.value_or(smoothed);

      // A backward first pass leaves the innermost state at the tip, with or
      // without a stitched second pass. Reverse the states so the track is
      // ordered inside-out like a forward one. Only after the two-way step,
      // which needs the first pass
      if (m_propagateBackward) {
        trackTip.reverseTrackStates(true);
      }

      // Extrapolate the (stitched or smoothed) track to the reference
      // surface (the IP perigee by default, or a beam-perpendicular plane
      // for telescope clients) so its track-level parameters (and hence
      // the AtIP edm4hep TrackState) are defined there, not at the
      // innermost measurement surface. Must happen before
      // ACTS2edm4hep_track, which fills the AtIP state. Include the material
      // crossed on the way (beam pipe) in the covariance.
      using ExtrapolationOptions =
          CKFPropagator::Options<Acts::ActorList<Acts::DefaultActor, Acts::MaterialInteractor>>;
      ExtrapolationOptions exOptions(m_geoCtx, m_magCtx);
      exOptions.maxSteps = m_maxSteps;
      const CKFPropagator& extrapolator = *m_extrapolator;
      auto exResult = Acts::extrapolateTrackToReferenceSurface(trackTip, *m_referenceSurface, extrapolator, exOptions,
                                                               Acts::TrackExtrapolationStrategy::firstOrLast);
      if (!exResult.ok()) {
        msg.warning() << "Reference-surface extrapolation error: " << exResult.error() << endmsg;
        continue;
      }
      if (m_trackSelector && !m_trackSelector->isValidTrack(trackTip)) {
        continue;
      }

      auto track = ACTSTracking::ACTS2edm4hep_track(m_geoCtx, m_magCtx, trackTip, hits, m_geo.magneticField());

      // Calo-face state is only appended to the final (stitched or
      // single-pass) output track, so it runs at most once per emitted
      // track regardless of two-way mode.
      m_caloAppender.addCaloState(msg, trackTip, track, magCache, caloMonitor);

      {
        std::lock_guard lock{trackMutex};
        trackCollection.push_back(track);
      }
    }
  }
}

Acts::MeasurementSelector::Config CKFRunner::makeSelectorConfig(const Config& cfg) {
  return {{Acts::GeometryIdentifier(),
           {{}, {cfg.chi2CutOff}, {static_cast<std::size_t>(cfg.numMeasurementsCutOff)}, {cfg.chi2CutOffOutlier}}}};
}

Acts::CombinatorialKalmanFilterBranchStopperResult
CKFRunner::branchStopper(const CKFTrackContainer::TrackProxy& track,
                         const CKFTrackContainer::TrackStateProxy& trackState) const {
  using Result = Acts::CombinatorialKalmanFilterBranchStopperResult;

  const int nMeas = static_cast<int>(track.nMeasurements());

  // pT branch stop: once the branch has enough measurements for the momentum
  // estimate to be reliable, drop it if |pT| has fallen below threshold.
  if (m_bsPtMin > 0.0 && nMeas >= m_bsPtMinMeasurements) {
    const auto& params = trackState.hasFiltered() ? trackState.filtered() : trackState.predicted();
    const double theta = params[Acts::eBoundTheta];
    const double qOverP = params[Acts::eBoundQOverP];
    if (qOverP != 0.0) {
      const double pt = std::abs(std::sin(theta) / qOverP); // native momentum units
      if (pt < m_bsPtMin * Acts::UnitConstants::GeV) {
        return Result::StopAndDrop;
      }
    }
  }

  // Hole / outlier branch stop.
  const bool tooManyHoles = static_cast<int>(track.nHoles()) > m_bsMaxHoles;
  const bool tooManyOutliers = static_cast<int>(track.nOutliers()) > m_bsMaxOutliers;
  if (!(tooManyHoles || tooManyOutliers)) {
    return Result::Continue;
  }
  // Keep the branch if it already has enough measurements, otherwise drop it.
  return (nMeas >= m_bsMinMeasurements) ? Result::StopAndKeep : Result::StopAndDrop;
}

} // namespace ACTSTracking
