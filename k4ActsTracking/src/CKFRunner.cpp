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
#include "k4ActsTracking/MeasurementCalibrator.hxx"

#include <Gaudi/Algorithm.h>

#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>
#include <Acts/TrackFinding/MeasurementSelector.hpp>
#include <Acts/TrackFinding/TrackStateCreator.hpp>
#include <Acts/TrackFitting/GainMatrixUpdater.hpp>
#include <Acts/Utilities/TrackHelpers.hpp>

namespace ACTSTracking {

using CKFTrackFinderOptions = Acts::CombinatorialKalmanFilterOptions<CKFTrackContainer>;
using CombKalmanFilter = Acts::CombinatorialKalmanFilter<CKFPropagator, CKFTrackContainer>;

// Keep the ACTS implementation and its template instantiations in one translation unit.
struct CKFRunner::Impl {
  using TrackStateCreatorType = Acts::TrackStateCreator<SourceLinkAccessor::Iterator, CKFTrackContainer>;

  Impl(const IActsGeoSvc& geo, const Config& cfg)
      : m_geo(geo), m_geoCtx(Acts::GeometryContext::dangerouslyDefaultConstruct()), m_maxSteps(cfg.maxSteps),
        m_propagateBackward(cfg.propagateBackward), m_useBranchStopper(cfg.useBranchStopper),
        m_bsMaxHoles(cfg.bsMaxHoles), m_bsMaxOutliers(cfg.bsMaxOutliers), m_bsMinMeasurements(cfg.bsMinMeasurements),
        m_bsPtMin(cfg.bsPtMin), m_bsPtMinMeasurements(cfg.bsPtMinMeasurements),
        m_measSelConfig(makeSelectorConfig(cfg)),
        m_trackFinder(std::make_unique<CombKalmanFilter>(makePropagator(geo, false))),
        m_referenceSurface(cfg.referenceSurface
                               ? cfg.referenceSurface
                               : Acts::Surface::makeShared<Acts::PerigeeSurface>(Acts::Vector3::Zero())),
        m_extrapolator(std::make_unique<CKFPropagator>(makePropagator(geo, false))),
        m_caloAppender(
            geo, m_geoCtx, m_magCtx,
            {.enabled = cfg.extrapolateToCalo, .addEndcapState = cfg.addEndcapCaloState, .maxSteps = cfg.maxSteps}) {}

  void findTracks(const Gaudi::Algorithm& alg, const MeasurementContainer& measurements,
                  const SourceLinkContainer& sourceLinks, const HitContainer& hits,
                  const std::vector<Acts::BoundTrackParameters>& paramseeds,
                  Acts::MagneticFieldProvider::Cache& magCache, edm4hep::TrackCollection& trackCollection,
                  std::mutex& trackMutex, const CaloExtrapMonitor* caloMonitor) const {
    alg.debug() << "Starting CKF track finding with " << paramseeds.size() << " seeds." << endmsg;

    // Bind event-local data and extensions so concurrent calls stay independent.
    Acts::GainMatrixUpdater kfUpdater;
    Acts::MeasurementSelector measSel{m_measSelConfig};
    MeasurementCalibrator measCal{measurements};
    SourceLinkAccessor slAccessor;
    slAccessor.container = &sourceLinks;

    TrackStateCreatorType trackStateCreator;
    trackStateCreator.sourceLinkAccessor.connect<&SourceLinkAccessor::range>(&slAccessor);
    trackStateCreator.calibrator.connect<&MeasurementCalibrator::calibrate>(&measCal);
    trackStateCreator.measurementSelector.connect<&Acts::MeasurementSelector::select<Acts::VectorMultiTrajectory>>(
        &measSel);

    Acts::CombinatorialKalmanFilterExtensions<CKFTrackContainer> extensions;
    extensions.updater.connect<&Acts::GainMatrixUpdater::operator()<Acts::VectorMultiTrajectory>>(&kfUpdater);
    extensions.createTrackStates.connect<&TrackStateCreatorType::createTrackStates>(&trackStateCreator);
    if (m_useBranchStopper) {
      extensions.branchStopper.connect<&Impl::branchStopper>(this);
    }

    Acts::PropagatorPlainOptions pOptions{m_geoCtx, m_magCtx};
    pOptions.maxSteps = m_maxSteps;
    if (m_propagateBackward) {
      pOptions.direction = Acts::Direction::Backward();
    }
    const CKFTrackFinderOptions ckfOptions(m_geoCtx, m_magCtx, m_calCtx, extensions, pOptions);

    auto trackContainer = std::make_shared<Acts::VectorTrackContainer>();
    auto trackStateContainer = std::make_shared<Acts::VectorMultiTrajectory>();
    CKFTrackContainer tracks(trackContainer, trackStateContainer);

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

          CKFPropagator::Options<> exOptions(m_geoCtx, m_magCtx);
          exOptions.maxSteps = m_maxSteps;
          const CKFPropagator& extrapolator = *m_extrapolator;
          auto exResult = Acts::extrapolateTrackToReferenceSurface(
              trackTip, *m_referenceSurface, extrapolator, exOptions, Acts::TrackExtrapolationStrategy::firstOrLast);
          if (!exResult.ok()) {
            alg.warning() << "Reference-surface extrapolation error: " << exResult.error() << endmsg;
            continue;
          }

          auto track = ACTS2edm4hep_track(m_geoCtx, m_magCtx, trackTip, hits, m_geo.magneticField());
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

  static Acts::MeasurementSelector::Config makeSelectorConfig(const Config& cfg) {
    return {{Acts::GeometryIdentifier(),
             {{}, {cfg.chi2CutOff}, {static_cast<std::size_t>(cfg.numMeasurementsCutOff)}, {cfg.chi2CutOffOutlier}}}};
  }

  Acts::CombinatorialKalmanFilterBranchStopperResult
  branchStopper(const CKFTrackContainer::TrackProxy& track,
                const CKFTrackContainer::TrackStateProxy& trackState) const {
    using Result = Acts::CombinatorialKalmanFilterBranchStopperResult;
    const int nMeas = static_cast<int>(track.nMeasurements());
    if (m_bsPtMin > 0.0 && nMeas >= m_bsPtMinMeasurements) {
      const auto& params = trackState.hasFiltered() ? trackState.filtered() : trackState.predicted();
      const double theta = params[Acts::eBoundTheta];
      const double qOverP = params[Acts::eBoundQOverP];
      if (qOverP != 0.0) {
        const double pt = std::abs(std::sin(theta) / qOverP);
        if (pt < m_bsPtMin * Acts::UnitConstants::GeV) {
          return Result::StopAndDrop;
        }
      }
    }

    const bool tooManyHoles = static_cast<int>(track.nHoles()) > m_bsMaxHoles;
    const bool tooManyOutliers = static_cast<int>(track.nOutliers()) > m_bsMaxOutliers;
    if (!(tooManyHoles || tooManyOutliers)) {
      return Result::Continue;
    }
    return (nMeas >= m_bsMinMeasurements) ? Result::StopAndKeep : Result::StopAndDrop;
  }

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
  Acts::MeasurementSelector::Config m_measSelConfig;

  std::unique_ptr<CombKalmanFilter> m_trackFinder;
  std::shared_ptr<const Acts::Surface> m_referenceSurface;
  std::unique_ptr<CKFPropagator> m_extrapolator;
  CaloStateAppender m_caloAppender;
};

CKFRunner::CKFRunner(const IActsGeoSvc& geo, const Config& cfg) : m_impl(std::make_unique<Impl>(geo, cfg)) {}

CKFRunner::~CKFRunner() = default;

void CKFRunner::findTracks(const Gaudi::Algorithm& alg, const MeasurementContainer& measurements,
                           const SourceLinkContainer& sourceLinks, const HitContainer& hits,
                           const std::vector<Acts::BoundTrackParameters>& paramseeds,
                           Acts::MagneticFieldProvider::Cache& magCache, edm4hep::TrackCollection& trackCollection,
                           std::mutex& trackMutex, const CaloExtrapMonitor* caloMonitor) const {
  m_impl->findTracks(alg, measurements, sourceLinks, hits, paramseeds, magCache, trackCollection, trackMutex,
                     caloMonitor);
}

} // namespace ACTSTracking
