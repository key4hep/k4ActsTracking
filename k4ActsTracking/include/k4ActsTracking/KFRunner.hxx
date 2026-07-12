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
#include "k4ActsTracking/Helpers.hxx"
#include "k4ActsTracking/IActsGeoSvc.h"
#include "k4ActsTracking/Measurement.hxx"
#include "k4ActsTracking/MeasurementCalibrator.hxx"
#include "k4ActsTracking/RunnerCommon.hxx"
#include "k4ActsTracking/SourceLink.hxx"

// edm4hep
#include <edm4hep/MutableTrack.h>

// ACTS
#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/EventData/SourceLink.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>
#include <Acts/EventData/VectorTrackContainer.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/TrackFitting/GainMatrixSmoother.hpp>
#include <Acts/TrackFitting/GainMatrixUpdater.hpp>
#include <Acts/TrackFitting/KalmanFitter.hpp>
#include <Acts/Utilities/CalibrationContext.hpp>

// Standard
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace ACTSTracking {

  /// ACTS Kalman fitter type built on the shared propagator.
  using KalmanFitter = Acts::KalmanFitter<CKFPropagator, Acts::VectorMultiTrajectory>;

  /// Surface accessor that resolves a SourceLink's surface via the tracking geometry.
  struct GeometrySurfaceAccessor {
    const Acts::TrackingGeometry* trackingGeometry = nullptr;
    const Acts::Surface*          operator()(const Acts::SourceLink& sourceLink) const {
      return trackingGeometry->findSurface(sourceLink.get<ACTSTracking::SourceLink>().geometryId());
    }
  };

  /**
   * @brief Owns the ACTS Kalman fitter and fits pre-grouped hit candidates.
   *
   * Whereas CKFRunner performs combinatorial track finding from seeds, KFRunner
   * fits a track candidate whose measurements are already known (e.g. hit groups
   * produced by an upstream pattern-recognition stage). For each candidate it
   * runs the ACTS KalmanFitter over the candidate's source links and converts
   * the smoothed result into an edm4hep::Track (via ACTS2edm4hep_track).
   *
   * Non-copyable and non-movable: the fitter extensions hold stable pointers to
   * the member calibrator/updater/smoother/surface-accessor. A single instance
   * may be shared by concurrent fit() calls (each uses its own track containers).
   */
  class KFRunner {
  public:
    struct Config {
      bool        propagateBackward = false;
      std::size_t maxSteps          = kDefaultMaxPropagationSteps;
    };

    /// @param measurements Event-local measurement container; must outlive the runner.
    /// @param hits Event-local hit container parallel to @p measurements; must outlive the runner.
    KFRunner(const IActsGeoSvc& geo, const Acts::GeometryContext& geoCtx, const Acts::MagneticFieldContext& magCtx,
             const Acts::CalibrationContext& calCtx, const ACTSTracking::MeasurementContainer& measurements,
             const ACTSTracking::HitContainer& hits, const Config& cfg)
        : m_geo(geo),
          m_geoCtx(geoCtx),
          m_trackingGeometry(geo.trackingGeometry()),
          m_perigee(Acts::Surface::makeShared<Acts::PerigeeSurface>(Acts::Vector3::Zero())),
          m_measCal(measurements),
          m_hits(hits) {
      m_fitter = std::make_unique<KalmanFitter>(makePropagator(geo, false));

      m_surfaceAccessor.trackingGeometry = m_trackingGeometry.get();
      m_extensions.surfaceAccessor.connect<&GeometrySurfaceAccessor::operator()>(&m_surfaceAccessor);
      m_extensions.calibrator.template connect<&ACTSTracking::MeasurementCalibrator::calibrate>(&m_measCal);
      m_extensions.updater.connect<&Acts::GainMatrixUpdater::operator()<Acts::VectorMultiTrajectory>>(&m_kfUpdater);
      m_extensions.smoother.connect<&Acts::GainMatrixSmoother::operator()<Acts::VectorMultiTrajectory>>(&m_kfSmoother);

      Acts::PropagatorPlainOptions pOptions{geoCtx, magCtx};
      pOptions.maxSteps = cfg.maxSteps;
      if (cfg.propagateBackward) {
        pOptions.direction = Acts::Direction::Backward();
      }

      m_kfOptions = std::make_unique<Acts::KalmanFitterOptions<Acts::VectorMultiTrajectory>>(
          geoCtx, magCtx, std::cref(calCtx), m_extensions, pOptions, m_perigee.get());
    }

    KFRunner(const KFRunner&)            = delete;
    KFRunner(KFRunner&&)                 = delete;
    KFRunner& operator=(const KFRunner&) = delete;
    KFRunner& operator=(KFRunner&&)      = delete;

    /// Fit a single candidate from its (uncalibrated) source links and initial parameters.
    /// @return The fitted edm4hep track, or std::nullopt if the fit failed.
    template <class Alg>
    std::optional<edm4hep::MutableTrack> fit(const Alg& alg, const std::vector<Acts::SourceLink>& sourceLinks,
                                             const Acts::BoundTrackParameters&   initialParameters,
                                             Acts::MagneticFieldProvider::Cache& magCache) const {
      auto              trackContainer      = std::make_shared<Acts::VectorTrackContainer>();
      auto              trackStateContainer = std::make_shared<Acts::VectorMultiTrajectory>();
      CKFTrackContainer tracks(trackContainer, trackStateContainer);

      auto result = m_fitter->fit(sourceLinks.begin(), sourceLinks.end(), initialParameters, *m_kfOptions, tracks);
      if (!result.ok()) {
        alg.warning() << "Kalman fit error: " << result.error() << endmsg;
        return std::nullopt;
      }

      return ACTSTracking::ACTS2edm4hep_track(m_geoCtx, result.value(), m_hits, m_geo.magneticField(), magCache);
    }

  private:
    const IActsGeoSvc&                            m_geo;
    Acts::GeometryContext                         m_geoCtx;
    std::shared_ptr<const Acts::TrackingGeometry> m_trackingGeometry;
    std::shared_ptr<Acts::PerigeeSurface>         m_perigee;

    Acts::GainMatrixUpdater             m_kfUpdater;
    Acts::GainMatrixSmoother            m_kfSmoother;
    ACTSTracking::MeasurementCalibrator m_measCal;
    const ACTSTracking::HitContainer&   m_hits;
    GeometrySurfaceAccessor             m_surfaceAccessor;

    Acts::KalmanFitterExtensions<Acts::VectorMultiTrajectory> m_extensions;

    std::unique_ptr<KalmanFitter>                                           m_fitter;
    std::unique_ptr<Acts::KalmanFitterOptions<Acts::VectorMultiTrajectory>> m_kfOptions;
  };

}  // namespace ACTSTracking
