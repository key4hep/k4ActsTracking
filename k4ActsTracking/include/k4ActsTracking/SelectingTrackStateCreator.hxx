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

#include "k4ActsTracking/Measurement.hxx"
#include "k4ActsTracking/MeasurementCalibrator.hxx"
#include "k4ActsTracking/SourceLink.hxx"

#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/EventData/MeasurementHelpers.hpp>
#include <Acts/EventData/SourceLink.hpp>
#include <Acts/EventData/SubspaceHelpers.hpp>
#include <Acts/EventData/TrackStatePropMask.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilterError.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilterExtensions.hpp>
#include <Acts/Utilities/CalibrationContext.hpp>
#include <Acts/Utilities/Logger.hpp>
#include <Acts/Utilities/Result.hpp>

#include <boost/container/small_vector.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <vector>

namespace ACTSTracking {

/**
 * @brief CKF track state creator that selects the measurements on a surface
 *        before creating track states for them.
 *
 * Equivalent to Acts::TrackStateCreator with an Acts::MeasurementSelector
 * configured with a single global cut bin (chi2 cut, outlier chi2 cut and
 * maximum number of measurements), and gives the same selection. The ACTS
 * creator allocates and calibrates a temporary track state for every
 * measurement on the surface before computing its chi2. Here the chi2 is
 * computed directly from the MeasurementContainer, which already holds the
 * calibrated local parameters and covariance, and track states are only
 * created for the selected measurements, as the ATLAS ActsTrk measurement
 * selector does.
 */
template <typename track_container_t>
class SelectingTrackStateCreator {
public:
  using TrackStateContainerBackend = typename track_container_t::TrackStateContainerBackend;
  using TrackStateProxy = typename track_container_t::TrackStateProxy;
  using BranchVector = Acts::CkfTypes::BranchVector<Acts::TrackIndexType>;

  struct Cuts {
    double chi2CutOff = 15;
    double chi2CutOffOutlier = std::numeric_limits<double>::max();
    std::size_t numMeasurementsCutOff = 1;
  };

  SelectingTrackStateCreator(const SourceLinkContainer& sourceLinks, const MeasurementContainer& measurements,
                             const MeasurementCalibrator& calibrator, const Cuts& cuts)
      : m_sourceLinks(sourceLinks), m_measurements(measurements), m_calibrator(calibrator), m_cuts(cuts) {}

  Acts::Result<BranchVector>
  createTrackStates(const Acts::GeometryContext& gctx, const Acts::CalibrationContext& calibrationContext,
                    const Acts::Surface& surface, const Acts::CkfTypes::BoundState& boundState,
                    Acts::TrackIndexType prevTip, std::vector<TrackStateProxy>& /*trackStateCandidates*/,
                    TrackStateContainerBackend& trajectory, const Acts::Logger& logger) const {
    using PM = Acts::TrackStatePropMask;

    auto [slBegin, slEnd] = m_sourceLinks.equal_range(surface.geometryId());
    if (slBegin == slEnd) {
      return Acts::Result<BranchVector>::success({});
    }
    if (m_cuts.numMeasurementsCutOff == 0) {
      return Acts::CombinatorialKalmanFilterError::MeasurementSelectionFailed;
    }

    const auto& [boundParams, jacobian, pathLength] = boundState;
    const Acts::BoundVector& predicted = boundParams.parameters();
    const Acts::BoundMatrix& predictedCovariance = *boundParams.covariance();

    // Same order, cuts and tie handling as Acts::MeasurementSelector::select:
    // the candidates passing the chi2 cut keep their source link order, the
    // outlier fallback is the first candidate with the smallest finite chi2.
    struct Candidate {
      const SourceLink* sourceLink;
      double chi2;
    };
    boost::container::small_vector<Candidate, 16> passed;
    const SourceLink* minSourceLink = nullptr;
    double minChi2 = std::numeric_limits<double>::max();
    for (auto it = slBegin; it != slEnd; ++it) {
      const double chi2 = calculateChi2(m_measurements[it->index()], predicted, predictedCovariance);
      if (!std::isfinite(chi2)) {
        continue;
      }
      if (chi2 < minChi2) {
        minChi2 = chi2;
        minSourceLink = &*it;
      }
      if (chi2 < m_cuts.chi2CutOff) {
        passed.push_back({&*it, chi2});
      }
    }

    bool isOutlier = false;
    if (passed.empty()) {
      if (minSourceLink == nullptr || !(minChi2 < m_cuts.chi2CutOffOutlier)) {
        ACTS_VERBOSE("No measurement candidate. Return empty chi2=" << minChi2);
        return Acts::Result<BranchVector>::success({});
      }
      ACTS_VERBOSE("No measurement candidate. Return an outlier measurement chi2=" << minChi2);
      isOutlier = true;
      passed.push_back({minSourceLink, minChi2});
    } else if (passed.size() > 1) {
      std::sort(passed.begin(), passed.end(), [](const Candidate& a, const Candidate& b) { return a.chi2 < b.chi2; });
      passed.resize(std::min(m_cuts.numMeasurementsCutOff, passed.size()));
    }

    BranchVector trackStateList;
    trackStateList.reserve(passed.size());
    std::optional<TrackStateProxy> firstTrackState;
    for (const Candidate& candidate : passed) {
      PM mask = PM::Predicted | PM::Jacobian | PM::Calibrated;
      if (firstTrackState) {
        mask &= ~PM::Predicted & ~PM::Jacobian;
      }
      auto trackState = trajectory.makeTrackState(mask, prevTip);
      if (firstTrackState) {
        trackState.shareFrom(*firstTrackState, PM::Predicted);
        trackState.shareFrom(*firstTrackState, PM::Jacobian);
      } else {
        trackState.predicted() = predicted;
        trackState.predictedCovariance() = predictedCovariance;
        trackState.jacobian() = jacobian;
        firstTrackState = trackState;
      }
      trackState.pathLength() = pathLength;
      trackState.setReferenceSurface(boundParams.referenceSurface().getSharedPtr());
      m_calibrator.calibrate(gctx, calibrationContext, Acts::SourceLink{*candidate.sourceLink}, trackState);
      trackState.chi2() = candidate.chi2;

      auto typeFlags = trackState.typeFlags();
      typeFlags.setHasParameters();
      typeFlags.setHasMeasurement();
      if (trackState.referenceSurface().hasMaterial()) {
        typeFlags.setHasMaterial();
      }
      if (isOutlier) {
        ACTS_VERBOSE("Creating outlier track state with tip = " << trackState.index());
        typeFlags.setIsOutlier();
      }
      trackStateList.push_back(trackState.index());
    }
    return trackStateList;
  }

private:
  /// chi2 of a measurement with respect to the predicted parameters, as
  /// Acts::MeasurementSelector::calculateChi2 computes it.
  static double calculateChi2(const Measurement& measurement, const Acts::BoundVector& predicted,
                              const Acts::BoundMatrix& predictedCovariance) {
    return Acts::visit_measurement(measurement.size(), [&](auto N) -> double {
      constexpr std::size_t kMeasurementSize = decltype(N)::value;
      const auto indices = measurement.subspaceIndices<kMeasurementSize>();
      Acts::FixedBoundSubspaceHelper<kMeasurementSize> subspaceHelper(
          std::span<const std::uint8_t, kMeasurementSize>(indices.begin(), kMeasurementSize));
      const Acts::Vector<kMeasurementSize> res =
          measurement.parameters<kMeasurementSize>() - subspaceHelper.projectVector(predicted);
      return (res.transpose() *
              (measurement.covariance<kMeasurementSize>() + subspaceHelper.projectMatrix(predictedCovariance))
                  .inverse() *
              res)
          .eval()(0, 0);
    });
  }

  const SourceLinkContainer& m_sourceLinks;
  const MeasurementContainer& m_measurements;
  const MeasurementCalibrator& m_calibrator;
  Cuts m_cuts;
};

} // namespace ACTSTracking
