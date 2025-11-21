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

#include "Acts/EventData/SourceLink.hpp"
#include "Acts/EventData/VectorMultiTrajectory.hpp"
#include "Acts/Geometry/GeometryContext.hpp"
#include "Acts/Utilities/CalibrationContext.hpp"

#include "k4ActsTracking/Measurement.hxx"
#include "k4ActsTracking/SourceLink.hxx"

namespace ACTSTracking {

  class MeasurementCalibrator {
  public:
    /// Construct using a user-provided container to chose measurements from.
    MeasurementCalibrator(const MeasurementContainer& measurements) : m_measurements(measurements) {}

    //! Find the measurement corresponding to the source link.
    /**
   * @tparam parameters_t Track parameters type
   * @param sourceLink Input source link
   * @param parameters Input track parameters (unused)
   */
    template <typename parameters_t>
    const Measurement& operator()(const SourceLink& sourceLink, const parameters_t& /* parameters */) const {
      assert((sourceLink.index() < m_measurements.size()) and "Source link index is outside the container bounds");
      return m_measurements[sourceLink.index()];
    }

    void calibrate(const Acts::GeometryContext&, const Acts::CalibrationContext&, const Acts::SourceLink& sourceLink,
                   Acts::VectorMultiTrajectory::TrackStateProxy trackState) const {
      trackState.setUncalibratedSourceLink(Acts::SourceLink{sourceLink});
      const SourceLink& idxSourceLink = sourceLink.get<SourceLink>();

      assert((idxSourceLink.index() < m_measurements.size()) and "Source link index is outside the container bounds");

      const Measurement& measurement = m_measurements[idxSourceLink.index()];

      Acts::visit_measurement(measurement.size(), [&](auto N) -> void {
        constexpr std::size_t kMeasurementSize = decltype(N)::value;

        trackState.allocateCalibrated(kMeasurementSize);
        trackState.calibrated<kMeasurementSize>()           = measurement.parameters<kMeasurementSize>();
        trackState.calibratedCovariance<kMeasurementSize>() = measurement.covariance<kMeasurementSize>();
        trackState.setProjectorSubspaceIndices(measurement.subspaceIndices<kMeasurementSize>());
      });
    }

  private:
    const MeasurementContainer& m_measurements;
  };

}  // namespace ACTSTracking
