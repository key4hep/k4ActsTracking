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

// ACTS
#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/EventData/FreeTrackParameters.hpp>
#include <Acts/EventData/ParticleHypothesis.hpp>

// Standard
#include <cmath>

namespace ACTSTracking {

  //! Build a diagonal initial track covariance matrix from per-parameter error estimates
  /**
   * \param p        Absolute momentum [ACTS units].
   * \param errPos   Local-position error [ACTS units].
   * \param errPhi   Azimuthal angle error [rad].
   * \param errLambda Polar angle (lambda) error [rad].
   * \param errRelP  Relative momentum error (dimensionless fraction).
   * \param errTime  Time error [ACTS units].
   * \return         5×5 diagonal bound covariance matrix.
   */
  inline Acts::BoundMatrix makeInitialCovariance(double p, double errPos, double errPhi, double errLambda,
                                                 double errRelP, double errTime) {
    Acts::BoundMatrix cov                       = Acts::BoundMatrix::Zero();
    cov(Acts::eBoundLoc0, Acts::eBoundLoc0)     = std::pow(errPos, 2);
    cov(Acts::eBoundLoc1, Acts::eBoundLoc1)     = std::pow(errPos, 2);
    cov(Acts::eBoundTime, Acts::eBoundTime)     = std::pow(errTime, 2);
    cov(Acts::eBoundPhi, Acts::eBoundPhi)       = std::pow(errPhi, 2);
    cov(Acts::eBoundTheta, Acts::eBoundTheta)   = std::pow(errLambda, 2);
    cov(Acts::eBoundQOverP, Acts::eBoundQOverP) = std::pow(errRelP * p / (p * p), 2);
    return cov;
  }

}  // namespace ACTSTracking
