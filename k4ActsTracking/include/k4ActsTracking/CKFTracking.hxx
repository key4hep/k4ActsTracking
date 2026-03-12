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
#include <Acts/EventData/TrackParameters.hpp>

// ACTSTracking
#include "k4ActsTracking/GeometryIdSelector.hxx"
#include "k4ActsTracking/Measurement.hxx"
#include "k4ActsTracking/SeedSpacePoint.hxx"
#include "k4ActsTracking/SourceLink.hxx"

// Standard
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace ACTSTracking {

  //! Parse seeding layer strings into a geometry ID selector
  /**
   * Parses a flat list of alternating volume/layer string tokens into an
   * \c Acts::GeometryIdentifier selection. Use "*" as a wildcard for either
   * the volume or the layer component.
   *
   * \param seedingLayers Flat vector of (volume, layer) string pairs.
   * \return Geometry ID selector for seed-layer filtering.
   * \throws std::runtime_error if the vector has an odd number of entries.
   */
  inline GeometryIdSelector parseSeedingLayers(const std::vector<std::string>& seedingLayers) {
    std::vector<std::string> layers;
    std::copy_if(seedingLayers.begin(), seedingLayers.end(), std::back_inserter(layers),
                 [](const std::string& s) { return !s.empty(); });

    if (layers.size() % 2 != 0) {
      throw std::runtime_error("SeedingLayers needs an even number of entries");
    }

    std::vector<Acts::GeometryIdentifier> geoSelection;
    for (std::size_t i = 0; i < layers.size(); i += 2) {
      Acts::GeometryIdentifier geoid;
      if (layers[i] != "*")        // volume
        geoid = geoid.withVolume(std::stoi(layers[i]));
      if (layers[i + 1] != "*")    // layer
        geoid = geoid.withLayer(std::stoi(layers[i + 1]));
      geoSelection.push_back(geoid);
    }

    return GeometryIdSelector(geoSelection);
  }

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
