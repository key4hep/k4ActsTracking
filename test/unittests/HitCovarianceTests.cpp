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
#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"

#include <k4ActsTracking/RunnerCommon.hxx>

#include <Acts/Definitions/Algebra.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Surfaces/PlaneSurface.hpp>
#include <Acts/Surfaces/RectangleBounds.hpp>

#include <edm4hep/TrackerHitPlaneCollection.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>

using Catch::Matchers::WithinAbs;

namespace {
constexpr double kDu = 0.05;
constexpr double kDv = 1.0;
constexpr double kHalfPi = std::numbers::pi / 2;
// edm4hep stores u/v and du/dv as float
constexpr double kTol = 1e-6;

std::shared_ptr<Acts::PlaneSurface> makePlane(double rotationAboutNormal) {
  Acts::Transform3 transform = Acts::Transform3::Identity();
  transform.rotate(Acts::AngleAxis3(rotationAboutNormal, Acts::Vector3::UnitZ()));
  return Acts::Surface::makeShared<Acts::PlaneSurface>(transform, std::make_shared<Acts::RectangleBounds>(10., 10.));
}

// u and v given as (theta, phi), the edm4hep convention
edm4hep::MutableTrackerHitPlane makeHit(float uTheta, float uPhi, float vTheta, float vPhi) {
  edm4hep::MutableTrackerHitPlane hit{};
  hit.setU({uTheta, uPhi});
  hit.setV({vTheta, vPhi});
  hit.setDu(kDu);
  hit.setDv(kDv);
  return hit;
}

Acts::SquareMatrix2 covariance(const Acts::PlaneSurface& surface, const edm4hep::TrackerHitPlane& hit) {
  return ACTSTracking::hitLocalCovariance(Acts::GeometryContext::dangerouslyDefaultConstruct(), surface,
                                          Acts::Vector3::Zero(), hit);
}
} // namespace

TEST_CASE("hitLocalCovariance u/v aligned with the local axes", "[hitcov]") {
  const auto cov = covariance(*makePlane(0.), makeHit(kHalfPi, 0.f, kHalfPi, kHalfPi));
  REQUIRE_THAT(cov(0, 0), WithinAbs(kDu * kDu, kTol));
  REQUIRE_THAT(cov(1, 1), WithinAbs(kDv * kDv, kTol));
  REQUIRE_THAT(cov(0, 1), WithinAbs(0., kTol));
}

TEST_CASE("hitLocalCovariance u/v swapped with respect to the local axes", "[hitcov]") {
  const auto cov = covariance(*makePlane(0.), makeHit(kHalfPi, kHalfPi, kHalfPi, 0.f));
  REQUIRE_THAT(cov(0, 0), WithinAbs(kDv * kDv, kTol));
  REQUIRE_THAT(cov(1, 1), WithinAbs(kDu * kDu, kTol));
  REQUIRE_THAT(cov(0, 1), WithinAbs(0., kTol));
}

TEST_CASE("hitLocalCovariance surface rotated by 45 degrees about its normal", "[hitcov]") {
  // local x = (1, 1)/sqrt2, local y = (-1, 1)/sqrt2; u along global x, v along global y
  const auto cov = covariance(*makePlane(std::numbers::pi / 4), makeHit(kHalfPi, 0.f, kHalfPi, kHalfPi));
  const double sum = kDu * kDu + kDv * kDv;
  const double diff = kDu * kDu - kDv * kDv;
  REQUIRE_THAT(cov(0, 0), WithinAbs(sum / 2, kTol));
  REQUIRE_THAT(cov(1, 1), WithinAbs(sum / 2, kTol));
  REQUIRE_THAT(cov(0, 1), WithinAbs(-diff / 2, kTol));
  REQUIRE_THAT(cov(1, 0), WithinAbs(cov(0, 1), kTol));
  REQUIRE_THAT(cov.trace(), WithinAbs(sum, kTol));
}

TEST_CASE("hitLocalCovariance rejects unset u/v", "[hitcov]") {
  REQUIRE_THROWS_AS(covariance(*makePlane(0.), makeHit(0.f, 0.f, 0.f, 0.f)), std::runtime_error);
}
