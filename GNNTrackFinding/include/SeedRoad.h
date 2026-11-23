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

// Seed inputs and road (seed, hit) edges of the seeded object condensation
// model. Port of acorn/utils/helix_utils.py (helix_residuals, build_road_edges)
// and of get_seed_data / _edge_inputs in
// acorn/stages/graph_construction/models/object_condensation_metric.py.
// Everything is in double precision, as the geometry is ill-conditioned for
// nearly straight tracks. No ONNX or Acts dependency.

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace SeedRoad {

inline constexpr double kCMmPerNs = 299.792458;
inline constexpr double kKappaStraight = 1e-9; // below this |kappa| the helix is treated as a straight line
inline constexpr double kTanLambdaFlat = 1e-9;
// Same number of Newton iterations as the default of helix_residuals in training
inline constexpr int kNewtonSteps = 3;

using Vec3 = std::array<double, 3>;
/// Indices of the three hits every seed was fitted to
using SeedHits = std::array<int64_t, 3>;

/// Perigee parameters of a seed, in mm and ns. kappa = 1/R is signed, positive
/// meaning counter-clockwise in xy along the direction of travel.
struct SeedParams {
  double d0{0.};
  double z0{0.};
  double phi0{0.};
  double tanLambda{0.};
  double kappa{0.};
  double t0{0.};
};

/// Hit positions in mm and, if available, times in ns (empty otherwise)
struct HitGeometry {
  std::vector<Vec3> xyz{};
  std::vector<double> t{};
};

/// Seed features fed to the network, with the scales that bring them to O(1)
struct SeedFeatureOptions {
  /// Any of d0, z0, phi0, sin_phi0, cos_phi0, tan_lambda, kappa, t0
  std::vector<std::string> features{"d0", "z0", "sin_phi0", "cos_phi0", "tan_lambda", "kappa", "t0"};
  /// One scale per feature
  std::vector<double> scales{10., 100., 1., 1., 1., 1e-3, 1.};
};

/// Cuts that define the road of (seed, hit) candidates around every seed helix.
/// Defaults are the values of metric_learning_oc_seeds_train.yaml.
struct RoadOptions {
  /// rad, cone opening angle; widens the cuts with the arc length
  double coneAngle{0.2};
  /// mm, maximum distance in the transverse plane
  double maxPerp{0.5};
  /// mm, maximum distance normal to the helix in the (tangent, z) plane
  double maxN{1.0};
  /// ns, maximum time residual; hits without a time are not cut on it
  std::optional<double> maxDt{0.4};
  /// mm, hits behind the perigee are dropped
  double minS{0.0};
  /// mm, hits further along the helix than this are dropped
  std::optional<double> maxS{2000.0};
  /// Keep only this many hits per seed, closest first
  std::optional<int> maxPerSeed{};
};

/// Road residuals of the hit-to-seed edges, with the scales that bring them to O(1)
struct SeedHitEdgeOptions {
  /// Any of s, d_perp, d_n, d_t. Empty gives one constant input per edge.
  std::vector<std::string> features{"s", "d_perp", "d_n", "d_t"};
  /// One scale per feature
  std::vector<double> scales{1000., 5., 10., 1.};
};

struct Options {
  SeedFeatureOptions seedFeatures{};
  RoadOptions road{};
  SeedHitEdgeOptions seedHitEdges{};
};

/// Helix residuals of a hit with respect to one seed
struct Residuals {
  double s{0.};     // arc length of the closest point
  double dPerp{0.}; // in the transverse plane, left of the direction of travel
  double dN{0.};    // normal to the helix in the (tangent, z) plane
  double dT{0.};    // hit time minus expected time, NaN without a hit time
};

/// Flat road edges sorted by (seed, hit)
struct RoadEdges {
  std::vector<int64_t> seed{};
  std::vector<int64_t> hit{};
  std::vector<double> s{};
  std::vector<double> dPerp{};
  std::vector<double> dN{};
  std::vector<double> dT{};

  std::size_t size() const { return seed.size(); }
};

/// The inputs of the seeded network for one event, in the order of its inputs
struct NetworkInputs {
  std::vector<float> seedInputs{}; // numSeeds x numSeedFeatures
  std::vector<int64_t> seedHits{}; // numSeeds x 3
  std::vector<int64_t> edgeSeed{}; // numEdges
  std::vector<int64_t> edgeHit{};  // numEdges
  std::vector<float> edgeInputs{}; // numEdges x numEdgeFeatures, or x1 without features
};

inline bool isStraight(double kappa) { return std::abs(kappa) < kKappaStraight; }

// Position at transverse arc length s from the perigee
inline Vec3 helixPosition(const SeedParams& p, double s) {
  const bool straight = isStraight(p.kappa);
  const double k = straight ? 1. : p.kappa; // div-0 protection for linear approx in straight case
  const double sinPhi0 = std::sin(p.phi0);
  const double cosPhi0 = std::cos(p.phi0);
  const double xp = -p.d0 * sinPhi0; // POCA
  const double yp = p.d0 * cosPhi0;
  const double ang = p.phi0 + k * s; // azimuthal angle at arc length s
  // straight line approx: x = x0 + s * cos(phi0); circular helix: x = x_c + R * sin(phi) - R * sin(phi0)
  const double x = straight ? xp + s * cosPhi0 : xp + (std::sin(ang) - sinPhi0) / k;
  // straight line approx: y = y0 + s * sin(phi0); circular helix: y = y_c - R * cos(phi) + R * cos(phi0)
  const double y = straight ? yp + s * sinPhi0 : yp - (std::cos(ang) - cosPhi0) / k;
  return {x, y, p.z0 + s * p.tanLambda};
}

// Residuals of a hit position w.r.t. helix defined by seed parameters
inline Residuals helixResiduals(const SeedParams& p, const Vec3& pos, std::optional<double> t) {
  const bool straight = isStraight(p.kappa);
  const double k = straight ? 1. : p.kappa; // div-0 protection for linear approx in straight case
  const double sinPhi0 = std::sin(p.phi0);
  const double cosPhi0 = std::cos(p.phi0);
  const double xp = -p.d0 * sinPhi0; // POCA
  const double yp = p.d0 * cosPhi0;

  const double cx = xp - sinPhi0 / k; // circle centre in xy plane
  const double cy = yp + cosPhi0 / k;
  const double sign = k > 0 ? 1. : -1.;
  const double wx = sign * sinPhi0; // signed direction vector of curvature center from POCA
  const double wy = -sign * cosPhi0;
  const double rx = pos[0] - cx; // hit residual from circle centre
  const double ry = pos[1] - cy;
  // straight line: projection of (hit - POCA) onto helix direction;
  // circular helix: arc length along helix circle, s = delta phi / k = arctan(w x r / w . r) / k
  const double sXY = straight ? (pos[0] - xp) * cosPhi0 + (pos[1] - yp) * sinPhi0
                              : std::atan2(wx * ry - wy * rx, wx * rx + wy * ry) / k;
  const bool hasSlope = std::abs(p.tanLambda) > kTanLambdaFlat;
  // non-zero slope: dz/ds = tan_l => ds = dz / tan_l; zero slope: xy projection as fallback
  const double sZ = hasSlope ? (pos[2] - p.z0) / p.tanLambda : sXY;

  auto dist2 = [&](double s) {
    const Vec3 h = helixPosition(p, s);
    return (h[0] - pos[0]) * (h[0] - pos[0]) + (h[1] - pos[1]) * (h[1] - pos[1]) + (h[2] - pos[2]) * (h[2] - pos[2]);
  };
  // Starting point for the Newton iteration: the s that gives the smaller distance to the hit
  double s = dist2(sZ) < dist2(sXY) ? sZ : sXY;

  const double kap = straight ? 0. : p.kappa; // correct zero curvature for straight segments
  // Newton iteration on f(s) . tang(s) = 0: the vector from helix to hit is perpendicular to the tangent
  for (int i = 0; i < kNewtonSteps; ++i) {
    const Vec3 h = helixPosition(p, s);
    const Vec3 f{h[0] - pos[0], h[1] - pos[1], h[2] - pos[2]};
    const double ang = p.phi0 + kap * s;
    const Vec3 tang{std::cos(ang), std::sin(ang), p.tanLambda};     // tangent vector of the helix at s
    const Vec3 curv{-std::sin(ang) * kap, std::cos(ang) * kap, 0.}; // curvature vector of the helix at s
    const double t2 = tang[0] * tang[0] + tang[1] * tang[1] + tang[2] * tang[2];
    const double num = f[0] * tang[0] + f[1] * tang[1] + f[2] * tang[2]; // projection of f onto tang
    // d(f . t)/ds = df/ds . t + f . dt/ds = t^2 + f . kappa
    const double den = t2 + f[0] * curv[0] + f[1] * curv[1] + f[2] * curv[2];
    s -= num / (den > 1e-6 ? den : t2); // s - (f . t) / (t^2 + f . kappa)
  }

  const double ang = p.phi0 + kap * s;                            // final angle at converged s
  const Vec3 ePerp{-std::sin(ang), std::cos(ang), 0.};            // unit vector perp to helix at angle ang
  const double normT = std::sqrt(1. + p.tanLambda * p.tanLambda); // norm of tangent incl. z slope
  // unit vector along the normal direction to the helix, including the z slope
  const Vec3 n2{-p.tanLambda * std::cos(ang) / normT, -p.tanLambda * std::sin(ang) / normT, 1. / normT};
  const Vec3 h = helixPosition(p, s);
  const Vec3 r{pos[0] - h[0], pos[1] - h[1], pos[2] - h[2]}; // final residual vector from helix to point

  Residuals res;
  res.s = s;
  res.dPerp = r[0] * ePerp[0] + r[1] * ePerp[1];
  res.dN = r[0] * n2[0] + r[1] * n2[1] + r[2] * n2[2];
  res.dT = t ? *t - (p.t0 + s * normT / kCMmPerNs) : std::numeric_limits<double>::quiet_NaN();
  return res;
}

namespace detail {

  inline const std::vector<double>& roadColumn(const RoadEdges& road, std::string_view name) {
    if (name == "s")
      return road.s;
    if (name == "d_perp")
      return road.dPerp;
    if (name == "d_n")
      return road.dN;
    if (name == "d_t")
      return road.dT;
    throw std::invalid_argument("Unknown edge feature '" + std::string(name) + "'");
  }

  // nan_to_num as in torch, after the cast to float32
  inline float nanToNumFloat(double x) {
    if (std::isnan(x)) {
      return 0.f;
    }
    const double maxF = static_cast<double>(std::numeric_limits<float>::max());
    return static_cast<float>(std::clamp(x, -maxF, maxF));
  }

  // asinh(x / scale) in float32, as the training side
  inline float asinhScaled(double x, double scale) {
    const float scaled = nanToNumFloat(x) / static_cast<float>(scale);
    return std::asinh(scaled);
  }

} // namespace detail

/// Value of a seed feature. sin_phi0 and cos_phi0 are taken from phi0.
inline double seedFeatureValue(const SeedParams& p, std::string_view name) {
  if (name == "d0")
    return p.d0;
  if (name == "z0")
    return p.z0;
  if (name == "phi0")
    return p.phi0;
  if (name == "sin_phi0")
    return std::sin(p.phi0);
  if (name == "cos_phi0")
    return std::cos(p.phi0);
  if (name == "tan_lambda")
    return p.tanLambda;
  if (name == "kappa")
    return p.kappa;
  if (name == "t0")
    return p.t0;
  throw std::invalid_argument("Unknown seed feature '" + std::string(name) + "'");
}

/// Hit geometry from the node features, row-major numHits x names.size(). The columns are
/// found by name (case insensitive), so their order does not matter. Positions come as x, y, z
/// or as r, phi, z, in which case x and y are derived. Time is read from t if it is a feature.
/// The values must be unscaled (mm, rad, ns).
inline HitGeometry hitGeometryFromFeatures(const std::vector<float>& nodeFeatures, std::size_t numHits,
                                           const std::vector<std::string>& names) {
  if (nodeFeatures.size() != numHits * names.size()) {
    throw std::invalid_argument("node features need numHits x numFeatures values");
  }
  auto column = [&](std::string_view wanted) -> std::optional<std::size_t> {
    for (std::size_t f = 0; f < names.size(); ++f) {
      std::string lower = names[f];
      std::transform(lower.begin(), lower.end(), lower.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (lower == wanted) {
        return f;
      }
    }
    return std::nullopt;
  };

  const auto iz = column("z");
  const auto ix = column("x");
  const auto iy = column("y");
  const auto ir = column("r");
  const auto iphi = column("phi");
  const auto it = column("t");
  const bool cartesian = ix && iy;
  if (!iz || (!cartesian && !(ir && iphi))) {
    throw std::invalid_argument("hit features need z and either x, y or r, phi");
  }

  const std::size_t numFeatures = names.size();
  HitGeometry hits;
  hits.xyz.resize(numHits);
  if (it) {
    hits.t.resize(numHits);
  }
  for (std::size_t j = 0; j < numHits; ++j) {
    const float* row = nodeFeatures.data() + j * numFeatures;
    const double z = row[*iz];
    if (cartesian) {
      hits.xyz[j] = {row[*ix], row[*iy], z};
    } else {
      const double r = row[*ir];
      const double phi = row[*iphi];
      hits.xyz[j] = {r * std::cos(phi), r * std::sin(phi), z};
    }
    if (it) {
      hits.t[j] = row[*it];
    }
  }
  return hits;
}

/// Candidate (seed, hit) pairs inside the road of every seed, sorted by (seed, hit).
/// The seed hits are always kept.
///
/// Hits behind the perigee (s < minS) are dropped, and with maxS so are hits further along the
/// helix than that arc length (mm). Those only exist for seeds that curl up or climb in z far
/// outside the detector. maxDt (ns) additionally cuts on the time residual where it is defined.
/// With maxPerSeed, only the hits with the smallest normalised residual are kept per seed.
inline RoadEdges buildRoadEdges(const std::vector<SeedParams>& seeds, const HitGeometry& hits, const RoadOptions& opt,
                                const std::vector<SeedHits>& seedHits) {
  const std::size_t numHits = hits.xyz.size();
  if (!hits.t.empty() && hits.t.size() != numHits) {
    throw std::invalid_argument("HitGeometry needs one time per hit or none");
  }
  if (seedHits.size() != seeds.size()) {
    throw std::invalid_argument("seedHits needs one entry per seed");
  }
  const bool hasTime = !hits.t.empty();

  struct Entry {
    int64_t hit;
    Residuals res;
  };

  RoadEdges out;
  for (std::size_t i = 0; i < seeds.size(); ++i) {
    const SeedParams& p = seeds[i];
    const bool straight = isStraight(p.kappa);
    const double k = straight ? 1. : p.kappa;
    const double sinPhi0 = std::sin(p.phi0);
    const double cosPhi0 = std::cos(p.phi0);
    const double xp = -p.d0 * sinPhi0;
    const double yp = p.d0 * cosPhi0;
    const double cx = xp - sinPhi0 / k; // circle center
    const double cy = yp + cosPhi0 / k;
    const double radius = 1. / std::abs(k); // circle radius 1/k
    const double sign = k > 0 ? 1. : -1.;
    const double wx = sign * sinPhi0; // as in helixResiduals
    const double wy = -sign * cosPhi0;

    std::vector<Entry> seedEntries;
    for (std::size_t j = 0; j < numHits; ++j) {
      const double hx = hits.xyz[j][0];
      const double hy = hits.xyz[j][1];
      // In-plane distance to the circle (or the line) approximates |d_perp|. The window margin
      // absorbs the 3D refinement that follows.
      const double circ = straight ? std::abs((hx - xp) * sinPhi0 - (hy - yp) * cosPhi0)
                                   : std::abs(std::hypot(hx - cx, hy - cy) - radius);
      double window = 1.5 * opt.maxPerp;
      if (opt.coneAngle > 0) {
        // The cone widens the window with the arc length, so the candidate selection widens with it
        // too. The in-plane arc length is enough here, the exact residuals and cuts follow below.
        const double rx = hx - cx;
        const double ry = hy - cy;
        const double sXY =
            straight ? (hx - xp) * cosPhi0 + (hy - yp) * sinPhi0 : std::atan2(wx * ry - wy * rx, wx * rx + wy * ry) / k;
        window = 1.5 * (opt.maxPerp + opt.coneAngle * std::abs(sXY));
      }
      if (!(circ < window)) {
        continue;
      }

      const std::optional<double> t = hasTime ? std::optional<double>(hits.t[j]) : std::nullopt;
      const Residuals res = helixResiduals(p, hits.xyz[j], t);
      const double cone = opt.coneAngle * std::abs(res.s); // allow larger d_perp, d_n for hits further along the helix
      bool keep = std::abs(res.dPerp) < opt.maxPerp + cone && std::abs(res.dN) < opt.maxN + cone && res.s >= opt.minS;
      if (opt.maxS) {
        keep = keep && res.s <= *opt.maxS;
      }
      if (opt.maxDt) {
        // NaN time residuals pass
        keep = keep && !(std::abs(res.dT) >= *opt.maxDt);
      }
      if (keep) {
        seedEntries.push_back({static_cast<int64_t>(j), res});
      }
    }

    if (opt.maxPerSeed && seedEntries.size() > static_cast<std::size_t>(*opt.maxPerSeed)) {
      // 3D norm distance in units of the cuts
      auto score = [&](const Entry& e) { return std::hypot(e.res.dPerp / opt.maxPerp, e.res.dN / opt.maxN); };
      std::stable_sort(seedEntries.begin(), seedEntries.end(),
                       [&](const Entry& a, const Entry& b) { return score(a) < score(b); });
      seedEntries.resize(static_cast<std::size_t>(*opt.maxPerSeed));
    }

    // The seed hits are always part of the road
    for (int64_t hitIndex : seedHits[i]) {
      if (hitIndex < 0 || static_cast<std::size_t>(hitIndex) >= numHits) {
        throw std::out_of_range("seed hit index outside of the hits");
      }
      const bool present =
          std::any_of(seedEntries.begin(), seedEntries.end(), [&](const Entry& e) { return e.hit == hitIndex; });
      if (!present) {
        const std::optional<double> t = hasTime ? std::optional<double>(hits.t[hitIndex]) : std::nullopt;
        seedEntries.push_back({hitIndex, helixResiduals(p, hits.xyz[hitIndex], t)});
      }
    }

    std::sort(seedEntries.begin(), seedEntries.end(), [](const Entry& a, const Entry& b) { return a.hit < b.hit; });
    for (const Entry& e : seedEntries) {
      out.seed.push_back(static_cast<int64_t>(i));
      out.hit.push_back(e.hit);
      out.s.push_back(e.res.s);
      out.dPerp.push_back(e.res.dPerp);
      out.dN.push_back(e.res.dN);
      out.dT.push_back(e.res.dT);
    }
  }
  return out;
}

/// asinh(residual / scale) of every edge, row-major numEdges x numFeatures.
/// Without features, a single constant input per edge.
inline std::vector<float> buildEdgeInputs(const RoadEdges& road, const SeedHitEdgeOptions& opt) {
  if (opt.features.empty()) {
    return std::vector<float>(road.size(), 1.f);
  }
  if (opt.features.size() != opt.scales.size()) {
    throw std::invalid_argument("seedHitEdges features and scales need the same number of entries");
  }
  const std::size_t numFeatures = opt.features.size();
  std::vector<float> out(road.size() * numFeatures);
  for (std::size_t f = 0; f < numFeatures; ++f) {
    const std::vector<double>& column = detail::roadColumn(road, opt.features[f]);
    for (std::size_t e = 0; e < road.size(); ++e) {
      out[e * numFeatures + f] = detail::asinhScaled(column[e], opt.scales[f]);
    }
  }
  return out;
}

/// asinh(feature / scale) of every seed, row-major numSeeds x numFeatures
inline std::vector<float> buildSeedInputs(const std::vector<SeedParams>& seeds, const SeedFeatureOptions& opt) {
  if (opt.features.size() != opt.scales.size()) {
    throw std::invalid_argument("seedFeatures and seedScales need the same number of entries");
  }
  const std::size_t numFeatures = opt.features.size();
  std::vector<float> out(seeds.size() * numFeatures);
  for (std::size_t i = 0; i < seeds.size(); ++i) {
    for (std::size_t f = 0; f < numFeatures; ++f) {
      out[i * numFeatures + f] = detail::asinhScaled(seedFeatureValue(seeds[i], opt.features[f]), opt.scales[f]);
    }
  }
  return out;
}

} // namespace SeedRoad
