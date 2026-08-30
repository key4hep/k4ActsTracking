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

// Shared ACTS type aliases and small helpers used by both the combinatorial
// (CKFRunner) and standalone-fit (KFRunner) track runners.

// k4ActsTracking
#include "k4ActsTracking/CKFTracking.hxx"
#include "k4ActsTracking/Helpers.hxx"
#include "k4ActsTracking/IActsGeoSvc.h"
#include "k4ActsTracking/Measurement.hxx"
#include "k4ActsTracking/SourceLink.hxx"

// edm4hep
#include <edm4hep/MutableTrack.h>
#include <edm4hep/TrackState.h>
#include <edm4hep/TrackerHit.h>
#include <edm4hep/TrackerHitPlaneCollection.h>

// podio
#include <podio/ObjectID.h>

// ACTS
#include <Acts/Definitions/Units.hpp>
#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/EventData/ParticleHypothesis.hpp>
#include <Acts/EventData/SourceLink.hpp>
#include <Acts/EventData/TrackContainer.hpp>
#include <Acts/EventData/TrackStateType.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>
#include <Acts/EventData/VectorTrackContainer.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Geometry/GeometryIdentifier.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Propagator/EigenStepper.hpp>
#include <Acts/Propagator/Navigator.hpp>
#include <Acts/Propagator/Propagator.hpp>
#include <Acts/Propagator/StraightLineStepper.hpp>
#include <Acts/Seeding/EstimateTrackParamsFromSeed.hpp>
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/Utilities/Result.hpp>

// ActsPlugins: centralised ACTS -> EDM4hep conversion
#include <ActsPlugins/EDM4hep/EDM4hepUtil.hpp>

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
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ACTSTracking {

  /// Default maximum number of propagation steps shared by the track runners
  /// (CKFRunner, KFRunner). Single source of truth for the runners' Config
  /// defaults and the algorithms' MaxPropagationSteps property default.
  inline constexpr std::size_t kDefaultMaxPropagationSteps = 10000;

  /// Track container shared by the CKF and Kalman-fit runners.
  using CKFTrackContainer =
      Acts::TrackContainer<Acts::VectorTrackContainer, Acts::VectorMultiTrajectory, std::shared_ptr>;

  /// Stepper / navigator / propagator shared by the runners.
  using CKFStepper    = Acts::EigenStepper<>;
  using CKFNavigator  = Acts::Navigator;
  using CKFPropagator = Acts::Propagator<CKFStepper, CKFNavigator>;

  /// Build a propagator over the tracking geometry. @p resolvePassive selects
  /// whether passive surfaces (e.g. calorimeter faces) are resolved during
  /// navigation. The result depends only on the geometry and field, so callers
  /// can build the propagators they need once and reuse them across events.
  inline CKFPropagator makePropagator(const IActsGeoSvc& geo, bool resolvePassive) {
    CKFNavigator::Config navigatorCfg{geo.trackingGeometry()};
    navigatorCfg.resolvePassive   = resolvePassive;
    navigatorCfg.resolveMaterial  = true;
    navigatorCfg.resolveSensitive = true;
    return CKFPropagator(CKFStepper(geo.magneticField()), CKFNavigator(navigatorCfg));
  }

  /// Field-free propagator used for geantino work (material validation).
  using GeantinoPropagator = Acts::Propagator<Acts::StraightLineStepper, CKFNavigator>;

  /// Build a straight-line propagator over the tracking geometry, for shooting
  /// geantinos through it.
  ///
  /// This deliberately does *not* use @c makePropagator. A geantino is neutral,
  /// and @c Acts::ChargeHypothesis::extractMomentum returns
  /// `charge / qOverP`, i.e. exactly 0 for a neutral particle whatever q/p is.
  /// @c Acts::detail::setupLoopProtection then computes a full helix path of
  /// `2*pi*p/B == 0` and clamps the propagator's path limit to zero, so with any
  /// non-zero field the propagation aborts before its first step and collects
  /// nothing. A field-free stepper sidesteps that (it is also what ACTS' own
  /// PropagatorMaterialAssigner unit test uses), and is the physically correct
  /// choice anyway: the Geant4 geantino scan the validation compares against is
  /// made of straight rays.
  inline GeantinoPropagator makeGeantinoPropagator(const IActsGeoSvc& geo, bool resolvePassive) {
    CKFNavigator::Config navigatorCfg{geo.trackingGeometry()};
    navigatorCfg.resolvePassive   = resolvePassive;
    navigatorCfg.resolveMaterial  = true;
    navigatorCfg.resolveSensitive = true;
    return GeantinoPropagator(Acts::StraightLineStepper(), CKFNavigator(navigatorCfg));
  }

  /**
   * @brief Thread-safe counters for the calorimeter-face extrapolation.
   *
   * Shared by the tracking algorithms: CaloStateAppender updates the counters
   * (through a const pointer, hence the mutable atomics) and each algorithm
   * reports summary() in finalize().
   */
  struct CaloExtrapMonitor {
    mutable std::atomic<std::size_t> attempts{0};      ///< tracks with a usable start state
    mutable std::atomic<std::size_t> noStartState{0};  ///< tracks without a measured smoothed state
    mutable std::atomic<std::size_t> notReached{0};    ///< propagation did not reach a calo face
    mutable std::atomic<std::size_t> propFailed{0};    ///< propagation itself failed
    mutable std::atomic<std::size_t> ok{0};            ///< reached a calo face
    /// The next two are only filled when per-section calo states are enabled
    /// (CaloStateAppender::Config::addEndcapState); the summary then reports them.
    mutable std::atomic<std::size_t> barrel{0};          ///< the face reached first was a barrel face
    mutable std::atomic<std::size_t> barrelToEndcap{0};  ///< barrel tracks that also crossed an endcap disc

    std::string summary() const {
      const std::size_t a        = attempts.load();
      const std::size_t o        = ok.load();
      const std::size_t nr       = notReached.load();
      const std::size_t pf       = propFailed.load();
      const std::size_t ns       = noStartState.load();
      const std::size_t b        = barrel.load();
      const std::size_t b2e      = barrelToEndcap.load();
      const std::size_t failed   = nr + pf;
      const double      failRate = a > 0 ? static_cast<double>(failed) / static_cast<double>(a) : 0.0;
      std::string       text     = fmt::format(
          "Calorimeter-face extrapolation summary: {} attempts, {} reached the face, {} failed "
                    "({:.2f}%: {} not reached, {} propagation errors); {} tracks had no measured smoothed start state.",
          a, o, failed, 100.0 * failRate, nr, pf, ns);
      if (b > 0) {
        text += fmt::format(
            " {} reached the barrel face, of which {} also reached an endcap disc (second AtCalorimeter state).", b,
            b2e);
      }
      return text;
    }
  };

  /**
   * @brief Adds AtCalorimeter track states to a fitted track.
   *
   * Owns the geometry-aware calorimeter-face propagator and turns the
   * extrapolation result into edm4hep track states. Both track runners hold one
   * (CKFRunner after the combinatorial find, KFRunner after a standalone fit),
   * so a track gets the same AtCalorimeter states however it was reconstructed.
   *
   * Build one instance per algorithm and reuse it: the propagator depends only
   * on the tracking geometry and field, so it is read-only and safe to share
   * across concurrent fits. Non-copyable and non-movable (it holds the
   * propagator by unique_ptr).
   */
  class CaloStateAppender {
  public:
    struct Config {
      /// Extrapolate fitted tracks to the calorimeter face at all. When false
      /// (the default) the appender is inert and adds nothing.
      bool enabled = false;
      /// Give a track that crosses both calorimeter sections one AtCalorimeter
      /// state per section instead of a single one; see addEndcapStateAfterBarrel.
      bool        addEndcapState = false;
      std::size_t maxSteps       = kDefaultMaxPropagationSteps;
    };

    CaloStateAppender(const IActsGeoSvc& geo, const Acts::GeometryContext& geoCtx,
                      const Acts::MagneticFieldContext& magCtx, const Config& cfg)
        : m_geo(geo),
          m_geoCtx(geoCtx),
          m_magCtx(magCtx),
          m_maxSteps(cfg.maxSteps),
          m_addEndcapState(cfg.addEndcapState) {
      // The calorimeter inner-face surfaces are passive surfaces of the tracking
      // geometry, so the calo propagator's navigator must resolve passive
      // surfaces. Only built when requested and when the geometry provides them.
      if (cfg.enabled && !geo.caloSurfaceGeoIds().empty()) {
        m_propagator = std::make_unique<ACTSTracking::CaloFacePropagator>(makePropagator(geo, true));
      }
    }

    CaloStateAppender(const CaloStateAppender&)            = delete;
    CaloStateAppender(CaloStateAppender&&)                 = delete;
    CaloStateAppender& operator=(const CaloStateAppender&) = delete;
    CaloStateAppender& operator=(CaloStateAppender&&)      = delete;

    /// Whether this appender will actually produce track states.
    bool active() const { return m_propagator != nullptr; }

    /// Extrapolate the smoothed track to the calorimeter face and, on success,
    /// append an AtCalorimeter track state to @p track. No-op when calo
    /// extrapolation is disabled. Starts from the measured smoothed state the
    /// particle reaches last, i.e. the one facing the calorimeter; see
    /// findStartState.
    ///
    /// With Config::addEndcapState set, a track that crosses both calorimeter
    /// sections gets one AtCalorimeter state per section; see
    /// addEndcapStateAfterBarrel. Otherwise every track keeps exactly one
    /// AtCalorimeter state, at the first calo face it reaches.
    ///
    /// @param trackTip     Fitted ACTS track (CKF result or standalone KF result).
    /// @param track        edm4hep track the state(s) are appended to.
    /// @param caloMonitor  Optional counters; may be null.
    template <class Alg, class TrackProxy>
    void addCaloState(const Alg& alg, const TrackProxy& trackTip, edm4hep::MutableTrack& track,
                      Acts::MagneticFieldProvider::Cache& magCache, const CaloExtrapMonitor* caloMonitor) const {
      if (!m_propagator) {
        return;
      }

      const auto startParams = findStartParams(trackTip);

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
      const ACTSTracking::CaloFacePropagator& caloPropagator = *m_propagator;
      const auto                              caloResult     = ACTSTracking::extrapolateToCaloFace(
          caloPropagator, *startParams, m_geo.caloSurfaceGeoIds(), m_geoCtx, m_magCtx, m_maxSteps);

      using ACTSTracking::CaloExtrapolationStatus;
      switch (caloResult.status) {
        case CaloExtrapolationStatus::Ok: {
          if (caloMonitor) {
            ++caloMonitor->ok;
          }
          appendCaloState(track, *caloResult.params, magCache);
          addEndcapStateAfterBarrel(alg, track, *caloResult.params, magCache, caloMonitor);
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

  private:
    /// Pick the measured, smoothed state to extrapolate to the calorimeter from:
    /// the end of the trajectory the particle reaches last.
    ///
    /// The obvious "take the first state of trackStatesReversed()" does not work,
    /// because that iterates from whichever end the fit finished on. For an
    /// outward fit (the default) that is the outermost hit, but with
    /// PropagateBackward the fit runs towards the beamline and the very same
    /// expression yields the *innermost* hit, so the extrapolation would set off
    /// from the hit closest to the beam and cross the whole tracker blind.
    ///
    /// Both ends of the measured trajectory are therefore collected and compared
    /// geometrically: the downstream end is the one whose momentum points away
    /// from the other end. That makes the choice independent of the fit
    /// direction, and of the detector shape - it selects the outermost hit of an
    /// outward collider track and the most downstream hit of a beam-parallel
    /// telescope track alike.
    ///
    /// @return The chosen parameters, or std::nullopt if the track carries no
    ///         measured smoothed state at all.
    template <class TrackProxy>
    std::optional<Acts::BoundTrackParameters> findStartParams(const TrackProxy& trackTip) const {
      std::optional<Acts::BoundTrackParameters> firstEnd;  ///< first in iteration order
      std::optional<Acts::BoundTrackParameters> lastEnd;   ///< last in iteration order

      for (const auto& state : trackTip.trackStatesReversed()) {
        const auto flags = state.typeFlags();
        if (!state.hasSmoothed() || !flags.test(Acts::TrackStateFlag::HasMeasurement) ||
            flags.test(Acts::TrackStateFlag::IsOutlier)) {
          continue;
        }
        Acts::BoundTrackParameters params(state.referenceSurface().getSharedPtr(), state.smoothed(),
                                          state.smoothedCovariance(), trackTip.particleHypothesis());
        if (!firstEnd) {
          firstEnd = params;
        }
        lastEnd = std::move(params);
      }

      if (!firstEnd) {
        return std::nullopt;
      }

      // Momentum at firstEnd projected on the vector pointing from lastEnd to
      // firstEnd: positive means moving forward from firstEnd leads away from
      // lastEnd, so firstEnd is the downstream end. A single measured state
      // makes the two identical and the projection zero, which picks it either
      // way.
      const Acts::Vector3 lastToFirst = firstEnd->position(m_geoCtx) - lastEnd->position(m_geoCtx);
      return lastToFirst.dot(firstEnd->direction()) >= 0 ? firstEnd : lastEnd;
    }

    /// Convert on-surface calorimeter-face parameters into an edm4hep
    /// AtCalorimeter track state and append it to @p track.
    void appendCaloState(edm4hep::MutableTrack& track, const Acts::BoundTrackParameters& params,
                         Acts::MagneticFieldProvider::Cache& magCache) const {
      // The calo-face parameters are local to the target surface; the
      // centralised converter re-expresses them at an ad-hoc perigee at the
      // calo-face position, sets the referencePoint accordingly, and evaluates
      // the local field there.
      track.addToTrackStates(ActsPlugins::EDM4hepUtil::writeTrackState(
          m_geoCtx, edm4hep::TrackState::AtCalorimeter, params, *m_geo.magneticField(), magCache));
    }

    /// Continue the calorimeter extrapolation from a barrel-face crossing out to
    /// the endcap discs and, when one is reached, append a second AtCalorimeter
    /// state.
    ///
    /// The barrel face and the endcap discs meet at the hermetic corner, so a
    /// track crossing the barrel face at large |z| goes on to cross the endcap
    /// disc as well (the disc radius extends past the barrel corner). Such a
    /// track really does enter both calorimeter sections, and a downstream
    /// client may need the entry point into each; the two states are appended
    /// in the order the track crosses them, barrel first.
    ///
    /// Opt-in via Config::addEndcapState, because a track carrying two states
    /// with the same edm4hep location is not what a client reading "the"
    /// AtCalorimeter state expects. Disabled (the default), every track keeps a
    /// single state at the first face it reaches. Also does nothing when
    /// @p barrelParams are not on a barrel face (the track ended on an endcap
    /// disc or on the telescope planar face) or when the geometry has no endcap
    /// discs.
    template <class Alg>
    void addEndcapStateAfterBarrel(const Alg& alg, edm4hep::MutableTrack& track,
                                   const Acts::BoundTrackParameters&   barrelParams,
                                   Acts::MagneticFieldProvider::Cache& magCache,
                                   const CaloExtrapMonitor*            caloMonitor) const {
      if (!m_addEndcapState) {
        return;
      }

      const auto& barrelIds = m_geo.caloBarrelSurfaceGeoIds();
      const auto& endcapIds = m_geo.caloEndcapSurfaceGeoIds();
      const auto  reachedId = barrelParams.referenceSurface().geometryId();
      if (endcapIds.empty() || std::find(barrelIds.begin(), barrelIds.end(), reachedId) == barrelIds.end()) {
        return;
      }

      if (caloMonitor) {
        ++caloMonitor->barrel;
      }

      // Restart the propagation from the barrel face, this time aborting only on
      // the endcap discs. Most barrel tracks leave through the outer boundary
      // without ever reaching the endcap z, which is not an error: only the ones
      // crossing the barrel/endcap corner region get the extra state.
      const ACTSTracking::CaloFacePropagator& caloPropagator = *m_propagator;
      const auto                              endcapResult =
          ACTSTracking::extrapolateToCaloFace(caloPropagator, barrelParams, endcapIds, m_geoCtx, m_magCtx, m_maxSteps);

      if (endcapResult.status != ACTSTracking::CaloExtrapolationStatus::Ok) {
        alg.verbose() << "Barrel calo-face track did not reach an endcap disc; keeping a single AtCalorimeter state."
                      << endmsg;
        return;
      }

      if (caloMonitor) {
        ++caloMonitor->barrelToEndcap;
      }
      appendCaloState(track, *endcapResult.params, magCache);
    }

    const IActsGeoSvc&                                m_geo;
    Acts::GeometryContext                             m_geoCtx;
    Acts::MagneticFieldContext                        m_magCtx;
    std::size_t                                       m_maxSteps       = kDefaultMaxPropagationSteps;
    bool                                              m_addEndcapState = false;
    std::unique_ptr<ACTSTracking::CaloFacePropagator> m_propagator;
  };

  /// Stable key for matching an edm4hep tracker hit across collections.
  inline std::uint64_t trackerHitKey(const edm4hep::TrackerHit& hit) {
    const podio::ObjectID id = hit.getObjectID();
    return (static_cast<std::uint64_t>(id.collectionID) << 32) | static_cast<std::uint32_t>(id.index);
  }

  /**
   * @brief Build the ACTS measurements and source links for a set of tracker hits.
   *
   * Shared hit-conversion loop of the tracking algorithms:
   *  - looks up the ACTS surface for each hit's cellID,
   *  - sorts hits by geometry identifier for efficient multiset insertion,
   *  - converts global to local coordinates and creates a Measurement + SourceLink.
   *
   * For each converted hit the @p hitSink callback is invoked with the hit, its
   * SourceLink and the geometry needed to build seeding space points, so each
   * algorithm can decide what to keep (a per-hit source-link map, seed grid
   * inputs, ...).
   *
   * The @p hits container is filled in lockstep with @p measurements: entry i is
   * the edm4hep hit of the source link whose index() is i, so the compact
   * SourceLink can recover its hit via hits[sourceLink.index()].
   *
   * @tparam Alg     Owning Gaudi algorithm (used only for level-aware logging).
   * @tparam HitSink Callback (const edm4hep::TrackerHitPlane&, const SourceLink&,
   *                 const Acts::Vector3& globalPos, const Acts::Surface&,
   *                 const Acts::SquareMatrix2& localCov).
   */
  template <class Alg, class HitSink>
  void prepareTrackerHits(const Alg& alg, const IActsGeoSvc& geo, const Acts::GeometryContext& geoCtx,
                          const edm4hep::TrackerHitPlaneCollection& trackerHits,
                          ACTSTracking::MeasurementContainer&       measurements,
                          ACTSTracking::SourceLinkContainer& sourceLinks, ACTSTracking::HitContainer& hits,
                          int numThreads, HitSink&& hitSink) {
    const auto& cellIdToSurface = geo.cellIdToSurfaceMap();

    std::vector<std::pair<Acts::GeometryIdentifier, edm4hep::TrackerHitPlane>> sortedHits;
    sortedHits.reserve(trackerHits.size());

    for (const auto& hit : trackerHits) {
      auto it = cellIdToSurface.find(hit.getCellID());
      if (it == cellIdToSurface.end()) {
        alg.warning() << "No surface found for cellID " << hit.getCellID() << ". skipping hit for tracking." << endmsg;
        continue;
      }
      sortedHits.push_back({it->second->geometryId(), hit});
    }
    alg.debug() << "Working with " << sortedHits.size() << " hits." << endmsg;

    // Sort hits by geometry ID for efficient SourceLink multiset insertion
    auto compare = [](const auto& a, const auto& b) { return a.first < b.first; };
    if (numThreads > 1) {
      tbb::task_arena arena(numThreads);
      arena.execute([&] { tbb::parallel_sort(sortedHits.begin(), sortedHits.end(), compare); });
    } else {
      std::sort(sortedHits.begin(), sortedHits.end(), compare);
    }

    sourceLinks.reserve(sortedHits.size());
    hits.reserve(sortedHits.size());

    for (const auto& hitPair : sortedHits) {
      const Acts::Surface* surface = geo.trackingGeometry()->findSurface(hitPair.first);
      if (surface == nullptr) {
        alg.warning() << "Surface with geoID " << hitPair.first
                      << " not found in tracking geometry. Skipping hit for tracking." << endmsg;
        continue;
      }

      const edm4hep::Vector3d& edmGlobalPos = hitPair.second.getPosition();
      Acts::Vector3            globalPos    = {edmGlobalPos.x, edmGlobalPos.y, edmGlobalPos.z};

      Acts::Result<Acts::Vector2> lpResult =
          surface->globalToLocal(geoCtx, globalPos, {0, 0, 0}, 0.5 * Acts::UnitConstants::um);
      if (!lpResult.ok()) {
        alg.warning() << "Global to local transformation did not succeed for hit. Skipping it in tracking." << endmsg;
        continue;
      }

      Acts::Vector2 loc = lpResult.value();

      Acts::SquareMatrix2 localCov = Acts::SquareMatrix2::Zero();
      localCov(0, 0)               = std::pow(hitPair.second.getDu() * Acts::UnitConstants::mm, 2);
      localCov(1, 1)               = std::pow(hitPair.second.getDv() * Acts::UnitConstants::mm, 2);

      ACTSTracking::SourceLink  sourceLink(surface->geometryId(), measurements.size());
      Acts::SourceLink          srcWrap{sourceLink};
      ACTSTracking::Measurement meas =
          ACTSTracking::makeMeasurement(srcWrap, loc, localCov, Acts::eBoundLoc0, Acts::eBoundLoc1);

      measurements.push_back(meas);
      hits.push_back(hitPair.second);
      sourceLinks.emplace_hint(sourceLinks.end(), sourceLink);

      hitSink(hitPair.second, sourceLink, globalPos, *surface, localCov);
    }
  }

  /**
   * @brief Estimate initial bound track parameters from a three-point seed.
   *
   * Evaluates the magnetic field at the bottom position, calls
   * Acts::estimateTrackParamsFromSeed on the bottom/middle/top positions and
   * builds a diagonal initial covariance.
   *
   * @return The estimated parameters, or std::nullopt if the estimation fails. A
   *         field-lookup failure throws, matching the tracking algorithms.
   */
  template <class Alg>
  std::optional<Acts::BoundTrackParameters> estimateSeedParameters(
      const Alg& alg, const IActsGeoSvc& geo, const Acts::GeometryContext& geoCtx, const Acts::Surface& bottomSurface,
      const Acts::Vector3& bottomPos, const Acts::Vector3& middlePos, const Acts::Vector3& topPos, double t0,
      Acts::MagneticFieldProvider::Cache& magCache, double errPos, double errPhi, double errLambda, double errRelP,
      double errTime) {
    // Magnetic field at the seed (bottom space point) position
    Acts::Result<Acts::Vector3> seedField = geo.magneticField()->getField(bottomPos, magCache);
    if (!seedField.ok()) {
      throw std::runtime_error("Field lookup error: " + std::to_string(seedField.error().value()));
    }

    Acts::Result<Acts::BoundVector> optParams =
        Acts::estimateTrackParamsFromSeed(geoCtx, bottomSurface, bottomPos, t0, middlePos, topPos, *seedField);
    if (!optParams.ok()) {
      alg.debug() << "Failed estimation of track parameters for seed." << endmsg;
      return std::nullopt;
    }

    const Acts::BoundVector& params = *optParams;
    float                    p      = std::abs(1.f / params[Acts::eBoundQOverP]);

    Acts::BoundMatrix cov = ACTSTracking::makeInitialCovariance(p, errPos, errPhi, errLambda, errRelP, errTime);

    return Acts::BoundTrackParameters(bottomSurface.getSharedPtr(), params, cov, Acts::ParticleHypothesis::pion());
  }

  /// A hit belonging to a seed candidate: global position, transverse radius and
  /// the source link of its ACTS measurement. Once the hits are ordered by
  /// radius the innermost, middle and outermost entries form the three-point
  /// seed.
  struct SeedHit {
    Acts::Vector3            pos;
    double                   r;
    ACTSTracking::SourceLink sl;
  };

  /**
   * @brief Collect the radius-ordered seed hits for a candidate's hits.
   *
   * For every hit in @p candidateHits that has a source link in @p slByHit a
   * SeedHit is built: its global position is taken from the parallel
   * @p hitContainer via the source-link index(), together with the transverse
   * radius and the source link itself. The returned vector is sorted by
   * increasing transverse radius, so front() / [size()/2] / back() form the
   * three-point seed.
   *
   * Hits without a source link are skipped, so the result may be smaller than
   * @p candidateHits; callers apply their own minimum-hit requirement.
   *
   * @tparam HitRange Iterable of edm4hep hits accepted by trackerHitKey().
   */
  template <class HitRange>
  std::vector<SeedHit> collectSeedHits(const HitRange&                                      candidateHits,
                                       const std::unordered_map<std::uint64_t, SourceLink>& slByHit,
                                       const ACTSTracking::HitContainer&                    hitContainer) {
    std::vector<SeedHit> seedHits;
    for (const auto& hit : candidateHits) {
      auto it = slByHit.find(trackerHitKey(hit));
      if (it == slByHit.end()) {
        continue;
      }
      const edm4hep::Vector3d p = hitContainer[it->second.index()].getPosition();
      seedHits.push_back({Acts::Vector3(p.x, p.y, p.z), std::hypot(p.x, p.y), it->second});
    }
    std::sort(seedHits.begin(), seedHits.end(), [](const SeedHit& a, const SeedHit& b) { return a.r < b.r; });
    return seedHits;
  }

  /**
   * @brief Estimate initial bound track parameters from radius-ordered seed hits.
   *
   * Convenience overload for candidates whose hits were collected with
   * collectSeedHits(): it picks the innermost / middle / outermost hit as the
   * three-point seed, resolves the surface of the innermost hit and takes the
   * seed time from the corresponding entry of @p hitContainer.
   *
   * @param hits Radius-ordered seed hits.
   * @return The estimated parameters, or std::nullopt if there are no hits, the
   *         innermost hit's surface is unknown or the estimation itself fails.
   */
  template <class Alg>
  std::optional<Acts::BoundTrackParameters> estimateSeedParameters(
      const Alg& alg, const IActsGeoSvc& geo, const Acts::GeometryContext& geoCtx, const std::vector<SeedHit>& hits,
      const ACTSTracking::HitContainer& hitContainer, Acts::MagneticFieldProvider::Cache& magCache, double errPos,
      double errPhi, double errLambda, double errRelP, double errTime) {
    if (hits.empty()) {
      return std::nullopt;
    }

    const SeedHit& bottom = hits.front();
    const SeedHit& middle = hits[hits.size() / 2];
    const SeedHit& top    = hits.back();

    const Acts::Surface* bottomSurface = geo.trackingGeometry()->findSurface(bottom.sl.geometryId());
    if (bottomSurface == nullptr) {
      alg.warning() << "Surface with geoID " << bottom.sl.geometryId() << " not found in tracking geometry" << endmsg;
      return std::nullopt;
    }

    return estimateSeedParameters(alg, geo, geoCtx, *bottomSurface, bottom.pos, middle.pos, top.pos,
                                  hitContainer[bottom.sl.index()].getTime(), magCache, errPos, errPhi, errLambda,
                                  errRelP, errTime);
  }

}  // namespace ACTSTracking
