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
#ifndef K4ACTSTRACKING_GEANTINOMATERIALASSIGNER_H
#define K4ACTSTRACKING_GEANTINOMATERIALASSIGNER_H

#include <Acts/Definitions/Algebra.hpp>
#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/Material/PropagatorMaterialAssigner.hpp>
#include <Acts/Material/interface/IAssignmentFinder.hpp>
#include <Acts/Propagator/ActorList.hpp>
#include <Acts/Propagator/StandardAborters.hpp>
#include <Acts/Propagator/SurfaceCollector.hpp>
#include <Acts/Utilities/Logger.hpp>
#include <Acts/Utilities/VectorHelpers.hpp>

#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ACTSTracking {

  /// Assignment finder that walks the geometry with the navigator, like
  /// @c Acts::PropagatorMaterialAssigner, but with a configurable step limit and
  /// without turning a propagation failure into an exception.
  ///
  /// @c Acts::PropagatorMaterialAssigner turns any propagation failure into a
  /// fatal error: it calls @c .value() on the propagation result, which throws
  /// if the propagation did not terminate normally. A single bad direction in a
  /// million-geantino scan therefore kills the whole job with
  ///
  ///     Value called on error value: PropagatorError: Propagation reached the
  ///     configured maximum number of steps
  ///
  /// after having written every track up to that point, which looks like a
  /// crash rather than like one unusable track. The same applies to the
  /// @c maxTargetSkipping limit, which surfaces as @c NextTargetLimitReached.
  /// A scan is a sampling of directions and a few pathological ones should cost
  /// their own entries, not the run.
  ///
  /// Acts discards the actor results when a propagation fails
  /// (@c Propagator::makeResult returns the error before moving the collectors
  /// into the result), so there is nothing to salvage for such a track. It is
  /// therefore reported as carrying no material, and counted: the validation
  /// still writes an entry, which keeps the output aligned entry by entry with
  /// the scan it is compared against, and @c stats() tells the caller how many
  /// entries are hollow so a silent deficit cannot be mistaken for physics.
  ///
  /// Note that hitting the step limit means the propagation is *stuck*, not that
  /// it was given too small a budget. Measured on MAIA_v0, a geantino needs
  /// about 40 steps, while a track that fails still fails with the limit raised
  /// a hundredfold. Raising @c maxSteps is therefore not a fix, and the counters
  /// are the point: they say how much of the validation is missing.
  ///
  /// @tparam propagator_t the propagator type, e.g. ACTSTracking::GeantinoPropagator
  template <typename propagator_t> class GeantinoMaterialAssigner final : public Acts::IAssignmentFinder {
  public:
    /// Tally of what the propagations did, for reporting by the caller.
    struct Stats {
      std::size_t nTracks{0};           ///< propagations attempted
      std::size_t nFailed{0};           ///< propagations that returned an error
      std::size_t maxStepsObserved{0};  ///< largest step count of a successful propagation
      std::string lastError{};          ///< message of the most recent failure, for the caller to report
    };

    /// @param propagator the propagator to walk the geometry with
    /// @param maxSteps step limit for a single propagation
    /// @param logger Acts logger for the failure warnings
    GeantinoMaterialAssigner(propagator_t propagator, unsigned int maxSteps, std::unique_ptr<const Acts::Logger> logger)
        : m_propagator(std::move(propagator)),
          m_maxSteps(maxSteps),
          m_stats(std::make_shared<Stats>()),
          m_logger(std::move(logger)) {}

    /// Counters shared with whatever built this assigner. The pointer is shared
    /// rather than copied out so the caller can read the tally after the run
    /// even though @c assignmentCandidates is const.
    ///
    /// @returns the statistics of the propagations run so far
    std::shared_ptr<const Stats> stats() const { return m_stats; }

    /// Find the material a geantino along @p direction from @p position crosses.
    ///
    /// @param gctx the geometry context
    /// @param mctx the magnetic field context
    /// @param position start position of the ray
    /// @param direction direction of the ray
    ///
    /// @returns the surface and volume assignments, both empty if the
    ///          propagation failed
    std::pair<std::vector<Acts::IAssignmentFinder::SurfaceAssignment>,
              std::vector<Acts::IAssignmentFinder::VolumeAssignment>>
    assignmentCandidates(const Acts::GeometryContext& gctx, const Acts::MagneticFieldContext& mctx,
                         const Acts::Vector3& position, const Acts::Vector3& direction) const final {
      std::pair<std::vector<Acts::IAssignmentFinder::SurfaceAssignment>,
                std::vector<Acts::IAssignmentFinder::VolumeAssignment>>
          candidates;

      using Acts::VectorHelpers::makeVector4;
      const Acts::BoundTrackParameters start = Acts::BoundTrackParameters::createCurvilinear(
          makeVector4(position, 0), direction, 1, std::nullopt, Acts::ParticleHypothesis::geantino());

      // Same actors as Acts::PropagatorMaterialAssigner; the collectors and the
      // surface selector are reused from its header rather than duplicated.
      using MaterialSurfaceCollector = Acts::SurfaceCollector<Acts::MaterialSurfaceIdentifier>;
      using ActorList =
          Acts::ActorList<MaterialSurfaceCollector, Acts::InteractionVolumeCollector, Acts::EndOfWorldReached>;
      using PropagatorOptions = typename propagator_t::template Options<ActorList>;

      PropagatorOptions options(gctx, mctx);
      options.maxSteps = m_maxSteps;

      const auto result = m_propagator.propagate(start, options, false);

      ++m_stats->nTracks;
      if (!result.ok()) {
        ++m_stats->nFailed;
        // Reported by the caller, which can name the offending scan entry; here
        // we only carry the reason out.
        m_stats->lastError = result.error().message();
        ACTS_DEBUG("Propagation failed for direction " << direction.transpose() << ": " << m_stats->lastError);
        return candidates;
      }

      m_stats->maxStepsObserved = std::max(m_stats->maxStepsObserved, result.value().steps);

      const auto& surfaceResult = result.value().template get<MaterialSurfaceCollector::result_type>();
      for (const auto& hit : surfaceResult.collected) {
        candidates.first.push_back(Acts::IAssignmentFinder::SurfaceAssignment{hit.surface, hit.position, direction});
      }

      const auto& volumeResult = result.value().template get<Acts::InteractionVolumeCollector::result_type>();
      for (const auto& [geoId, assignment] : volumeResult.collected) {
        candidates.second.push_back(assignment);
      }

      return candidates;
    }

  private:
    const Acts::Logger& logger() const { return *m_logger; }

    propagator_t                        m_propagator;
    unsigned int                        m_maxSteps;
    std::shared_ptr<Stats>              m_stats;
    std::unique_ptr<const Acts::Logger> m_logger;
  };

}  // namespace ACTSTracking

#endif  // K4ACTSTRACKING_GEANTINOMATERIALASSIGNER_H
