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

// edm4hep
#include <edm4hep/MCParticle.h>
#include <edm4hep/MutableTrack.h>
#include <edm4hep/Track.h>
#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackState.h>

// Gaudi
#include <GaudiKernel/IDataManagerSvc.h>
#include <GaudiKernel/IDataProviderSvc.h>
#include <GaudiKernel/IEvtSelector.h>
#include <GaudiKernel/MsgStream.h>
#include <GaudiKernel/ServiceHandle.h>
#include <GaudiKernel/SmartDataPtr.h>
#include <GaudiKernel/StatusCode.h>

// ACTS
#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/EventData/FreeTrackParameters.hpp>
#include <Acts/EventData/ParticleHypothesis.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>
#include <Acts/EventData/VectorTrackContainer.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Geometry/GeometryIdentifier.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Propagator/EigenStepper.hpp>
#include <Acts/Propagator/Navigator.hpp>
#include <Acts/Propagator/Propagator.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>
#include <Acts/TrackFitting/KalmanFitter.hpp>

// ACTSTracking
#include "k4ActsTracking/IActsGeoSvc.h"
#include "k4ActsTracking/SourceLink.hxx"

// Standard
#include <cstddef>
#include <optional>
#include <vector>

namespace ACTSTracking {

  /// Geometry-aware propagator used to extrapolate fitted tracks out to the
  /// calorimeter face. It navigates the tracking geometry (which now includes
  /// the passive calo volumes), so material along the way is accounted for and
  /// the actual curved trajectory selects which calo surface is hit.
  using CaloFacePropagator = Acts::Propagator<Acts::EigenStepper<>, Acts::Navigator>;

  using TrackResult =
      Acts::TrackContainer<Acts::VectorTrackContainer, Acts::VectorMultiTrajectory, std::shared_ptr>::TrackProxy;

  //! Get path to a resource file
  /**
 * Get absolute file of a file `inpath` by looking in the following places:
 *  - `inpath` to the current working directory
 *  - `ACTSTRACKING_SOURCEDIR/inpath`
 *  - `ACTSTRACKING_DATADIR/inpath`
 *
 * If the files is not found at any location, then `inpath` is returned.
 * If `path` starts with a /, then it is returned directly.
 *
 * \parm inpath File to find.
 *
 * \return Absolute path to file.
 */
  std::string findFile(const std::string& inpath);

  //! Convert ACTS KF result to edm4hep track class
  /**
 * Converted properties are:
 *  - goodness of fit (chi2, ndf)
 *  - associated hits
 *  - track states at IP
 *
 * \param gctx geometry context used to resolve surface positions
 * \param fitOutput KF fit result
 * \param hits edm4hep hits parallel to the measurement container; the hit of
 *        each track state is recovered via its source-link index()
 * \param magneticField magnetic field at different locations in the detector
 * \param magCache cache to help with magnetic field lookup
 *
 * \return Track with equivalent parameters of the ACTS track
 */
  edm4hep::MutableTrack ACTS2edm4hep_track(const Acts::GeometryContext& gctx, const TrackResult& fitter_res,
                                           const HitContainer&                                hits,
                                           std::shared_ptr<const Acts::MagneticFieldProvider> magneticField,
                                           Acts::MagneticFieldProvider::Cache&                magCache);

  //! Convert ACTS track state class to edm4hep class
  /**
 * The EDM4hep track state uses a perigee (D0, Z0, phi, omega, tanLambda)
 * parametrization defined relative to a reference point. If \p params are not
 * already expressed on a perigee surface, they are re-expressed at an ad-hoc
 * perigee surface created at their global position (transporting parameters and
 * covariance), and the state's referencePoint is set to that perigee. This
 * mirrors ActsPlugins EDM4hep::convertTrackParametersToEdm4hep.
 *
 * \param location Location where the track state is defined (ie: `AtIP`)
 * \param gctx geometry context used to resolve surface positions
 * \param params ACTS track state parameters
 * \params Bz magnetic field at location of track state [Tesla]
 *
 * \return Track state with equivalent parameters of the ACTS track
 */
  edm4hep::TrackState ACTS2edm4hep_trackState(int location, const Acts::GeometryContext& gctx,
                                              const Acts::BoundTrackParameters& params, double Bz);
  //! Helper Method for ACTS2edm4hep_trackState. Expects \p value / \p cov to
  //! already be in the perigee reference frame and does NOT set the
  //! referencePoint; callers holding generic on-surface parameters should use
  //! the BoundTrackParameters overload above, which re-expresses them at an
  //! ad-hoc perigee first.
  edm4hep::TrackState ACTS2edm4hep_trackState(int location, const Acts::BoundVector& value,
                                              const Acts::BoundMatrix& cov, double Bz);

  //! Get the track state at a given location, keyed by its `location` field.
  /**
 * edm4hep's `Track::getTrackStates(i)` accesses states by array **position**,
 * not by location, and the location enum values are non-zero (`AtIP == 1`), so
 * `getTrackStates(AtIP)` returns the wrong (index-1) state. This searches the
 * track's states for the first one whose `location` matches, letting callers
 * fetch e.g. the AtIP parameters by meaning rather than by array index.
 *
 * \param track    Track to inspect
 * \param location edm4hep::TrackState location (e.g. `AtIP`, `AtFirstHit`)
 *
 * \return The matching track state, or std::nullopt if the track has none at
 *         that location.
 */
  std::optional<edm4hep::TrackState> trackStateAt(const edm4hep::Track& track, int location);

  //! Get particle hypothesis in ACTS format
  /**
 * \param MCParticle
 *
 * \return Particle Hypothesis based on MCParticle PDG
 */
  Acts::ParticleHypothesis convertParticle(const edm4hep::MCParticle mcParticle);

  //! Outcome of a calorimeter-face extrapolation.
  enum class CaloExtrapolationStatus {
    Ok,                ///< reached a calo-face surface
    NoSurfaces,        ///< no calo-face surfaces are configured
    NotReached,        ///< propagation finished without reaching a calo-face surface
    PropagationError,  ///< the propagation itself failed
  };

  //! Result of a calorimeter-face extrapolation.
  struct CaloExtrapolationResult {
    std::optional<Acts::BoundTrackParameters> params{};
    CaloExtrapolationStatus                   status{CaloExtrapolationStatus::NotReached};
  };

  //! Extrapolate track parameters to the calorimeter face.
  /**
 * Propagates the given parameters through the tracking geometry (which contains
 * the passive calo volumes) and terminates as soon as the actual trajectory
 * reaches one of the calorimeter-face surfaces, identified by their geometry
 * ids. Because the propagation follows the real curved trajectory through the
 * geometry, no straight-line pre-selection of the target surface is needed and
 * material handling integrates naturally once the calo volumes carry material.
 *
 * \param propagator        Geometry-aware propagator
 * \param start             Track parameters to extrapolate from (e.g. at the last hit)
 * \param caloSurfaceGeoIds Geometry ids of the calo-face surfaces (from IActsGeoSvc)
 * \param gctx              Geometry context
 * \param mctx              Magnetic-field context
 * \param maxSteps          Maximum number of propagation steps
 *
 * \return Bound track parameters at the calorimeter face together with a status
 *         describing why the extrapolation succeeded or failed.
 */
  CaloExtrapolationResult extrapolateToCaloFace(const CaloFacePropagator&                    propagator,
                                                const Acts::BoundTrackParameters&            start,
                                                const std::vector<Acts::GeometryIdentifier>& caloSurfaceGeoIds,
                                                const Acts::GeometryContext& gctx, const Acts::MagneticFieldContext& mctx,
                                                std::size_t maxSteps);

}  // namespace ACTSTracking
