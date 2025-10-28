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

#include "k4ActsTracking/Helpers.hxx"

// edm4hep
#include <edm4hep/MutableTrack.h>
#include <edm4hep/TrackerHit.h>

// Standard
#include <filesystem>
#include <memory>
#include <vector>

// ACTS
#include <Acts/MagneticField/InterpolatedBFieldMap.hpp>

// ACTSTracking
#include "config.h.in"

namespace ACTSTracking {

  std::string findFile(const std::string& inpath) {
    if (inpath.empty())
      return inpath;

    // Already absolute path
    if (inpath[0] == '/')
      return inpath;

    // relative to cwd
    if (std::filesystem::exists(inpath)) {
      return inpath;
    }

    // relative to absolute paths
    if (std::filesystem::exists(ACTSTRACKING_SOURCEDIR + inpath)) {
      return ACTSTRACKING_SOURCEDIR + inpath;
    }

    if (std::filesystem::exists(ACTSTRACKING_DATADIR + inpath)) {
      return ACTSTRACKING_DATADIR + inpath;
    }

    // nothing was found :( <- :(((
    return inpath;
  }

  edm4hep::MutableTrack* ACTS2edm4hep_track(const TrackResult&                           fitter_res,
                                            std::shared_ptr<Acts::MagneticFieldProvider> magneticField,
                                            Acts::MagneticFieldProvider::Cache&          magCache) {
    // Create new object
    edm4hep::MutableTrack* track = new edm4hep::MutableTrack();

    // Basic properties
    track->setChi2(fitter_res.chi2());
    track->setNdf(fitter_res.nDoF());
    track->setNholes(fitter_res.nHoles());

    // ACTS magnetic field
    const Acts::Vector3         zeroPos(0, 0, 0);
    Acts::Result<Acts::Vector3> fieldRes = magneticField->getField(zeroPos, magCache);
    if (!fieldRes.ok()) {
      throw std::runtime_error("Field lookup error: " + fieldRes.error().value());
    }
    Acts::Vector3 field = *fieldRes;

    // Covarienc and states
    const Acts::BoundVector& params         = fitter_res.parameters();
    const Acts::BoundMatrix& covariance     = fitter_res.covariance();
    edm4hep::TrackState*     trackStateAtIP = ACTSTracking::ACTS2edm4hep_trackState(
        edm4hep::TrackState::AtIP, params, covariance, field[2] / Acts::UnitConstants::T);
    track->addToTrackStates(*trackStateAtIP);

    std::vector<edm4hep::TrackerHit> hitsOnTrack;
    std::vector<edm4hep::TrackState> statesOnTrack;

    // Handle each track state
    for (const auto& trk_state : fitter_res.trackStatesReversed()) {
      if (!trk_state.hasUncalibratedSourceLink())
        continue;

      auto sl = trk_state.getUncalibratedSourceLink().get<ACTSTracking::SourceLink>();

      edm4hep::TrackerHit curr_hit(*sl.edm4hepTHitP());

      hitsOnTrack.push_back(curr_hit);

      const Acts::Vector3 hitPos(curr_hit.getPosition().x, curr_hit.getPosition().y, curr_hit.getPosition().z);

      Acts::Result<Acts::Vector3> fieldRes = magneticField->getField(hitPos, magCache);
      if (!fieldRes.ok()) {
        throw std::runtime_error("Field lookup error: " + fieldRes.error().value());
      }
      Acts::Vector3 field = *fieldRes;

      edm4hep::TrackState* trackState =
          ACTSTracking::ACTS2edm4hep_trackState(edm4hep::TrackState::AtOther, trk_state.smoothed(),
                                                trk_state.smoothedCovariance(), field[2] / Acts::UnitConstants::T);
      statesOnTrack.push_back(*trackState);
    }

    std::reverse(hitsOnTrack.begin(), hitsOnTrack.end());
    std::reverse(statesOnTrack.begin(), statesOnTrack.end());

    // Add Track Hits
    for (const auto& hit : hitsOnTrack) {
      track->addToTrackerHits(hit);
    }

    if (!statesOnTrack.empty()) {
      statesOnTrack.back().location  = edm4hep::TrackState::AtLastHit;
      statesOnTrack.front().location = edm4hep::TrackState::AtFirstHit;
    }

    // Add Track States
    for (const auto& state : statesOnTrack) {
      track->addToTrackStates(state);
    }

    return track;
  }

  edm4hep::TrackState* ACTS2edm4hep_trackState(int location, const Acts::BoundTrackParameters& params, double Bz) {
    return ACTS2edm4hep_trackState(location, params.parameters(), params.covariance().value(), Bz);
  }

  edm4hep::TrackState* ACTS2edm4hep_trackState(int location, const Acts::BoundVector& value,
                                               const Acts::BoundMatrix& cov, double Bz) {
    // Create new object
    edm4hep::TrackState* trackState = new edm4hep::TrackState();

    // Basic properties
    trackState->location = location;

    // Trajectory parameters
    // Central values
    double d0     = value[Acts::eBoundLoc0];
    double z0     = value[Acts::eBoundLoc1];
    double phi    = value[Acts::eBoundPhi];
    double theta  = value[Acts::eBoundTheta];
    double qoverp = value[Acts::eBoundQOverP];

    double p         = 1e3 / qoverp;
    double omega     = (0.3 * Bz) / (p * std::sin(theta));
    double lambda    = M_PI / 2 - theta;
    double tanlambda = std::tan(lambda);

    trackState->phi       = phi;
    trackState->tanLambda = tanlambda;
    trackState->omega     = omega;
    trackState->D0        = d0;
    trackState->Z0        = z0;

    // Uncertainties (covariance matrix)
    Acts::ActsMatrix<6, 6> jac = Acts::ActsMatrix<6, 6>::Zero();

    jac(0, Acts::eBoundLoc0) = 1;

    jac(1, Acts::eBoundPhi) = 1;

    jac(2, Acts::eBoundTheta)  = omega / std::tan(theta);
    jac(2, Acts::eBoundQOverP) = omega / qoverp;

    jac(3, Acts::eBoundLoc1) = 1;

    jac(4, Acts::eBoundTheta) = std::pow(1 / std::cos(lambda), 2);

    Acts::ActsMatrix<6, 6> trcov = (jac * cov * jac.transpose());

    int count = 0;
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < i; ++j) {
        trackState->covMatrix[count] = trcov(j, i);
        count++;
      }
    }

    return trackState;
  }

  Acts::ParticleHypothesis convertParticle(const edm4hep::MCParticle mcParticle) {
    switch (mcParticle.getPDG()) {
      case 11:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eElectron};
      case -11:
        return Acts::ParticleHypothesis{Acts::PdgParticle::ePositron};
      case 13:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eMuon};
      case -13:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eAntiMuon};
      case 15:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eTau};
      case -15:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eAntiTau};
      case 22:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eGamma};
      case 111:
        return Acts::ParticleHypothesis{Acts::PdgParticle::ePionZero};
      case 211:
        return Acts::ParticleHypothesis{Acts::PdgParticle::ePionPlus};
      case -211:
        return Acts::ParticleHypothesis{Acts::PdgParticle::ePionMinus};
      case 2112:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eNeutron};
      case -2112:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eAntiNeutron};
      case 2212:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eProton};
      case -2212:
        return Acts::ParticleHypothesis{Acts::PdgParticle::eAntiProton};
    }

    Acts::PdgParticle pdg  = Acts::PdgParticle::eInvalid;
    float             mass = 0.0f;
    Acts::AnyCharge   charge_type{0.0f};
    return Acts::ParticleHypothesis{pdg, mass, charge_type};
  }

}  // namespace ACTSTracking
