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

#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackState.h>

#include <Gaudi/Property.h>
#include <k4FWCore/Transformer.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

/// Drops tracks of a later tracking pass that are pieces of a track already kept: same charge and
/// compatible phi, tanLambda, omega and z0 at the IP (chi2 of the differences with the summed
/// variances below MaxChi2). Such split tracks share no hits with the kept track when the later
/// pass runs on the unused hits, so the hit-based duplicate removal cannot see them.
struct SplitTrackRemovalAlg final
    : k4FWCore::Transformer<edm4hep::TrackCollection(const edm4hep::TrackCollection&,
                                                     const std::vector<const edm4hep::TrackCollection*>&)> {
  SplitTrackRemovalAlg(const std::string& name, ISvcLocator* svcLoc)
      : Transformer(name, svcLoc,
                    {KeyValues("InputTrackCollection", {"Tracks"}), KeyValues("KeptTrackCollections", {"KeptTracks"})},
                    {KeyValues("OutputTrackCollection", {"TracksNoSplits"})}) {}

  edm4hep::TrackCollection
  operator()(const edm4hep::TrackCollection& tracks,
             const std::vector<const edm4hep::TrackCollection*>& keptCollections) const override {
    std::vector<edm4hep::TrackState> kept;
    for (const auto* coll : keptCollections) {
      for (const auto& trk : *coll) {
        if (auto st = trk.getTrackState(edm4hep::TrackState::AtIP)) {
          kept.push_back(*st);
        }
      }
    }
    edm4hep::TrackCollection out;
    out.setSubsetCollection();
    for (const auto& trk : tracks) {
      const auto st = trk.getTrackState(edm4hep::TrackState::AtIP);
      bool split = false;
      if (st) {
        for (const auto& k : kept) {
          if (chi2(*st, k) < m_maxChi2) {
            split = true;
            break;
          }
        }
      }
      if (!split) {
        out.push_back(trk);
      }
    }
    return out;
  }

private:
  static double chi2(const edm4hep::TrackState& a, const edm4hep::TrackState& b) {
    using P = edm4hep::TrackParams;
    if (a.omega * b.omega <= 0) {
      return std::numeric_limits<double>::max();
    }
    double dphi = a.phi - b.phi;
    dphi = std::remainder(dphi, 2 * std::numbers::pi);
    const auto term = [&](double d, P p) {
      const double var = a.getCovMatrix(p, p) + b.getCovMatrix(p, p);
      return var > 0 ? d * d / var : std::numeric_limits<double>::max();
    };
    return term(dphi, P::phi) + term(a.tanLambda - b.tanLambda, P::tanLambda) + term(a.omega - b.omega, P::omega) +
           term(a.Z0 - b.Z0, P::z0);
  }

  Gaudi::Property<double> m_maxChi2{this, "MaxChi2", 50., "Maximum chi2 (4 parameters) to call a track a split piece"};
};

DECLARE_COMPONENT(SplitTrackRemovalAlg)
