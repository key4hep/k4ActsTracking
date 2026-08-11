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
#include "GeantinoMaterialAssigner.h"
#include "MaterialSurfaces.h"

#include "k4ActsTracking/ActsGaudiLogger.h"
#include "k4ActsTracking/IActsGeoSvc.h"
#include "k4ActsTracking/RunnerCommon.hxx"

#include <k4FWCore/GaudiChecks.h>

#include <Gaudi/Algorithm.h>
#include <Gaudi/Property.h>
#include <GaudiKernel/StatusCode.h>

#include <Acts/Definitions/Algebra.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/Material/IntersectionMaterialAssigner.hpp>
#include <Acts/Material/MaterialInteraction.hpp>
#include <Acts/Material/MaterialValidator.hpp>
#include <ActsPlugins/Root/RootMaterialTrackIo.hpp>

#include <TChain.h>
#include <TFile.h>
#include <TTree.h>

#include <fmt/format.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/// Re-measures the material of the tracking geometry by propagating geantinos
/// through it, so the result can be compared against the Geant4 scan the map was
/// built from.
///
/// This is step 5 of doc/material_mapping.md, the physics validation. Run it with
/// @c ActsGeoSvc.MaterialMapFile pointing at the map under test: the geometry
/// then carries the mapped material, and what this algorithm records is what
/// reconstruction will actually see.
///
/// Directions are not sampled independently. They are taken from the same scan
/// file the map was produced from, so every propagated track has a Geant4
/// counterpart at the same entry index and the comparison is track by track,
/// with no binning or sampling differences to confound it.
///
/// The output is written in the same format as the scan (a `material_tracks`
/// tree via @c ActsPlugins::RootMaterialTrackIo), so both files can be read by
/// the same tooling -- see examples/compare_material_tracks.py.
///
/// The assigner is the propagator-based one rather than the intersection-based
/// one used for mapping: it reports the material the *navigator* finds, which is
/// what tracking sees, and can differ from a pure geometric intersection if
/// navigation misses a surface.
struct MaterialValidationAlg final : public Gaudi::Algorithm {
  using Gaudi::Algorithm::Algorithm;

  StatusCode initialize() override;
  StatusCode execute(const EventContext&) const override;
  StatusCode finalize() override;

  Gaudi::Property<std::vector<std::string>> m_inputFiles{
      this, "InputFiles", {}, "Geantino scan ROOT file(s). Only the track start positions and directions are used."};
  Gaudi::Property<std::string> m_treeName{this, "TreeName", "material_tracks", "Name of the TTree in the scan files."};
  Gaudi::Property<std::string> m_outputFile{
      this, "OutputFile", "propagated-material-tracks.root",
      "ROOT file to write the propagated material tracks to, in the same format as the scan."};
  Gaudi::Property<std::int64_t> m_maxTracks{
      this, "MaxTracks", -1, "Stop after this many tracks. Negative (default) processes the whole input."};
  Gaudi::Property<std::string> m_assigner{
      this, "Assigner", "intersection",
      "How to find the material a geantino crosses. 'intersection' intersects the designated surfaces "
      "geometrically, which is what the mapping step uses and therefore validates the map's content. "
      "'propagator' walks the geometry with the navigator, which additionally validates that tracking can "
      "reach that material."};
  Gaudi::Property<bool> m_resolvePassive{
      this, "ResolvePassive", true,
      "Let the navigator stop on passive surfaces. The material receivers are volume portals, so this must stay true "
      "for the propagation to see them."};
  Gaudi::Property<unsigned int> m_maxPropagationSteps{
      this, "MaxPropagationSteps", 1000,
      "Step limit for one geantino propagation, for the 'propagator' assigner. This is a safety valve against a "
      "propagation that never terminates, not a budget to be tuned: on MAIA_v0 a geantino needs about 40 steps, so "
      "the default is already an ample margin. A track that exceeds it is stuck rather than slow -- raising the "
      "limit does not rescue it, it only takes longer to give up."};

private:
  SmartIF<IActsGeoSvc> m_actsGeoSvc;

  std::unique_ptr<const Acts::Logger>               m_actsLogger{nullptr};
  std::unique_ptr<TChain>                           m_chain{nullptr};
  std::unique_ptr<ActsPlugins::RootMaterialTrackIo> m_reader{nullptr};

  using Assigner = ACTSTracking::GeantinoMaterialAssigner<ACTSTracking::GeantinoPropagator>;
  std::unique_ptr<Acts::MaterialValidator> m_validator{nullptr};
  /// Non-null only for the 'propagator' assigner; read in finalize() to report
  /// how many propagations failed.
  std::shared_ptr<const Assigner::Stats> m_assignerStats{nullptr};

  // Mutated from the const execute(); this is a one-shot job.
  mutable std::unique_ptr<TFile>                            m_outFile{nullptr};
  mutable TTree*                                            m_outTree{nullptr};
  mutable std::unique_ptr<ActsPlugins::RootMaterialTrackIo> m_writer{nullptr};
  mutable std::size_t                                       m_nProcessed{0};
  mutable bool                                              m_done{false};

  Acts::GeometryContext      m_gctx = Acts::GeometryContext::dangerouslyDefaultConstruct();
  Acts::MagneticFieldContext m_mctx{};
};

DECLARE_COMPONENT(MaterialValidationAlg)

/// How many individual propagation failures are reported before falling back to
/// the run-level count in finalize().
static constexpr std::size_t kMaxReportedFailures = 5;

StatusCode MaterialValidationAlg::initialize() {
  if (auto sc = Gaudi::Algorithm::initialize(); sc.isFailure()) {
    return sc;
  }

  if (m_inputFiles.value().empty()) {
    error() << "No input files given. Set InputFiles to the geantino scan the map was built from." << endmsg;
    return StatusCode::FAILURE;
  }

  m_actsGeoSvc = svcLoc()->service<IActsGeoSvc>("ActsGeoSvc");
  K4_GAUDI_CHECK(m_actsGeoSvc);

  m_actsLogger = makeActsGaudiLogger(this);

  Acts::MaterialValidator::Config cfg;
  if (m_assigner.value() == "propagator") {
    auto assigner =
        std::make_shared<const Assigner>(ACTSTracking::makeGeantinoPropagator(*m_actsGeoSvc, m_resolvePassive.value()),
                                         m_maxPropagationSteps.value(), m_actsLogger->cloneWithSuffix("|Assigner"));
    m_assignerStats      = assigner->stats();
    cfg.materialAssigner = std::move(assigner);
  } else if (m_assigner.value() == "intersection") {
    const auto surfaces = MaterialSurfaces::collectMaterialSurfaces(*m_actsGeoSvc->trackingGeometry());
    if (surfaces.empty()) {
      error() << "The tracking geometry carries no material at all. Did you forget "
                 "ActsGeoSvc.MaterialMapFile? Without a map there is nothing to validate."
              << endmsg;
      return StatusCode::FAILURE;
    }
    info() << fmt::format("Validating against {} material surfaces.", surfaces.size()) << endmsg;
    Acts::IntersectionMaterialAssigner::Config assignerCfg;
    assignerCfg.surfaces = surfaces;
    cfg.materialAssigner = std::make_shared<const Acts::IntersectionMaterialAssigner>(
        assignerCfg, m_actsLogger->cloneWithSuffix("|Assigner"));
  } else {
    error() << fmt::format("Unknown Assigner '{}'; use 'intersection' or 'propagator'.", m_assigner.value()) << endmsg;
    return StatusCode::FAILURE;
  }
  m_validator = std::make_unique<Acts::MaterialValidator>(cfg, m_actsLogger->cloneWithSuffix("|Validator"));

  m_chain = std::make_unique<TChain>(m_treeName.value().c_str());
  for (const auto& file : m_inputFiles.value()) {
    if (m_chain->Add(file.c_str()) == 0) {
      error() << fmt::format("Could not add '{}' to the input chain.", file) << endmsg;
      return StatusCode::FAILURE;
    }
  }
  if (m_chain->GetEntries() == 0) {
    error() << fmt::format("The input chain holds no entries. Is '{}' the right tree name?", m_treeName.value())
            << endmsg;
    return StatusCode::FAILURE;
  }

  ActsPlugins::RootMaterialTrackIo::Config ioCfg;
  ioCfg.prePostStepInfo = true;
  m_reader              = std::make_unique<ActsPlugins::RootMaterialTrackIo>(ioCfg);
  m_reader->connectForRead(*m_chain);

  info() << fmt::format("Re-propagating {} scan directions through the tracking geometry.", m_chain->GetEntries())
         << endmsg;

  return StatusCode::SUCCESS;
}

StatusCode MaterialValidationAlg::execute(const EventContext&) const {
  if (m_done) {
    return StatusCode::SUCCESS;
  }
  m_done = true;

  m_outFile.reset(TFile::Open(m_outputFile.value().c_str(), "RECREATE"));
  if (m_outFile == nullptr || m_outFile->IsZombie()) {
    error() << fmt::format("Could not open '{}' for writing.", m_outputFile.value()) << endmsg;
    return StatusCode::FAILURE;
  }
  m_outTree = new TTree(m_treeName.value().c_str(), "propagated material tracks");

  ActsPlugins::RootMaterialTrackIo::Config ioCfg;
  ioCfg.prePostStepInfo = true;
  m_writer              = std::make_unique<ActsPlugins::RootMaterialTrackIo>(ioCfg);
  m_writer->connectForWrite(*m_outTree);

  const std::int64_t nEntries = m_chain->GetEntries();
  const std::int64_t nToRead =
      m_maxTracks.value() < 0 ? nEntries : std::min<std::int64_t>(m_maxTracks.value(), nEntries);

  for (std::int64_t entry = 0; entry < nToRead; ++entry) {
    if (m_chain->GetEntry(entry) <= 0) {
      warning() << fmt::format("Could not read entry {}, stopping here.", entry) << endmsg;
      break;
    }
    const auto scanTrack = m_reader->read();

    const Acts::Vector3& position  = scanTrack.first.first;
    const Acts::Vector3  direction = scanTrack.first.second.normalized();

    // Same entry index as the scan, so the two files line up track by track.
    const std::size_t failedBefore = m_assignerStats != nullptr ? m_assignerStats->nFailed : 0;

    const auto propagated = m_validator->recordMaterial(m_gctx, m_mctx, position, direction);

    // Name the offending scan entry: the assigner knows the reason but not
    // which entry it came from, and the entry index is what makes the failure
    // reproducible. Loud for the first few, then only the finalize() summary --
    // a scan is millions of tracks.
    if (m_assignerStats != nullptr && m_assignerStats->nFailed > failedBefore) {
      if (m_assignerStats->nFailed <= kMaxReportedFailures) {
        warning() << fmt::format(
                         "Entry {} (direction {:.6f} {:.6f} {:.6f}) could not be propagated: {}. It is written with "
                         "no material to keep the output aligned with the scan, so exclude it from the comparison.",
                         entry, direction.x(), direction.y(), direction.z(), m_assignerStats->lastError)
                  << endmsg;
      } else if (m_assignerStats->nFailed == kMaxReportedFailures + 1) {
        warning() << "Further propagation failures will only be counted; see the summary at the end." << endmsg;
      }
    }

    m_writer->write(m_gctx, static_cast<std::uint32_t>(entry), propagated);
    m_outTree->Fill();
    m_nProcessed++;

    if (m_nProcessed % 10000 == 0) {
      info() << fmt::format("Propagated {} of {} tracks.", m_nProcessed, nToRead) << endmsg;
    }
  }

  info() << fmt::format("Propagated {} tracks.", m_nProcessed) << endmsg;
  return StatusCode::SUCCESS;
}

StatusCode MaterialValidationAlg::finalize() {
  if (m_outFile != nullptr) {
    m_outFile->cd();
    if (m_outTree != nullptr) {
      m_outTree->Write();
    }
    m_outFile->Close();
    m_outFile.reset();
  }

  if (m_nProcessed == 0) {
    error() << "No tracks were propagated. Was the algorithm executed (EvtMax>0, EvtSel=\"NONE\")?" << endmsg;
    return StatusCode::FAILURE;
  }

  info() << fmt::format("Wrote {} propagated material tracks to '{}'.", m_nProcessed, m_outputFile.value()) << endmsg;

  if (m_assignerStats != nullptr) {
    info() << fmt::format("Deepest propagation took {} of the {} allowed steps.", m_assignerStats->maxStepsObserved,
                          m_maxPropagationSteps.value())
           << endmsg;
    if (m_assignerStats->nFailed > 0) {
      warning() << fmt::format(
                       "{} of {} propagations failed and were written with no material, which biases the comparison "
                       "low for those entries. Raise MaxPropagationSteps (currently {}) and re-run.",
                       m_assignerStats->nFailed, m_assignerStats->nTracks, m_maxPropagationSteps.value())
                << endmsg;
    }
  }

  return Gaudi::Algorithm::finalize();
}
