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
#include "MaterialScanRegion.h"

#include "k4ActsTracking/IActsGeoSvc.h"

#include <k4FWCore/GaudiChecks.h>
#include <k4Interface/IGeoSvc.h>

#include <Gaudi/Algorithm.h>
#include <Gaudi/Property.h>
#include <GaudiKernel/StatusCode.h>

#include <Acts/Definitions/Algebra.hpp>
#include <Acts/Definitions/Units.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Material/MaterialInteraction.hpp>
#include <Acts/Utilities/UnitVectors.hpp>
#include <ActsPlugins/Root/RootMaterialTrackIo.hpp>
#include <ActsPlugins/Root/TGeoMaterialConverter.hpp>

#include <DD4hep/DD4hepUnits.h>
#include <DD4hep/Detector.h>

#include <TChain.h>
#include <TFile.h>
#include <TGeoManager.h>
#include <TGeoMaterial.h>
#include <TGeoMedium.h>
#include <TGeoNavigator.h>
#include <TGeoVolume.h>
#include <TTree.h>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

/// Records the material along straight (geantino) rays through the DD4hep
/// geometry, walked with the TGeo navigator, for @c MaterialMappingAlg.
///
/// Each volume crossing becomes one @c Acts::MaterialInteraction at the point
/// where the ray enters the volume, as in ACTS' Geant4 material recording. Rays
/// end at the boundary of @c MaterialScanRegion (by default the ECAL inner face).
///
/// @c dd4hep::rec::MaterialManager::materialsBetween is not used: after a step
/// shorter than its MINSTEP it loses the next volume crossing.
///
/// All rays are recorded on the first event, so run it with EvtMax=1.
struct MaterialRecordingAlg final : public Gaudi::Algorithm {
  using Gaudi::Algorithm::Algorithm;

  StatusCode initialize() override;
  StatusCode execute(const EventContext&) const override;
  StatusCode finalize() override;

  Gaudi::Property<std::string> m_outputFile{this, "OutputFile", "material-tracks.root",
                                            "ROOT file to write the recorded material tracks to."};
  Gaudi::Property<std::string> m_treeName{this, "TreeName", "material_tracks",
                                          "Name of the TTree to write. MaterialMappingAlg reads 'material_tracks'."};
  Gaudi::Property<std::int64_t> m_numTracks{
      this, "NumTracks", 1000,
      "Number of rays to record. With ReplayFiles, a negative value replays every entry of the input."};
  Gaudi::Property<std::uint64_t> m_seed{this, "Seed", 42,
                                        "Seed of the direction sampling. Give every job of a split scan its own seed."};
  Gaudi::Property<std::vector<double>> m_etaRange{this, "EtaRange", {-4., 4.}, "Pseudorapidity range, sampled flat."};
  Gaudi::Property<std::vector<double>> m_phiRange{
      this, "PhiRange", {-std::numbers::pi, std::numbers::pi}, "Azimuth range in rad, sampled flat."};
  Gaudi::Property<std::vector<std::string>> m_replayFiles{
      this,
      "ReplayFiles",
      {},
      "Scan file(s), e.g. a Geant4 scan, whose start points and directions are replayed entry by entry instead of "
      "sampling new ones. The output then lines up track by track with the input."};
  Gaudi::Property<double> m_maxR{this, "MaxR", -1.,
                                 "Radius in mm at which rays stop. Non-positive (default): the ECAL barrel inner face, "
                                 "or the tracking world if the detector has none. Keep it the same as in the mapping."};
  Gaudi::Property<double> m_maxZ{this, "MaxZ", -1.,
                                 "|z| in mm at which rays stop. Non-positive (default): the ECAL endcap inner face, "
                                 "or the tracking world if the detector has none. Keep it the same as in the mapping."};
  Gaudi::Property<std::vector<std::string>> m_excludeMaterials{
      this,
      "ExcludeMaterials",
      {"Air", "Vacuum"},
      "Materials not recorded, as in ACTS' Geant4 material recording (Geant4MaterialRecording.excludeMaterials)."};

private:
  /// Start point and direction of the next ray, sampled or replayed.
  std::pair<Acts::Vector3, Acts::Vector3> nextRay() const;
  /// Walk @p length from @p start along @p direction through the geometry and
  /// append one interaction per volume crossed to @p track.
  void walk(const Acts::Vector3& start, const Acts::Vector3& direction, double length,
            Acts::RecordedMaterialTrack& track) const;

  SmartIF<IGeoSvc> m_geoSvc;
  TGeoManager* m_tgeoManager{nullptr};

  /// Converts DD4hep (TGeo) lengths to ACTS lengths.
  double m_lengthScale = Acts::UnitConstants::cm / dd4hep::cm;
  MaterialScanRegion::Cylinder m_region;

  std::unique_ptr<TChain> m_replayChain{nullptr};
  std::unique_ptr<ActsPlugins::RootMaterialTrackIo> m_replayReader{nullptr};

  // Mutated from the const execute(); this is a one-shot job.
  mutable std::mt19937_64 m_rng;
  mutable std::int64_t m_replayEntry{0};
  mutable std::unique_ptr<TFile> m_outFile{nullptr};
  mutable TTree* m_outTree{nullptr};
  mutable std::unique_ptr<ActsPlugins::RootMaterialTrackIo> m_writer{nullptr};
  mutable std::size_t m_nTracks{0};
  mutable std::size_t m_nSteps{0};
  mutable std::size_t m_nExcludedSteps{0};
  mutable std::size_t m_nPushes{0};
  mutable std::size_t m_nFailed{0};
  mutable bool m_done{false};

  Acts::GeometryContext m_gctx = Acts::GeometryContext::dangerouslyDefaultConstruct();
};

DECLARE_COMPONENT(MaterialRecordingAlg)

/// How many individual ray failures are reported before falling back to the
/// count in finalize().
static constexpr std::size_t kMaxReportedFailures = 5;

/// Shortest step taken from TGeo, in cm. On a boundary TGeo can return (near)
/// zero steps; the ray is then pushed on by this much and relocated.
static constexpr double kMinStep = 1e-5;

/// Steps after which a ray counts as stuck.
static constexpr std::size_t kMaxSteps = 100000;

StatusCode MaterialRecordingAlg::initialize() {
  if (auto sc = Gaudi::Algorithm::initialize(); sc.isFailure()) {
    return sc;
  }

  m_geoSvc = svcLoc()->service<IGeoSvc>("GeoSvc");
  K4_GAUDI_CHECK(m_geoSvc);

  // The material conversion below reads TGeo densities as g/cm3, which holds
  // for DD4hep's default (ROOT) unit system but not for a DD4hep built with
  // DD4HEP_USE_GEANT4_UNITS.
  if (TGeoManager::GetDefaultUnits() != TGeoManager::kRootUnits) {
    error() << "TGeo does not use ROOT units (cm, g/cm3); the material conversion assumes them." << endmsg;
    return StatusCode::FAILURE;
  }

  m_tgeoManager = &m_geoSvc->getDetector()->manager();

  if (m_etaRange.value().size() != 2 || m_phiRange.value().size() != 2) {
    error() << "EtaRange and PhiRange take exactly two values (min max)." << endmsg;
    return StatusCode::FAILURE;
  }

  if (m_maxR.value() > 0. && m_maxZ.value() > 0.) {
    m_region = {m_maxR.value(), m_maxZ.value()};
  } else {
    auto actsGeoSvc = svcLoc()->service<IActsGeoSvc>("ActsGeoSvc");
    K4_GAUDI_CHECK(actsGeoSvc);
    const auto region = MaterialScanRegion::fromGeometry(*actsGeoSvc, m_maxR.value(), m_maxZ.value());
    if (!region) {
      error() << "No calorimeter face and no cylindrical tracking world to stop the rays at; set MaxR and MaxZ."
              << endmsg;
      return StatusCode::FAILURE;
    }
    m_region = *region;
  }
  info() << fmt::format("Rays stop at r = {:.1f} mm, |z| = {:.1f} mm.", m_region.r, m_region.halfZ) << endmsg;

  if (!m_replayFiles.value().empty()) {
    m_replayChain = std::make_unique<TChain>(m_treeName.value().c_str());
    for (const auto& file : m_replayFiles.value()) {
      if (m_replayChain->Add(file.c_str()) == 0) {
        error() << fmt::format("Could not add '{}' to the replay chain.", file) << endmsg;
        return StatusCode::FAILURE;
      }
    }
    if (m_replayChain->GetEntries() == 0) {
      error() << fmt::format("The replay chain holds no entries. Is '{}' the right tree name?", m_treeName.value())
              << endmsg;
      return StatusCode::FAILURE;
    }
    ActsPlugins::RootMaterialTrackIo::Config ioCfg;
    ioCfg.prePostStepInfo = true;
    m_replayReader = std::make_unique<ActsPlugins::RootMaterialTrackIo>(ioCfg);
    m_replayReader->connectForRead(*m_replayChain);
    info() << fmt::format("Replaying the directions of {} scan entries.", m_replayChain->GetEntries()) << endmsg;
  } else if (m_numTracks.value() <= 0) {
    error() << "NumTracks must be positive when directions are sampled." << endmsg;
    return StatusCode::FAILURE;
  }

  m_rng.seed(m_seed.value());

  return StatusCode::SUCCESS;
}

std::pair<Acts::Vector3, Acts::Vector3> MaterialRecordingAlg::nextRay() const {
  if (m_replayReader != nullptr) {
    m_replayChain->GetEntry(m_replayEntry++);
    const auto track = m_replayReader->read();
    return {track.first.first, track.first.second.normalized()};
  }
  std::uniform_real_distribution<double> eta(m_etaRange.value()[0], m_etaRange.value()[1]);
  std::uniform_real_distribution<double> phi(m_phiRange.value()[0], m_phiRange.value()[1]);
  // Draw phi first so the sequence does not depend on the order of evaluation.
  const double p = phi(m_rng);
  return {Acts::Vector3::Zero(), Acts::makeDirectionFromPhiEta(p, eta(m_rng))};
}

void MaterialRecordingAlg::walk(const Acts::Vector3& start, const Acts::Vector3& direction, double length,
                                Acts::RecordedMaterialTrack& track) const {
  const ActsPlugins::TGeoMaterialConverter::Options convOptions{.unitLengthScalor = m_lengthScale,
                                                                .unitMassScalor = Acts::UnitConstants::g};
  const auto& excluded = m_excludeMaterials.value();

  // TGeo works in DD4hep length units.
  const Acts::Vector3 origin = start / m_lengthScale;
  const double total = length / m_lengthScale;

  TGeoNavigator& nav = *m_tgeoManager->GetCurrentNavigator();
  nav.InitTrack(origin.data(), direction.data());

  double travelled = 0.;
  for (std::size_t n = 0; travelled < total && !nav.IsOutside(); ++n) {
    if (n == kMaxSteps) {
      throw std::runtime_error(
          fmt::format("stuck after {} steps at {:.3f} mm along the ray", kMaxSteps, travelled * m_lengthScale));
    }
    // The current volume is the one crossed; the step moves into the next.
    const TGeoMedium* medium = nav.GetCurrentVolume()->GetMedium();
    nav.FindNextBoundaryAndStep(total - travelled);
    const double step = nav.GetStep();
    if (step < kMinStep) {
      travelled += kMinStep;
      const Acts::Vector3 point = origin + travelled * direction;
      nav.SetCurrentPoint(point.data());
      nav.FindNode();
      m_nPushes++;
      continue;
    }
    const double s0 = travelled;
    travelled += step;

    const TGeoMaterial* tgMaterial = medium != nullptr ? medium->GetMaterial() : nullptr;
    if (tgMaterial == nullptr) {
      continue;
    }
    if (std::ranges::find(excluded, std::string{tgMaterial->GetName()}) != excluded.end()) {
      m_nExcludedSteps++;
      continue;
    }

    const double thickness = step * m_lengthScale;
    Acts::MaterialInteraction interaction;
    interaction.position = start + s0 * m_lengthScale * direction;
    interaction.direction = direction;
    interaction.materialSlab =
        ActsPlugins::TGeoMaterialConverter::materialSlab(*tgMaterial, thickness, thickness, convOptions);
    // As in the Geant4 recording.
    interaction.pathCorrection = thickness;
    track.second.materialInX0 += interaction.materialSlab.thicknessInX0();
    track.second.materialInL0 += interaction.materialSlab.thicknessInL0();
    track.second.materialInteractions.push_back(std::move(interaction));
  }
}

StatusCode MaterialRecordingAlg::execute(const EventContext&) const {
  if (m_done) {
    return StatusCode::SUCCESS;
  }
  m_done = true;

  m_outFile.reset(TFile::Open(m_outputFile.value().c_str(), "RECREATE"));
  if (m_outFile == nullptr || m_outFile->IsZombie()) {
    error() << fmt::format("Could not open '{}' for writing.", m_outputFile.value()) << endmsg;
    return StatusCode::FAILURE;
  }
  m_outTree = new TTree(m_treeName.value().c_str(), "recorded material tracks");

  // Same writer settings as ACTS' material_recording.py, which MaterialMappingAlg
  // expects.
  ActsPlugins::RootMaterialTrackIo::Config ioCfg;
  ioCfg.prePostStepInfo = true;
  ioCfg.recalculateTotals = true;
  m_writer = std::make_unique<ActsPlugins::RootMaterialTrackIo>(ioCfg);
  m_writer->connectForWrite(*m_outTree);

  std::int64_t nToRecord = m_numTracks.value();
  if (m_replayChain != nullptr) {
    const std::int64_t nEntries = m_replayChain->GetEntries();
    nToRecord = nToRecord < 0 ? nEntries : std::min(nToRecord, nEntries);
  }

  // TGeo keeps one navigator per thread and only makes the first one itself.
  if (m_tgeoManager->GetCurrentNavigator() == nullptr) {
    m_tgeoManager->AddNavigator();
  }
  // Leave the navigator as we found it for whoever uses it next.
  m_tgeoManager->DoBackupState();

  for (std::int64_t i = 0; i < nToRecord; ++i) {
    const auto [start, direction] = nextRay();

    Acts::RecordedMaterialTrack track;
    track.first = {start, direction};

    // Path length (in ACTS units) from the start point to the stop cylinder.
    const double length = m_region.pathToBoundary(start, direction);
    try {
      if (length <= 0.) {
        throw std::runtime_error("the start point is outside the stop cylinder");
      }
      walk(start, direction, length, track);
    } catch (const std::exception& e) {
      // Keep the entry, empty, so the output stays aligned with a replayed
      // input.
      track.second = {};
      m_nFailed++;
      if (m_nFailed <= kMaxReportedFailures) {
        warning() << fmt::format("Ray {} (direction {:.6f} {:.6f} {:.6f}) could not be recorded: {}. It is written "
                                 "with no material.",
                                 i, direction.x(), direction.y(), direction.z(), e.what())
                  << endmsg;
      } else if (m_nFailed == kMaxReportedFailures + 1) {
        warning() << "Further failures will only be counted; see the summary at the end." << endmsg;
      }
    }

    m_nSteps += track.second.materialInteractions.size();
    m_writer->write(m_gctx, static_cast<std::uint32_t>(i), track);
    m_outTree->Fill();
    m_nTracks++;

    if (m_nTracks % 10000 == 0) {
      info() << fmt::format("Recorded {} of {} rays.", m_nTracks, nToRecord) << endmsg;
    }
  }

  m_tgeoManager->DoRestoreState();

  info() << fmt::format("Recorded {} rays with {} material steps ({} steps in excluded materials skipped, {} pushes "
                        "past a boundary).",
                        m_nTracks, m_nSteps, m_nExcludedSteps, m_nPushes)
         << endmsg;
  return StatusCode::SUCCESS;
}

StatusCode MaterialRecordingAlg::finalize() {
  if (m_outFile != nullptr) {
    m_outFile->cd();
    if (m_outTree != nullptr) {
      m_outTree->Write();
    }
    m_outFile->Close();
    m_outFile.reset();
  }

  if (m_nTracks == 0) {
    error() << "No rays were recorded. Was the algorithm executed (EvtMax>0, EvtSel=\"NONE\")?" << endmsg;
    return StatusCode::FAILURE;
  }

  info() << fmt::format("Wrote {} material tracks to '{}'.", m_nTracks, m_outputFile.value()) << endmsg;
  if (m_nFailed > 0) {
    warning() << fmt::format("{} of {} rays failed and were written with no material.", m_nFailed, m_nTracks) << endmsg;
  }

  return Gaudi::Algorithm::finalize();
}
