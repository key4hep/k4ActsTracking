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
#include "MaterialSurfaces.h"

#include "k4ActsTracking/ActsGaudiLogger.h"
#include "k4ActsTracking/IActsGeoSvc.h"

#include <k4FWCore/GaudiChecks.h>

#include <Gaudi/Algorithm.h>
#include <Gaudi/Property.h>
#include <GaudiKernel/StatusCode.h>

#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/Material/BinnedSurfaceMaterialAccumulator.hpp>
#include <Acts/Material/IntersectionMaterialAssigner.hpp>
#include <Acts/Material/MaterialInteraction.hpp>
#include <Acts/Material/MaterialMapper.hpp>
#include <Acts/Material/TrackingGeometryMaterial.hpp>
#include <ActsPlugins/Json/MaterialMapJsonConverter.hpp>
#include <ActsPlugins/Root/RootMaterialTrackIo.hpp>

#include <TChain.h>

#include <fmt/format.h>

#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

/// Projects the material recorded by a Geant4 geantino scan onto the material
/// receivers designated by the blueprint, and writes the resulting map.
///
/// This is step 3 of the workflow in doc/material_mapping.md. It has to run in
/// the same job as @c ActsGeoSvc: the map is keyed by
/// @c Acts::GeometryIdentifier, and for a Gen3 geometry those identifiers are
/// assigned by the blueprint traversal order, so a map is only meaningful for
/// the exact geometry it was produced against. Producing it in an external ACTS
/// job would key it on a completely different volume numbering.
///
/// The algorithm is a one-shot job rather than an event loop: it reads every
/// recorded track from the input file on the first event, so run it with
/// EvtMax=1.
struct MaterialMappingAlg final : public Gaudi::Algorithm {
  using Gaudi::Algorithm::Algorithm;

  StatusCode initialize() override;
  StatusCode execute(const EventContext&) const override;
  StatusCode finalize() override;

  Gaudi::Property<std::vector<std::string>> m_inputFiles{
      this, "InputFiles", {}, "ROOT files holding the recorded material tracks from the geantino scan."};
  Gaudi::Property<std::string>  m_treeName{this, "TreeName", "material_tracks",
                                          "Name of the TTree holding the recorded material tracks. Must match the "
                                           "treeName the recording job's RootMaterialTrackWriter used."};
  Gaudi::Property<std::string>  m_outputFile{this, "OutputFile", "material-map.json",
                                            "Path of the material map to write. Feed this back to "
                                             "ActsGeoSvc.MaterialMapFile for reconstruction."};
  Gaudi::Property<std::int64_t> m_maxTracks{
      this, "MaxTracks", -1, "Stop after this many recorded tracks. Negative (default) processes the whole input."};
  Gaudi::Property<bool> m_emptyBinCorrection{
      this, "EmptyBinCorrection", true, "Let the accumulator correct for bins that no geantino crossed (recommended)."};
  Gaudi::Property<bool> m_prePostStepInfo{
      this, "PrePostStepInfo", true,
      "Whether the input carries pre- and post-step positions. Must match the recording job's "
      "RootMaterialTrackWriter.prePostStep, which is True for a Geant4 scan."};
  Gaudi::Property<bool> m_surfaceInfo{
      this, "SurfaceInfo", false,
      "Whether the input carries per-step surface information. Must match the recording job's "
      "RootMaterialTrackWriter.storeSurface, which is False for a scan that knows nothing of our geometry."};
  Gaudi::Property<bool> m_volumeInfo{this, "VolumeInfo", false,
                                     "Whether the input carries per-step volume information. Must match the recording "
                                     "job's RootMaterialTrackWriter.storeVolume."};

private:
  SmartIF<IActsGeoSvc> m_actsGeoSvc;

  std::unique_ptr<const Acts::Logger>               m_actsLogger{nullptr};
  std::unique_ptr<Acts::MaterialMapper>             m_mapper{nullptr};
  std::unique_ptr<TChain>                           m_chain{nullptr};
  std::unique_ptr<ActsPlugins::RootMaterialTrackIo> m_trackIo{nullptr};

  // Mutated from the const execute(); the algorithm is a one-shot job and is
  // not meant to be run concurrently.
  mutable std::unique_ptr<Acts::MaterialMapper::State> m_state{nullptr};
  mutable std::size_t                                  m_nProcessed{0};
  mutable bool                                         m_done{false};

  Acts::GeometryContext      m_gctx = Acts::GeometryContext::dangerouslyDefaultConstruct();
  Acts::MagneticFieldContext m_mctx{};
};

DECLARE_COMPONENT(MaterialMappingAlg)

StatusCode MaterialMappingAlg::initialize() {
  if (auto sc = Gaudi::Algorithm::initialize(); sc.isFailure()) {
    return sc;
  }

  if (m_inputFiles.value().empty()) {
    error() << "No input files given. Set InputFiles to the output of the geantino scan." << endmsg;
    return StatusCode::FAILURE;
  }

  m_actsGeoSvc = svcLoc()->service<IActsGeoSvc>("ActsGeoSvc");
  K4_GAUDI_CHECK(m_actsGeoSvc);

  m_actsLogger = makeActsGaudiLogger(this);

  const auto trackingGeometry = m_actsGeoSvc->trackingGeometry();
  if (!trackingGeometry) {
    error() << "ActsGeoSvc provided no tracking geometry." << endmsg;
    return StatusCode::FAILURE;
  }

  // The receivers are exactly the surfaces the blueprint marked up: they carry a
  // proto-material placeholder that holds the binning but no material yet.
  const auto materialSurfaces = MaterialSurfaces::collectProtoMaterialSurfaces(*trackingGeometry);
  if (materialSurfaces.empty()) {
    error() << "The tracking geometry designates no material receivers, so there is nothing to map onto. Either this "
               "detector has no material designation in DD4hepBlueprintConstruction.cpp, or a material map was already "
               "loaded via ActsGeoSvc.MaterialMapFile (leave it unset when mapping)."
            << endmsg;
    return StatusCode::FAILURE;
  }
  info() << fmt::format("Mapping material onto {} designated surfaces.", materialSurfaces.size()) << endmsg;

  Acts::IntersectionMaterialAssigner::Config assignerCfg;
  assignerCfg.surfaces = materialSurfaces;

  Acts::BinnedSurfaceMaterialAccumulator::Config accumulatorCfg;
  accumulatorCfg.materialSurfaces   = materialSurfaces;
  accumulatorCfg.emptyBinCorrection = m_emptyBinCorrection.value();

  Acts::MaterialMapper::Config mapperCfg;
  mapperCfg.assignmentFinder = std::make_shared<const Acts::IntersectionMaterialAssigner>(
      assignerCfg, m_actsLogger->cloneWithSuffix("|Assigner"));
  mapperCfg.surfaceMaterialAccumulator = std::make_shared<const Acts::BinnedSurfaceMaterialAccumulator>(
      accumulatorCfg, m_actsLogger->cloneWithSuffix("|Accumulator"));

  m_mapper = std::make_unique<Acts::MaterialMapper>(mapperCfg, m_actsLogger->cloneWithSuffix("|Mapper"));
  m_state  = m_mapper->createState(m_gctx);

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

  // These have to mirror how the recording job configured its
  // RootMaterialTrackWriter, otherwise the branches do not line up.
  ActsPlugins::RootMaterialTrackIo::Config ioCfg;
  ioCfg.prePostStepInfo = m_prePostStepInfo.value();
  ioCfg.surfaceInfo     = m_surfaceInfo.value();
  ioCfg.volumeInfo      = m_volumeInfo.value();

  m_trackIo = std::make_unique<ActsPlugins::RootMaterialTrackIo>(ioCfg);
  m_trackIo->connectForRead(*m_chain);

  info() << fmt::format("Reading {} recorded material tracks from tree '{}'.", m_chain->GetEntries(),
                        m_treeName.value())
         << endmsg;

  return StatusCode::SUCCESS;
}

StatusCode MaterialMappingAlg::execute(const EventContext&) const {
  if (m_done) {
    return StatusCode::SUCCESS;
  }
  m_done = true;

  const std::int64_t nEntries = m_chain->GetEntries();
  const std::int64_t nToRead =
      m_maxTracks.value() < 0 ? nEntries : std::min<std::int64_t>(m_maxTracks.value(), nEntries);

  for (std::int64_t entry = 0; entry < nToRead; ++entry) {
    if (m_chain->GetEntry(entry) <= 0) {
      warning() << fmt::format("Could not read entry {}, stopping here.", entry) << endmsg;
      break;
    }
    const auto track = m_trackIo->read();
    m_mapper->mapMaterial(*m_state, m_gctx, m_mctx, track);
    m_nProcessed++;

    if (m_nProcessed % 10000 == 0) {
      info() << fmt::format("Mapped {} of {} tracks.", m_nProcessed, nToRead) << endmsg;
    }
  }

  info() << fmt::format("Mapped {} recorded material tracks.", m_nProcessed) << endmsg;
  return StatusCode::SUCCESS;
}

StatusCode MaterialMappingAlg::finalize() {
  if (m_nProcessed == 0) {
    error() << "No material tracks were mapped, refusing to write an empty map. Was the algorithm executed (EvtMax>0)?"
            << endmsg;
    return StatusCode::FAILURE;
  }

  const Acts::TrackingGeometryMaterial maps = m_mapper->finalizeMaps(*m_state, m_gctx);

  // In a Gen3 geometry the receivers are volume portals, the sensors carry no
  // material, and there is no volume material, so the writer only needs to
  // handle boundaries.
  Acts::MaterialMapJsonConverter::Config converterCfg;
  converterCfg.context             = m_gctx;
  converterCfg.processSensitives   = false;
  converterCfg.processApproaches   = false;
  converterCfg.processRepresenting = false;
  converterCfg.processBoundaries   = true;
  converterCfg.processVolumes      = false;

  Acts::MaterialMapJsonConverter converter{converterCfg, Acts::Logging::INFO};

  std::ofstream out{m_outputFile.value()};
  if (!out.is_open()) {
    error() << fmt::format("Could not open '{}' for writing.", m_outputFile.value()) << endmsg;
    return StatusCode::FAILURE;
  }
  out << converter.materialMapsToJson(maps).dump(2) << std::endl;
  out.close();

  info() << fmt::format("Wrote material for {} surfaces to '{}'.", maps.first.size(), m_outputFile.value()) << endmsg;

  return Gaudi::Algorithm::finalize();
}
