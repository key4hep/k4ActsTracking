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

// k4ActsTracking
#include "k4ActsTracking/CKFTrackingAlg.hxx"
#include "k4ActsTracking/CellIDSelector.h"
#include "k4ActsTracking/Helpers.hxx"
#include "k4ActsTracking/IActsGeoSvc.h"
#include "k4ActsTracking/Measurement.hxx"
#include "k4ActsTracking/MeasurementCalibrator.hxx"
#include "k4ActsTracking/SourceLink.hxx"

// k4FWCore
#include <k4FWCore/GaudiChecks.h>
#include <k4FWCore/Transformer.h>

#include <podio/ObjectID.h>
// edm4hep
#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackerHitPlaneCollection.h>
#include <edm4hep/TrackerHitSimTrackerHitLinkCollection.h>

// Gaudi
#include <Gaudi/Property.h>
#include <GaudiKernel/SmartIF.h>

// DD4hep
#include <DDSegmentation/BitFieldCoder.h>

// ACTS
#include <Acts/Definitions/Units.hpp>
#include <Acts/EventData/SeedContainer2.hpp>
#include <Acts/EventData/SpacePointContainer2.hpp>
#include <Acts/EventData/TrackContainer.hpp>
#include <Acts/EventData/TrackParameters.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>
#include <Acts/EventData/VectorTrackContainer.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Propagator/EigenStepper.hpp>
#include <Acts/Propagator/Navigator.hpp>
#include <Acts/Propagator/Propagator.hpp>
#include <Acts/Seeding/EstimateTrackParamsFromSeed.hpp>
#include <Acts/Seeding2/BroadTripletSeedFilter.hpp>
#include <Acts/Seeding2/CylindricalSpacePointGrid2.hpp>
#include <Acts/Seeding2/DoubletSeedFinder.hpp>
#include <Acts/Seeding2/TripletSeedFinder.hpp>
#include <Acts/Seeding2/TripletSeeder.hpp>
#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>
#include <Acts/TrackFinding/MeasurementSelector.hpp>
#include <Acts/TrackFinding/TrackStateCreator.hpp>
#include <Acts/TrackFitting/GainMatrixUpdater.hpp>
#include <Acts/Utilities/Logger.hpp>
#include <Acts/Utilities/RangeXD.hpp>

// TBB
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>

#include <fmt/ostream.h>

// Standard
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <vector>

using namespace Acts::UnitLiterals;

template <> struct fmt::formatter<podio::ObjectID> : fmt::ostream_formatter {};

/**
 * @brief Seeded CKF tracking algorithm using ActsGeoSvc.
 *
 * Functionally equivalent to ACTSSeededCKFTrackingAlg but obtains the
 * tracking geometry, magnetic field and surface–cellID mapping directly
 * from IActsGeoSvc instead of inheriting from ACTSAlgBase.
 *
 * ACTS contexts (geometry, magnetic-field, calibration) are default-
 * constructed as documented by the ACTS framework.
 */
struct CKFTrackingAlg final
    : k4FWCore::MultiTransformer<std::tuple<edm4hep::TrackCollection, edm4hep::TrackCollection>(
          const edm4hep::TrackerHitPlaneCollection&, const edm4hep::TrackerHitSimTrackerHitLinkCollection&)> {
  using TrackContainer = Acts::TrackContainer<Acts::VectorTrackContainer, Acts::VectorMultiTrajectory, std::shared_ptr>;
  using TrackFinderOptions = Acts::CombinatorialKalmanFilterOptions<TrackContainer>;

  using Stepper    = Acts::EigenStepper<>;
  using Navigator  = Acts::Navigator;
  using Propagator = Acts::Propagator<Stepper, Navigator>;
  using CKF        = Acts::CombinatorialKalmanFilter<Propagator, TrackContainer>;

  CKFTrackingAlg(const std::string& name, ISvcLocator* svcLoc);

  StatusCode initialize() override;

  std::tuple<edm4hep::TrackCollection, edm4hep::TrackCollection> operator()(
      const edm4hep::TrackerHitPlaneCollection&             trackerHitCollection,
      const edm4hep::TrackerHitSimTrackerHitLinkCollection& trackerHitRelations) const override;

  // ----- private helpers ---------------------------------------------------
private:
  /// Convert the seeds found by the triplet seeder into ACTS bound track
  /// parameters and create the corresponding edm4hep seed tracks. The space
  /// point indices stored in @p seeds reference @p spacePoints.
  std::vector<Acts::BoundTrackParameters> seedsToParameters(const Acts::SeedContainer2&        seeds,
                                                            const Acts::SpacePointContainer2&  spacePoints,
                                                            edm4hep::TrackCollection&          seedCollection,
                                                            Acts::MagneticFieldProvider::Cache& magCache) const;

  StatusCode tracking(const std::vector<Acts::BoundTrackParameters>& paramseeds, std::size_t begin, std::size_t end,
                      const CKF& trackFinder, const TrackFinderOptions& ckfOptions,
                      Acts::MagneticFieldProvider::Cache& magCache, edm4hep::TrackCollection& trackCollection) const;

  // ----- Gaudi properties --------------------------------------------------

  /// @name Run control
  ///@{
  Gaudi::Property<bool> m_runCKF{this, "RunCKF", true, "Run tracking using CKF. False means stop at seeding."};
  Gaudi::Property<bool> m_propagateBackward{this, "PropagateBackward", false, "Extrapolates tracks towards beamline."};
  ///@}

  /// @name Seed-finding configuration
  ///@{
  Gaudi::Property<float> m_seedFinding_rMax{this, "SeedFinding_RMax", 150, "Maximum radius of hits to consider."};
  Gaudi::Property<float> m_seedFinding_deltaRMin{this, "SeedFinding_DeltaRMin", 5,
                                                 "Minimum dR between hits in a seed."};
  Gaudi::Property<float> m_seedFinding_deltaRMax{this, "SeedFinding_DeltaRMax", 80,
                                                 "Maximum dR between hits in a seed."};
  Gaudi::Property<float> m_seedFinding_deltaRMinTop{this, "SeedFinding_DeltaRMinTop", 0.f,
                                                    "Minimum dR between reference and outer hits in a seed."};
  Gaudi::Property<float> m_seedFinding_deltaRMaxTop{this, "SeedFinding_DeltaRMaxTop", 0.f,
                                                    "Maximum dR between reference and outer hits in a seed."};
  Gaudi::Property<float> m_seedFinding_deltaRMinBottom{this, "SeedFinding_DeltaRMinBottom", 0.f,
                                                       "Minimum dR between reference and inner hits in a seed."};
  Gaudi::Property<float> m_seedFinding_deltaRMaxBottom{this, "SeedFinding_DeltaRMaxBottom", 0.f,
                                                       "Maximum dR between reference and inner hits in a seed."};
  Gaudi::Property<float> m_seedFinding_collisionRegion{this, "SeedFinding_CollisionRegion", 75.0,
                                                       "Half-size of the collision region along z."};
  Gaudi::Property<float> m_seedFinding_zMax{this, "SeedFinding_ZMax", 600.0, "Maximum |z| of hits to consider."};
  Gaudi::Property<float> m_seedFinding_sigmaScattering{this, "SeedFinding_SigmaScattering", 50.0,
                                                       "Number of sigmas allowed in scattering angle."};
  Gaudi::Property<float> m_seedFinding_radLengthPerSeed{this, "SeedFinding_RadLengthPerSeed", 0.1,
                                                        "Average radiation length per seed."};
  Gaudi::Property<float> m_seedFinding_minPt{this, "SeedFinding_MinPt", 500.0, "Minimum pT of tracks to seed [MeV]."};
  Gaudi::Property<float> m_seedFinding_impactMax{this, "SeedFinding_ImpactMax", 3.0,
                                                 "Maximum d0 of tracks to seed [mm]."};

  std::vector<std::string>                  m_default_empty_vec;
  Gaudi::Property<std::vector<std::string>> m_seedFinding_zBinEdges{this, "SeedFinding_zBinEdges", m_default_empty_vec,
                                                                    "Custom z bin edges for the seeding grid."};
  Gaudi::Property<int> m_zTopBinLen{this, "SeedFinding_zTopBinLen", 1, "Number of top bins along Z for seeding."};
  Gaudi::Property<int> m_zBottomBinLen{this, "SeedFinding_zBottomBinLen", 1,
                                       "Number of bottom bins along Z for seeding."};
  Gaudi::Property<int> m_phiTopBinLen{this, "SeedFinding_phiTopBinLen", 1, "Number of top bins along phi for seeding."};
  Gaudi::Property<int> m_phiBottomBinLen{this, "SeedFinding_phiBottomBinLen", 1,
                                         "Number of bottom bins along phi for seeding."};
  ///@}

  /// @name Track-fit initial error estimates
  ///@{
  Gaudi::Property<double>  m_initialTrackError_pos{this, "InitialTrackError_Pos", 10 * Acts::UnitConstants::um,
                                                  "Initial track error for local position."};
  Gaudi::Property<double>  m_initialTrackError_phi{this, "InitialTrackError_Phi", 1 * Acts::UnitConstants::degree,
                                                  "Initial track error for phi."};
  Gaudi::Property<double>  m_initialTrackError_relP{this, "InitialTrackError_RelP", 0.25,
                                                   "Initial track error for momentum (relative)."};
  Gaudi::Property<double>  m_initialTrackError_lambda{this, "InitialTrackError_Lambda", 1 * Acts::UnitConstants::degree,
                                                     "Initial track error for lambda."};
  Gaudi::Property<double>  m_initialTrackError_time{this, "InitialTrackError_Time", 100 * Acts::UnitConstants::ns,
                                                   "Initial track error for time."};
  Gaudi::Property<double>  m_CKF_chi2CutOff{this, "CKF_Chi2CutOff", 15, "Maximum local chi2 contribution."};
  Gaudi::Property<int32_t> m_CKF_numMeasurementsCutOff{this, "CKF_NumMeasurementsCutOff", 10,
                                                       "Maximum measurements on a single surface."};
  ///@}

  /// @name Seeding layer selection
  ///@{
  Gaudi::Property<std::vector<std::string>> m_seedingSensorsCellIDs{
      this, "SeedingSensorsCellIDs", m_default_empty_vec,
      "CellIDSelector selection strings for seed space-point filtering. "
      "Each entry is a comma-separated list of field:value constraints (e.g. \"system:1,layer:2|3\"). "
      "Multiple entries are OR-ed together. Omitted fields act as wildcards."};
  ///@}

  /// @name Multi-threading
  ///@{
  Gaudi::Property<int> m_numThreads{this, "NumThreads", 1, "Number of threads for internal TBB parallelism."};
  ///@}

  SmartIF<IActsGeoSvc> m_actsGeoSvc;

  // Track finder (propagator) is geometry/field dependent only, so it is built
  // once in initialize() and reused (read-only) across events and threads.
  std::optional<CKF> m_trackFinder{};

  k4ActsTracking::CellIDSelector m_seedSelector{};

  mutable std::mutex m_seedMutex{};
  mutable std::mutex m_trackMutex{};
};

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CKFTrackingAlg::CKFTrackingAlg(const std::string& name, ISvcLocator* svcLoc)
    : MultiTransformer(name, svcLoc,
                       {KeyValue("InputTrackerHitCollection", "TrackerHits"),
                        KeyValue("InputTrackerHitRelationCollection", "TrackerHitRelations")},
                       {KeyValue("OutputSeedCollection", "SeedTracks"), KeyValue("OutputTrackCollection", "Tracks")}) {}

StatusCode CKFTrackingAlg::initialize() {
  m_actsGeoSvc = svcLoc()->service<IActsGeoSvc>("ActsGeoSvc");
  K4_GAUDI_CHECK(m_actsGeoSvc);

  m_seedSelector =
      k4ActsTracking::CellIDSelector(m_actsGeoSvc->cellIDEncodingString(), m_seedingSensorsCellIDs.value());

  // Apply deltaR fallback defaults
  if (m_seedFinding_deltaRMinTop == 0.f)
    m_seedFinding_deltaRMinTop = m_seedFinding_deltaRMin;
  if (m_seedFinding_deltaRMaxTop == 0.f)
    m_seedFinding_deltaRMaxTop = m_seedFinding_deltaRMax;
  if (m_seedFinding_deltaRMinBottom == 0.f)
    m_seedFinding_deltaRMinBottom = m_seedFinding_deltaRMin;
  if (m_seedFinding_deltaRMaxBottom == 0.f)
    m_seedFinding_deltaRMaxBottom = m_seedFinding_deltaRMax;

  // The propagator and CKF only depend on the tracking geometry and magnetic
  // field, both available here, so build them once instead of per event.
  Navigator::Config navigatorCfg{m_actsGeoSvc->trackingGeometry()};
  navigatorCfg.resolvePassive   = false;
  navigatorCfg.resolveMaterial  = true;
  navigatorCfg.resolveSensitive = true;

  Stepper    stepper(m_actsGeoSvc->magneticField());
  Navigator  navigator(navigatorCfg);
  Propagator propagator(std::move(stepper), std::move(navigator));
  m_trackFinder.emplace(std::move(propagator));

  return StatusCode::SUCCESS;
}

std::tuple<edm4hep::TrackCollection, edm4hep::TrackCollection> CKFTrackingAlg::operator()(
    const edm4hep::TrackerHitPlaneCollection& trackerHitCollection,
    const edm4hep::TrackerHitSimTrackerHitLinkCollection& /*trackerHitRelations*/) const {
  edm4hep::TrackCollection seedCollection;
  edm4hep::TrackCollection trackCollection;

  // Default-construct ACTS contexts
  const Acts::GeometryContext      geoCtx = Acts::GeometryContext::dangerouslyDefaultConstruct();
  const Acts::MagneticFieldContext magCtx{};
  const Acts::CalibrationContext   calCtx{};

  const auto& cellIdToSurface = m_actsGeoSvc->cellIdToSurfaceMap();

  dd4hep::DDSegmentation::BitFieldCoder decoder{m_actsGeoSvc->cellIDEncodingString()};

  // Minimal per-hit information needed to build the seeding space points. The
  // global position and rho/z variances are computed during the hit loop and
  // later transferred (in grid-bin order) into an Acts::SpacePointContainer2.
  struct SeedInput {
    float                    x, y, z, r, phi;
    float                    varR, varZ;
    ACTSTracking::SourceLink sourceLink;
  };

  std::vector<std::pair<Acts::GeometryIdentifier, edm4hep::TrackerHitPlane>> sortedHits;
  ACTSTracking::SourceLinkContainer                                          sourceLinks;
  ACTSTracking::MeasurementContainer                                         measurements;
  std::vector<SeedInput>                                                     seedInputs;

  sortedHits.reserve(trackerHitCollection.size());

  for (const auto& hit : trackerHitCollection) {
    verbose() << fmt::format("Adding hit {} with cell id {:x}", hit.id(), hit.getCellID()) << endmsg;
    auto it = cellIdToSurface.find(hit.getCellID());
    if (it == cellIdToSurface.end()) {
      warning() << "No surface found for cellID " << hit.getCellID() << ". skipping hit for tracking." << endmsg;
      continue;
    }
    sortedHits.push_back({it->second->geometryId(), hit});
  }
  debug() << "Working with " << sortedHits.size() << " hits." << endmsg;

  // Sort hits by geometry ID for efficient SourceLink multiset insertion
  auto            compare = [](const auto& a, const auto& b) { return a.first < b.first; };
  tbb::task_arena arena(m_numThreads.value());
  if (m_numThreads > 1) {
    arena.execute([&] { tbb::parallel_sort(sortedHits.begin(), sortedHits.end(), compare); });
  } else {
    std::sort(sortedHits.begin(), sortedHits.end(), compare);
  }

  sourceLinks.reserve(sortedHits.size());

  for (const auto& hitPair : sortedHits) {
    const Acts::Surface* surface = m_actsGeoSvc->trackingGeometry()->findSurface(hitPair.first);
    if (surface == nullptr) {
      warning() << "Surface with geoID " << hitPair.first
                << " not found in tracking geometry. Skipping hit for tracking." << endmsg;
      continue;
    }

    const edm4hep::Vector3d& edmGlobalPos = hitPair.second.getPosition();
    Acts::Vector3            globalPos    = {edmGlobalPos.x, edmGlobalPos.y, edmGlobalPos.z};

    verbose() << "Converting hit " << hitPair.second.id() << " to local position (pos = " << edmGlobalPos
              << ") using surface with geoId " << hitPair.first << endmsg;

    Acts::Result<Acts::Vector2> lpResult = surface->globalToLocal(geoCtx, globalPos, {0, 0, 0}, 0.5_um);
    if (!lpResult.ok()) {
      warning() << "Global to local transformation did not succeed for hit. Skipping it in tracking." << endmsg;
      continue;
    }

    Acts::Vector2 loc = lpResult.value();

    Acts::SquareMatrix2            localCov = Acts::SquareMatrix2::Zero();
    const edm4hep::TrackerHitPlane hitplane = hitPair.second;
    localCov(0, 0)                          = std::pow(hitplane.getDu() * Acts::UnitConstants::mm, 2);
    localCov(1, 1)                          = std::pow(hitplane.getDv() * Acts::UnitConstants::mm, 2);

    ACTSTracking::SourceLink  sourceLink(surface->geometryId(), measurements.size(), hitPair.second);
    Acts::SourceLink          srcWrap{sourceLink};
    ACTSTracking::Measurement meas =
        ACTSTracking::makeMeasurement(srcWrap, loc, localCov, Acts::eBoundLoc0, Acts::eBoundLoc1);

    measurements.push_back(meas);
    sourceLinks.emplace_hint(sourceLinks.end(), sourceLink);

    // Create space point for seeding if this surface is selected
    if (m_seedSelector.accept(hitPair.second.getCellID())) {
      Acts::RotationMatrix3 rotLocalToGlobal = surface->referenceFrame(geoCtx, globalPos, {0, 0, 0});

      // Jacobian from global (x,y,z) to (rho, z)
      double             x            = globalPos[Acts::ePos0];
      double             y            = globalPos[Acts::ePos1];
      double             scale        = 2 / std::hypot(x, y);
      Acts::Matrix<2, 3> jacXyzToRhoZ = Acts::Matrix<2, 3>::Zero();
      jacXyzToRhoZ(0, Acts::ePos0)    = scale * x;
      jacXyzToRhoZ(0, Acts::ePos1)    = scale * y;
      jacXyzToRhoZ(1, Acts::ePos2)    = 1;
      const auto jac                  = jacXyzToRhoZ * rotLocalToGlobal.block<3, 2>(Acts::ePos0, Acts::ePos0);
      const auto var                  = (jac * localCov * jac.transpose()).diagonal();

      SeedInput sp;
      sp.x          = static_cast<float>(globalPos[Acts::ePos0]);
      sp.y          = static_cast<float>(globalPos[Acts::ePos1]);
      sp.z          = static_cast<float>(globalPos[Acts::ePos2]);
      sp.r          = std::hypot(sp.x, sp.y);
      sp.phi        = std::atan2(sp.y, sp.x);
      sp.varR       = static_cast<float>(var[0]);
      sp.varZ       = static_cast<float>(var[1]);
      sp.sourceLink = sourceLink;
      seedInputs.push_back(sp);
    }
  }

  debug() << fmt::format("Created {} sourceLinks and {} space points for seeding", sourceLinks.size(),
                         seedInputs.size())
          << endmsg;

  Acts::MagneticFieldProvider::Cache magCache = m_actsGeoSvc->magneticField()->makeCache(magCtx);

  static const Acts::Vector3 zeropos(0, 0, 0);

  const float bFieldInZ      = (*m_actsGeoSvc->magneticField()->getField(zeropos, magCache))[2];
  const float cotThetaMax    = 7.40627f;  // ~2.7 η
  const float minPt          = m_seedFinding_minPt * Acts::UnitConstants::MeV;
  const float impactMax      = m_seedFinding_impactMax * Acts::UnitConstants::mm;
  const float collisionRegion = m_seedFinding_collisionRegion;

  // -------------------------------------------------------------------------
  // Seeding grid (SoA): bin the seed space points in (phi, z, r).
  // -------------------------------------------------------------------------
  Acts::CylindricalSpacePointGrid2::Config gridCfg;
  gridCfg.minPt       = minPt;
  gridCfg.rMin        = 0.f;
  gridCfg.rMax        = m_seedFinding_rMax;
  gridCfg.zMin        = -m_seedFinding_zMax;
  gridCfg.zMax        = m_seedFinding_zMax;
  gridCfg.deltaRMax   = m_seedFinding_deltaRMax;
  gridCfg.cotThetaMax = cotThetaMax;
  gridCfg.impactMax   = impactMax;
  gridCfg.bFieldInZ   = bFieldInZ;
  if (!m_seedFinding_zBinEdges.empty()) {
    gridCfg.zBinEdges.resize(m_seedFinding_zBinEdges.size());
    for (std::size_t k = 0; k < m_seedFinding_zBinEdges.size(); k++) {
      float pos = std::atof(m_seedFinding_zBinEdges[k].c_str());
      if (pos >= gridCfg.zMin && pos < gridCfg.zMax) {
        gridCfg.zBinEdges[k] = pos;
      } else {
        warning() << "Wrong parameter SeedFinding_zBinEdges; default used" << endmsg;
        gridCfg.zBinEdges.clear();
        break;
      }
    }
  }
  gridCfg.bottomBinFinder.emplace(m_phiBottomBinLen.value(), m_zBottomBinLen.value(), 0);
  gridCfg.topBinFinder.emplace(m_phiTopBinLen.value(), m_zTopBinLen.value(), 0);

  Acts::CylindricalSpacePointGrid2 grid(gridCfg, Acts::getDefaultLogger("CKFSeedingGrid", Acts::Logging::WARNING));

  for (std::size_t i = 0; i < seedInputs.size(); ++i) {
    const SeedInput& sp = seedInputs[i];
    grid.insert(static_cast<Acts::SpacePointIndex2>(i), sp.phi, sp.z, sp.r);
  }

  // Sort each bin by radius, as required by the radius-sorted doublet finders.
  for (std::size_t i = 0; i < grid.numberOfBins(); ++i) {
    std::ranges::sort(grid.at(i), [&](const Acts::SpacePointIndex2& a, const Acts::SpacePointIndex2& b) {
      return seedInputs[a].r < seedInputs[b].r;
    });
  }

  // -------------------------------------------------------------------------
  // Build the SoA space point container in grid-bin order so that every bin
  // maps to a contiguous index range, as expected by the triplet seeder.
  // -------------------------------------------------------------------------
  Acts::SpacePointContainer2 spacePoints(Acts::SpacePointColumns::SourceLinks | Acts::SpacePointColumns::PackedXY |
                                         Acts::SpacePointColumns::PackedZR | Acts::SpacePointColumns::VarianceZ |
                                         Acts::SpacePointColumns::VarianceR);
  spacePoints.reserve(grid.numberOfSpacePoints());
  std::vector<Acts::SpacePointIndexRange2> gridSpacePointRanges;
  gridSpacePointRanges.reserve(grid.numberOfBins());
  for (std::size_t i = 0; i < grid.numberOfBins(); ++i) {
    std::uint32_t begin = spacePoints.size();
    for (Acts::SpacePointIndex2 spIndex : grid.at(i)) {
      const SeedInput& in    = seedInputs[spIndex];
      auto             newSp = spacePoints.createSpacePoint();
      newSp.xy()             = {in.x, in.y};
      newSp.zr()             = {in.z, in.r};
      newSp.varianceR()      = in.varR;
      newSp.varianceZ()      = in.varZ;
      std::array<Acts::SourceLink, 1> sls{Acts::SourceLink{in.sourceLink}};
      newSp.assignSourceLinks(sls);
    }
    std::uint32_t end = spacePoints.size();
    gridSpacePointRanges.emplace_back(begin, end);
  }

  // Radius range, exploiting the per-bin radius sorting performed above.
  float minRange = std::numeric_limits<float>::max();
  float maxRange = std::numeric_limits<float>::lowest();
  for (const Acts::SpacePointIndexRange2& range : gridSpacePointRanges) {
    if (range.first == range.second)
      continue;
    minRange = std::min(spacePoints[range.first].zr()[1], minRange);
    maxRange = std::max(spacePoints[range.second - 1].zr()[1], maxRange);
  }

  // Variable middle space point radial region of interest.
  constexpr float            deltaRMiddleMinSPRange = 10.f * Acts::UnitConstants::mm;
  constexpr float            deltaRMiddleMaxSPRange = 10.f * Acts::UnitConstants::mm;
  const std::pair<float, float> rMiddleSPRange{std::floor(minRange / 2) * 2 + deltaRMiddleMinSPRange,
                                               std::floor(maxRange / 2) * 2 - deltaRMiddleMaxSPRange};

  // -------------------------------------------------------------------------
  // Doublet / triplet finders and seed filter.
  // -------------------------------------------------------------------------
  Acts::DoubletSeedFinder::Config bottomFinderCfg;
  bottomFinderCfg.spacePointsSortedByRadius = true;
  bottomFinderCfg.candidateDirection        = Acts::Direction::Backward();
  bottomFinderCfg.deltaRMin                 = m_seedFinding_deltaRMinBottom;
  bottomFinderCfg.deltaRMax                 = m_seedFinding_deltaRMaxBottom;
  bottomFinderCfg.impactMax                 = impactMax;
  bottomFinderCfg.collisionRegionMin        = -collisionRegion;
  bottomFinderCfg.collisionRegionMax        = collisionRegion;
  bottomFinderCfg.cotThetaMax               = cotThetaMax;
  bottomFinderCfg.minPt                     = minPt;
  auto bottomFinder =
      Acts::DoubletSeedFinder::create(Acts::DoubletSeedFinder::DerivedConfig(bottomFinderCfg, bFieldInZ));

  Acts::DoubletSeedFinder::Config topFinderCfg = bottomFinderCfg;
  topFinderCfg.candidateDirection             = Acts::Direction::Forward();
  topFinderCfg.deltaRMin                       = m_seedFinding_deltaRMinTop;
  topFinderCfg.deltaRMax                       = m_seedFinding_deltaRMaxTop;
  auto topFinder = Acts::DoubletSeedFinder::create(Acts::DoubletSeedFinder::DerivedConfig(topFinderCfg, bFieldInZ));

  Acts::TripletSeedFinder::Config tripletFinderCfg;
  tripletFinderCfg.useStripInfo     = false;
  tripletFinderCfg.sortedByCotTheta = true;
  tripletFinderCfg.minPt            = minPt;
  tripletFinderCfg.sigmaScattering  = m_seedFinding_sigmaScattering;
  tripletFinderCfg.radLengthPerSeed = m_seedFinding_radLengthPerSeed;
  tripletFinderCfg.impactMax        = impactMax;
  auto tripletFinder =
      Acts::TripletSeedFinder::create(Acts::TripletSeedFinder::DerivedConfig(tripletFinderCfg, bFieldInZ));

  Acts::BroadTripletSeedFilter::Config filterCfg;
  filterCfg.deltaRMin      = m_seedFinding_deltaRMin;
  filterCfg.maxSeedsPerSpM = 1;

  auto seedingLogger = Acts::getDefaultLogger("CKFSeeding", Acts::Logging::WARNING);

  // The triplet seeder itself is stateless: all per-event scratch lives in the
  // Cache argument, so a single const instance is shared across threads.
  const Acts::TripletSeeder seeder(seedingLogger->clone());

  // Collect the binned groups up front so they can be processed in parallel.
  using GroupValue = std::decay_t<decltype(*grid.binnedGroup().begin())>;
  std::vector<GroupValue> groups;
  groups.reserve(grid.numberOfBins());
  for (const auto& group : grid.binnedGroup()) {
    groups.push_back(group);
  }

  // -------------------------------------------------------------------------
  // Combinatorial Kalman filter track-finding objects. These are read-only
  // during track finding and therefore shared across threads.
  // -------------------------------------------------------------------------
  const CKF& trackFinder = *m_trackFinder;

  Acts::MeasurementSelector::Config measurementSelectorCfg = {
      {Acts::GeometryIdentifier(), {{}, {m_CKF_chi2CutOff}, {(std::size_t)(m_CKF_numMeasurementsCutOff)}}}};

  Acts::PropagatorPlainOptions pOptions{geoCtx, magCtx};
  pOptions.maxSteps = 10000;
  if (m_propagateBackward) {
    pOptions.direction = Acts::Direction::Backward();
  }

  Acts::GainMatrixUpdater             kfUpdater;
  Acts::MeasurementSelector           measSel{measurementSelectorCfg};
  ACTSTracking::MeasurementCalibrator measCal{measurements};

  ACTSTracking::SourceLinkAccessor slAccessor;
  slAccessor.container = &sourceLinks;

  using TrackStateCreatorType = Acts::TrackStateCreator<ACTSTracking::SourceLinkAccessor::Iterator, TrackContainer>;
  TrackStateCreatorType trackStateCreator;
  trackStateCreator.sourceLinkAccessor.template connect<&ACTSTracking::SourceLinkAccessor::range>(&slAccessor);
  trackStateCreator.calibrator.template connect<&ACTSTracking::MeasurementCalibrator::calibrate>(&measCal);
  trackStateCreator.measurementSelector
      .template connect<&Acts::MeasurementSelector::select<Acts::VectorMultiTrajectory>>(&measSel);

  Acts::CombinatorialKalmanFilterExtensions<TrackContainer> extensions;
  extensions.updater.connect<&Acts::GainMatrixUpdater::operator()<Acts::VectorMultiTrajectory>>(&kfUpdater);
  extensions.createTrackStates.template connect<&TrackStateCreatorType::createTrackStates>(&trackStateCreator);

  TrackFinderOptions ckfOptions = TrackFinderOptions(geoCtx, magCtx, calCtx, extensions, pOptions);

  // -------------------------------------------------------------------------
  // Seeding and CKF track finding, parallelised over the grid groups. Each
  // task owns its seeder cache, seed filter (with its own state/cache), seed
  // container and magnetic-field cache; the shared edm4hep collections are
  // guarded by mutexes (m_seedMutex / m_trackMutex).
  // -------------------------------------------------------------------------
  auto parallelSeedingAndTracking = [&](const tbb::blocked_range<size_t>& r) {
    // The magnetic-field cache is mutated on every field lookup, so each
    // parallel invocation needs its own cache rather than sharing one.
    Acts::MagneticFieldProvider::Cache localMagCache = m_actsGeoSvc->magneticField()->makeCache(magCtx);

    Acts::TripletSeeder::Cache          seederCache;
    Acts::BroadTripletSeedFilter::State filterState;
    Acts::BroadTripletSeedFilter::Cache filterCache;
    Acts::BroadTripletSeedFilter        seedFilter(filterCfg, filterState, filterCache, *seedingLogger);

    Acts::SeedContainer2 seeds;
    seeds.assignSpacePointContainer(spacePoints);

    std::vector<Acts::SpacePointContainer2::ConstRange> bottomSpRanges;
    std::vector<Acts::SpacePointContainer2::ConstRange> topSpRanges;

    for (size_t i = r.begin(); i != r.end(); ++i) {
      const auto& [bottom, middle, top] = groups[i];

      Acts::SpacePointContainer2::ConstRange middleSpRange =
          spacePoints.range(gridSpacePointRanges.at(middle)).asConst();
      if (middleSpRange.empty())
        continue;

      bottomSpRanges.clear();
      for (const auto b : bottom) {
        bottomSpRanges.push_back(spacePoints.range(gridSpacePointRanges.at(b)).asConst());
      }
      topSpRanges.clear();
      for (const auto t : top) {
        topSpRanges.push_back(spacePoints.range(gridSpacePointRanges.at(t)).asConst());
      }

      seeder.createSeedsFromGroups(seederCache, *bottomFinder, *topFinder, *tripletFinder, seedFilter, spacePoints,
                                   bottomSpRanges, middleSpRange, topSpRanges, rMiddleSPRange, seeds);
    }

    // Convert this task's seeds into bound track parameters and seed tracks.
    std::vector<Acts::BoundTrackParameters> paramseeds =
        seedsToParameters(seeds, spacePoints, seedCollection, localMagCache);

    if (!m_runCKF)
      return;

    if (!tracking(paramseeds, 0, paramseeds.size(), trackFinder, ckfOptions, localMagCache, trackCollection)
             .isSuccess()) {
      warning() << "Tracking failed for this event" << endmsg;
    }
  };

  if (m_numThreads > 1) {
    arena.execute([&] { tbb::parallel_for(tbb::blocked_range<size_t>(0, groups.size()), parallelSeedingAndTracking); });
  } else {
    parallelSeedingAndTracking(tbb::blocked_range<size_t>(0, groups.size()));
  }

  debug() << "Track Collection Size: " << trackCollection.size() << endmsg;
  return std::make_tuple(std::move(seedCollection), std::move(trackCollection));
}

std::vector<Acts::BoundTrackParameters> CKFTrackingAlg::seedsToParameters(
    const Acts::SeedContainer2& seeds, const Acts::SpacePointContainer2& spacePoints,
    edm4hep::TrackCollection& seedCollection, Acts::MagneticFieldProvider::Cache& magCache) const {
  const Acts::GeometryContext geoCtx = Acts::GeometryContext::dangerouslyDefaultConstruct();

  std::vector<Acts::BoundTrackParameters> paramseeds;
  paramseeds.reserve(seeds.size());

  auto position = [](const Acts::ConstSpacePointProxy2& sp) {
    return Acts::Vector3(sp.xy()[0], sp.xy()[1], sp.zr()[0]);
  };
  auto sourceLinkOf = [](const Acts::ConstSpacePointProxy2& sp) -> const ACTSTracking::SourceLink& {
    return sp.sourceLinks()[0].get<ACTSTracking::SourceLink>();
  };

  for (const Acts::ConstSeedProxy2& seed : seeds) {
    const std::span<const Acts::SpacePointIndex2> spIndices = seed.spacePointIndices();
    if (spIndices.size() != 3) {
      continue;
    }

    const Acts::ConstSpacePointProxy2 bottomSp = spacePoints[spIndices[0]];
    const Acts::ConstSpacePointProxy2 middleSp = spacePoints[spIndices[1]];
    const Acts::ConstSpacePointProxy2 topSp    = spacePoints[spIndices[2]];

    const ACTSTracking::SourceLink& bottomSL = sourceLinkOf(bottomSp);
    const Acts::GeometryIdentifier  geoId    = bottomSL.geometryId();
    const Acts::Surface*            surface  = m_actsGeoSvc->trackingGeometry()->findSurface(geoId);
    if (surface == nullptr) {
      warning() << "Surface with geoID " << geoId << " not found in tracking geometry" << endmsg;
      continue;
    }

    const Acts::Vector3 bottomPos = position(bottomSp);
    const Acts::Vector3 middlePos = position(middleSp);
    const Acts::Vector3 topPos    = position(topSp);
    const double        t0        = bottomSL.edm4hepHit().getTime();

    // Magnetic field at the seed (bottom space point) position
    Acts::Result<Acts::Vector3> seedField = m_actsGeoSvc->magneticField()->getField(bottomPos, magCache);
    if (!seedField.ok()) {
      throw std::runtime_error("Field lookup error: " + std::to_string(seedField.error().value()));
    }

    Acts::Result<Acts::BoundVector> optParams =
        Acts::estimateTrackParamsFromSeed(geoCtx, *surface, bottomPos, t0, middlePos, topPos, *seedField);
    if (!optParams.ok()) {
      debug() << "Failed estimation of track parameters for seed." << endmsg;
      continue;
    }

    const Acts::BoundVector& params = *optParams;
    float                    p      = std::abs(1.f / params[Acts::eBoundQOverP]);

    Acts::BoundMatrix cov = ACTSTracking::makeInitialCovariance(p, m_initialTrackError_pos, m_initialTrackError_phi,
                                                                m_initialTrackError_lambda, m_initialTrackError_relP,
                                                                m_initialTrackError_time);

    Acts::BoundTrackParameters paramseed(surface->getSharedPtr(), params, cov, Acts::ParticleHypothesis::pion());
    paramseeds.push_back(paramseed);

    Acts::Vector3 globalPos =
        surface->localToGlobal(geoCtx, {params[Acts::eBoundLoc0], params[Acts::eBoundLoc1]}, {0, 0, 0});
    Acts::Result<Acts::Vector3> hitField = m_actsGeoSvc->magneticField()->getField(globalPos, magCache);
    if (!hitField.ok()) {
      throw std::runtime_error("Field lookup error: " + std::to_string(hitField.error().value()));
    }

    auto seedTrackState = ACTSTracking::ACTS2edm4hep_trackState(edm4hep::TrackState::AtFirstHit, paramseed,
                                                                (*hitField)[2] / Acts::UnitConstants::T);

    {
      std::lock_guard<std::mutex> lock(m_seedMutex);
      auto                        seedTrack = seedCollection.create();
      seedTrack.addToTrackerHits(bottomSL.edm4hepHit());
      seedTrack.addToTrackerHits(sourceLinkOf(middleSp).edm4hepHit());
      seedTrack.addToTrackerHits(sourceLinkOf(topSp).edm4hepHit());
      seedTrack.addToTrackStates(seedTrackState);
    }

    debug() << "Seed Parameters" << std::endl << paramseed << endmsg;
  }

  debug() << "Seeds found: " << paramseeds.size() << endmsg;
  return paramseeds;
}

StatusCode CKFTrackingAlg::tracking(const std::vector<Acts::BoundTrackParameters>& paramseeds, std::size_t begin,
                                    std::size_t end, const CKF& trackFinder, const TrackFinderOptions& ckfOptions,
                                    Acts::MagneticFieldProvider::Cache& magCache,
                                    edm4hep::TrackCollection&           trackCollection) const {
  const Acts::GeometryContext geoCtx = Acts::GeometryContext::dangerouslyDefaultConstruct();

  debug() << "Starting CKF track finding on " << (end - begin) << " seeds." << endmsg;

  auto           trackContainer      = std::make_shared<Acts::VectorTrackContainer>();
  auto           trackStateContainer = std::make_shared<Acts::VectorMultiTrajectory>();
  TrackContainer tracks(trackContainer, trackStateContainer);

  for (std::size_t iseed = begin; iseed < end; ++iseed) {
    tracks.clear();
    auto result = trackFinder.findTracks(paramseeds.at(iseed), ckfOptions, tracks);
    if (result.ok()) {
      const auto& fitOutput = result.value();
      for (const TrackContainer::TrackProxy& trackItem : fitOutput) {
        auto trackTip = tracks.makeTrack();
        trackTip.copyFrom(trackItem);
        auto smoothResult = Acts::smoothTrack(geoCtx, trackTip);
        if (!smoothResult.ok()) {
          warning() << "Track smoothing error: " << smoothResult.error() << endmsg;
          continue;
        }

        auto track = ACTSTracking::ACTS2edm4hep_track(trackTip, m_actsGeoSvc->magneticField(), magCache);
        {
          std::lock_guard lock{m_trackMutex};
          trackCollection.push_back(track);
        }
      }
    } else {
      warning() << "Track fit error: " << result.error() << endmsg;
    }
  }

  return StatusCode::SUCCESS;
}

DECLARE_COMPONENT(CKFTrackingAlg);
