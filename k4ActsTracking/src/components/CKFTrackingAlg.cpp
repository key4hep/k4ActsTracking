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
#include "k4ActsTracking/SeedSpacePoint.hxx"
#include "k4ActsTracking/SourceLink.hxx"
#include "k4ActsTracking/SpacePointContainer.hxx"

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
#include <Acts/EventData/SpacePointContainer.hpp>
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
#include <Acts/Seeding/SeedFinder.hpp>
#include <Acts/Seeding/SpacePointGrid.hpp>
#include <Acts/Seeding/detail/CylindricalSpacePointGrid.hpp>
#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>
#include <Acts/TrackFinding/MeasurementSelector.hpp>
#include <Acts/TrackFinding/TrackStateCreator.hpp>
#include <Acts/TrackFitting/GainMatrixUpdater.hpp>
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

  using SSPoint = typename Acts::SpacePointContainer<
      ACTSTracking::SpacePointContainer<std::vector<const ACTSTracking::SeedSpacePoint*>>,
      Acts::detail::RefHolder>::SpacePointProxyType;

  using SSPointGrid = Acts::CylindricalSpacePointGrid<SSPoint>;

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
  std::vector<Acts::BoundTrackParameters> findSeeds(const Acts::SeedFinder<SSPoint, SSPointGrid>& finder,
                                                    const Acts::SeedFinderOptions&                finderOpts,
                                                    const auto& spacePointGroup, const SSPointGrid& grid,
                                                    Acts::Range1D<float> middleSpRange, std::size_t mutSpDataSize,
                                                    edm4hep::TrackCollection&           seedCollection,
                                                    Acts::MagneticFieldProvider::Cache& magCache) const;

  StatusCode tracking(const std::vector<Acts::BoundTrackParameters>& paramseeds, const CKF& trackFinder,
                      const TrackFinderOptions& ckfOptions, Acts::MagneticFieldProvider::Cache& magCache,
                      edm4hep::TrackCollection& trackCollection) const;

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

  k4ActsTracking::CellIDSelector m_seedSelector{};

  mutable std::mutex m_seedMutex{};
  mutable std::mutex m_trackMutex{};
};

DECLARE_COMPONENT(CKFTrackingAlg)

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

  std::vector<std::pair<Acts::GeometryIdentifier, edm4hep::TrackerHitPlane>> sortedHits;
  ACTSTracking::SourceLinkContainer                                          sourceLinks;
  ACTSTracking::MeasurementContainer                                         measurements;
  ACTSTracking::SeedSpacePointContainer                                      spacePoints;

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

      spacePoints.push_back(ACTSTracking::SeedSpacePoint(globalPos, var[0], var[1], sourceLink));
    }
  }

  debug() << fmt::format("Created {} sourceLinks and {} space points for seeding", sourceLinks.size(),
                         spacePoints.size())
          << endmsg;

  Acts::MagneticFieldProvider::Cache magCache = m_actsGeoSvc->magneticField()->makeCache(magCtx);

  static const Acts::Vector3 zeropos(0, 0, 0);

  Acts::SeedFinderConfig<SSPoint> finderCfg;
  finderCfg.rMax                     = m_seedFinding_rMax;
  finderCfg.deltaRMin                = m_seedFinding_deltaRMin;
  finderCfg.deltaRMax                = m_seedFinding_deltaRMax;
  finderCfg.deltaRMinTopSP           = m_seedFinding_deltaRMinTop;
  finderCfg.deltaRMaxTopSP           = m_seedFinding_deltaRMaxTop;
  finderCfg.deltaRMinBottomSP        = m_seedFinding_deltaRMinBottom;
  finderCfg.deltaRMaxBottomSP        = m_seedFinding_deltaRMaxBottom;
  finderCfg.collisionRegionMin       = -m_seedFinding_collisionRegion;
  finderCfg.collisionRegionMax       = m_seedFinding_collisionRegion;
  finderCfg.zMin                     = -m_seedFinding_zMax;
  finderCfg.zMax                     = m_seedFinding_zMax;
  finderCfg.maxSeedsPerSpM           = 1;
  finderCfg.cotThetaMax              = 7.40627;  // ~2.7 η
  finderCfg.sigmaScattering          = m_seedFinding_sigmaScattering;
  finderCfg.radLengthPerSeed         = m_seedFinding_radLengthPerSeed;
  finderCfg.minPt                    = m_seedFinding_minPt * Acts::UnitConstants::MeV;
  finderCfg.impactMax                = m_seedFinding_impactMax * Acts::UnitConstants::mm;
  finderCfg.useVariableMiddleSPRange = true;

  Acts::SeedFilterConfig filterCfg;
  filterCfg.maxSeedsPerSpM = finderCfg.maxSeedsPerSpM;
  finderCfg.seedFilter     = std::make_unique<Acts::SeedFilter<SSPoint>>(filterCfg);
  finderCfg                = finderCfg.calculateDerivedQuantities();

  Acts::SeedFinderOptions finderOpts;
  finderOpts.bFieldInZ = (*m_actsGeoSvc->magneticField()->getField(zeropos, magCache))[2];
  finderOpts.beamPos   = {0, 0};
  finderOpts           = finderOpts.calculateDerivedQuantities(finderCfg);

  Acts::CylindricalSpacePointGridConfig gridCfg;
  gridCfg.cotThetaMax = finderCfg.cotThetaMax;
  gridCfg.deltaRMax   = finderCfg.deltaRMax;
  gridCfg.minPt       = finderCfg.minPt;
  gridCfg.rMax        = finderCfg.rMax;
  gridCfg.zMax        = finderCfg.zMax;
  gridCfg.zMin        = finderCfg.zMin;
  gridCfg.impactMax   = finderCfg.impactMax;
  if (!m_seedFinding_zBinEdges.empty()) {
    gridCfg.zBinEdges.resize(m_seedFinding_zBinEdges.size());
    for (std::size_t k = 0; k < m_seedFinding_zBinEdges.size(); k++) {
      float pos = std::atof(m_seedFinding_zBinEdges[k].c_str());
      if (pos >= finderCfg.zMin && pos < finderCfg.zMax) {
        gridCfg.zBinEdges[k] = pos;
      } else {
        warning() << "Wrong parameter SeedFinding_zBinEdges; default used" << endmsg;
        gridCfg.zBinEdges.clear();
        break;
      }
    }
  }

  Acts::CylindricalSpacePointGridOptions gridOpts;
  gridOpts.bFieldInZ = (*m_actsGeoSvc->magneticField()->getField(zeropos, magCache))[2];

  // Wrap space points for ACTS seed finder
  std::vector<const ACTSTracking::SeedSpacePoint*> spacePointPtrs(spacePoints.size(), nullptr);
  std::transform(spacePoints.begin(), spacePoints.end(), spacePointPtrs.begin(),
                 [](const ACTSTracking::SeedSpacePoint& sp) { return &sp; });

  Acts::SpacePointContainerConfig spConfig;
  spConfig.useDetailedDoubleMeasurementInfo = finderCfg.useDetailedDoubleMeasurementInfo;
  Acts::SpacePointContainerOptions spOptions;
  spOptions.beamPos = {0., 0.};

  ACTSTracking::SpacePointContainer                                       container(spacePointPtrs);
  Acts::SpacePointContainer<decltype(container), Acts::detail::RefHolder> spContainer(spConfig, spOptions, container);

  SSPointGrid grid = Acts::CylindricalSpacePointGridCreator::createGrid<SSPoint>(gridCfg, gridOpts);
  Acts::CylindricalSpacePointGridCreator::fillGrid(finderCfg, finderOpts, grid, spContainer);

  const Acts::GridBinFinder<3ul> bottomBinFinder(m_phiBottomBinLen.value(), m_zBottomBinLen.value(), 0);
  const Acts::GridBinFinder<3ul> topBinFinder(m_phiTopBinLen.value(), m_zTopBinLen.value(), 0);

  Acts::SeedFinder<SSPoint, SSPointGrid> finder(finderCfg);

  Navigator::Config navigatorCfg{m_actsGeoSvc->trackingGeometry()};
  navigatorCfg.resolvePassive   = false;
  navigatorCfg.resolveMaterial  = true;
  navigatorCfg.resolveSensitive = true;

  Stepper    stepper(m_actsGeoSvc->magneticField());
  Navigator  navigator(navigatorCfg);
  Propagator propagator(std::move(stepper), std::move(navigator));
  CKF        trackFinder(std::move(propagator));

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

  float minRange = std::numeric_limits<float>::max();
  float maxRange = std::numeric_limits<float>::lowest();
  for (const auto& coll : grid) {
    if (coll.empty())
      continue;
    minRange = std::min(coll.front()->radius(), minRange);
    maxRange = std::max(coll.back()->radius(), maxRange);
  }

  auto spacePointsGrouping = Acts::CylindricalBinnedGroup<SSPoint>(std::move(grid), bottomBinFinder, topBinFinder);

  const Acts::Range1D<float> rMiddleSPRange(std::floor(minRange / 2) * 2 + finderCfg.deltaRMiddleMinSPRange,
                                            std::floor(maxRange / 2) * 2 - finderCfg.deltaRMiddleMaxSPRange);

  using GroupIterator = decltype(spacePointsGrouping.begin());
  using GroupValue    = std::decay_t<decltype(*std::declval<GroupIterator>())>;
  std::vector<GroupValue> spacePointGroups;
  spacePointGroups.reserve(spacePointsGrouping.grid().size());
  for (auto spGroup : spacePointsGrouping) {
    spacePointGroups.push_back(spGroup);
  }

  auto parallelSeedingAndTracking = [&](const tbb::blocked_range<size_t>& r) {
    for (size_t i = r.begin(); i != r.end(); ++i) {
      const auto paramseeds = findSeeds(finder, finderOpts, spacePointGroups[i], spacePointsGrouping.grid(),
                                        rMiddleSPRange, spContainer.size(), seedCollection, magCache);
      if (!m_runCKF)
        continue;
      if (!tracking(paramseeds, trackFinder, ckfOptions, magCache, trackCollection).isSuccess()) {
        warning() << "Tracking failed for this event" << endmsg;
      }
    }
  };

  if (m_numThreads > 1) {
    arena.execute(
        [&] { tbb::parallel_for(tbb::blocked_range<size_t>(0, spacePointGroups.size()), parallelSeedingAndTracking); });
  } else {
    for (size_t i = 0; i < spacePointGroups.size(); ++i) {
      parallelSeedingAndTracking(tbb::blocked_range<size_t>(i, i + 1));
    }
  }

  debug() << "Track Collection Size: " << trackCollection.size() << endmsg;
  return std::make_tuple(std::move(seedCollection), std::move(trackCollection));
}

std::vector<Acts::BoundTrackParameters> CKFTrackingAlg::findSeeds(
    const Acts::SeedFinder<SSPoint, SSPointGrid>& finder, const Acts::SeedFinderOptions& finderOpts,
    const auto& spacePointGroup, const SSPointGrid& grid, Acts::Range1D<float> middleSpRange, std::size_t mutSpDataSize,
    edm4hep::TrackCollection& seedCollection, Acts::MagneticFieldProvider::Cache& magCache) const {
  const Acts::GeometryContext geoCtx = Acts::GeometryContext::dangerouslyDefaultConstruct();

  const auto& [bottom, middle, top] = spacePointGroup;

  std::vector<Acts::Seed<SSPoint>>                     seeds;
  std::vector<Acts::BoundTrackParameters>              paramseeds;
  Acts::SeedFinder<SSPoint, SSPointGrid>::SeedingState state;
  state.spacePointMutableData.resize(mutSpDataSize);

  finder.createSeedsForGroup(finderOpts, state, grid, seeds, bottom, middle, top, middleSpRange);

  // Unwrap proxy types back to concrete SeedSpacePoint
  std::vector<Acts::Seed<ACTSTracking::SeedSpacePoint>> f_seeds;
  f_seeds.reserve(seeds.size());
  for (const Acts::Seed<SSPoint>& seed : seeds) {
    const auto& sps = seed.sp();
    f_seeds.emplace_back(*sps[0]->externalSpacePoint(), *sps[1]->externalSpacePoint(), *sps[2]->externalSpacePoint());
  }

  for (const auto& seed : f_seeds) {
    const ACTSTracking::SeedSpacePoint* bottomSP   = seed.sp().front();
    const auto&                         sourceLink = bottomSP->sourceLink();
    const Acts::GeometryIdentifier&     geoId      = sourceLink.geometryId();
    const Acts::Surface*                surface    = m_actsGeoSvc->trackingGeometry()->findSurface(geoId);
    if (surface == nullptr) {
      warning() << "Surface with geoID " << geoId << " not found in tracking geometry" << endmsg;
      continue;
    }

    // Magnetic field at the seed position
    const Acts::Vector3         seedPos(bottomSP->x(), bottomSP->y(), bottomSP->z());
    Acts::Result<Acts::Vector3> seedField = m_actsGeoSvc->magneticField()->getField(seedPos, magCache);
    if (!seedField.ok()) {
      throw std::runtime_error("Field lookup error: " + std::to_string(seedField.error().value()));
    }

    Acts::Result<Acts::BoundVector> optParams =
        Acts::estimateTrackParamsFromSeed(geoCtx, seed.sp(), *surface, *seedField);
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

    // Compute seed track state before acquiring the lock
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
      for (const ACTSTracking::SeedSpacePoint* sp : seed.sp()) {
        seedTrack.addToTrackerHits(sp->sourceLink().edm4hepHit());
      }
      seedTrack.addToTrackStates(seedTrackState);
    }

    debug() << "Seed Parameters" << std::endl << paramseed << endmsg;
  }

  debug() << "Seeds found: " << paramseeds.size() << endmsg;
  return paramseeds;
}

StatusCode CKFTrackingAlg::tracking(const std::vector<Acts::BoundTrackParameters>& paramseeds, const CKF& trackFinder,
                                    const TrackFinderOptions& ckfOptions, Acts::MagneticFieldProvider::Cache& magCache,
                                    edm4hep::TrackCollection& trackCollection) const {
  const Acts::GeometryContext geoCtx = Acts::GeometryContext::dangerouslyDefaultConstruct();

  debug() << "Starting CKF track finding with " << paramseeds.size() << " seeds." << endmsg;

  auto           trackContainer      = std::make_shared<Acts::VectorTrackContainer>();
  auto           trackStateContainer = std::make_shared<Acts::VectorMultiTrajectory>();
  TrackContainer tracks(trackContainer, trackStateContainer);

  for (std::size_t iseed = 0; iseed < paramseeds.size(); ++iseed) {
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
