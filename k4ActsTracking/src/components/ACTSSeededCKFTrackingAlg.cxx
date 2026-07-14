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
#include "k4ActsTracking/ACTSSeededCKFTrackingAlg.hxx"

// ACTSTracking
#include "k4ActsTracking/MeasurementCalibrator.hxx"

// edm4hep
#include <edm4hep/MCParticle.h>
#include <edm4hep/MutableTrack.h>
#include <edm4hep/SimTrackerHit.h>
#include <edm4hep/TrackState.h>
#include <edm4hep/TrackerHitPlane.h>

// ACTS
#include <Acts/Definitions/Units.hpp>
#include <Acts/Seeding/BroadTripletSeedFilter.hpp>
#include <Acts/Seeding/CylindricalSpacePointGrid.hpp>
#include <Acts/Seeding/DoubletSeedFinder.hpp>
#include <Acts/Seeding/EstimateTrackParamsFromSeed.hpp>
#include <Acts/Seeding/TripletSeedFinder.hpp>
#include <Acts/Seeding/TripletSeeder.hpp>
#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>
#include <Acts/TrackFinding/MeasurementSelector.hpp>
#include <Acts/TrackFinding/TrackStateCreator.hpp>
#include <Acts/TrackFitting/GainMatrixUpdater.hpp>
#include <Acts/Utilities/Logger.hpp>
#include <Acts/Utilities/RangeXD.hpp>
#include <Acts/Utilities/TrackHelpers.hpp>

// TBB
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>

#include <chrono>
#include <span>

using namespace Acts::UnitLiterals;

DECLARE_COMPONENT(ACTSSeededCKFTrackingAlg)

// Constructor
ACTSSeededCKFTrackingAlg::ACTSSeededCKFTrackingAlg(const std::string& name, ISvcLocator* svcLoc)
    : ACTSAlgBase(name, svcLoc) {}

StatusCode ACTSSeededCKFTrackingAlg::initialize() {
  // Initialize the base
  StatusCode init = ACTSAlgBase::initialize();

  // Initialize seeding layers
  std::vector<std::string> seedingLayers;
  std::copy_if(m_seedingLayers.begin(), m_seedingLayers.end(), std::back_inserter(seedingLayers),
               [](const std::string& s) { return !s.empty(); });

  if (seedingLayers.size() % 2 != 0) {
    throw std::runtime_error("SeedingLayers needs an even number of entries");
  }

  std::vector<Acts::GeometryIdentifier> geoSelection;
  for (uint32_t i = 0; i < seedingLayers.size(); i += 2) {
    Acts::GeometryIdentifier geoid;
    if (m_seedingLayers[i + 0] != "*")  // volume
      geoid = geoid.withVolume(std::stoi(m_seedingLayers[i + 0]));
    if (m_seedingLayers[i + 1] != "*")  // layer
      geoid = geoid.withLayer(std::stoi(m_seedingLayers[i + 1]));

    geoSelection.push_back(geoid);
  }

  m_seedGeometrySelection = ACTSTracking::GeometryIdSelector(geoSelection);

  if (m_seedFinding_deltaRMinTop == 0.f)
    m_seedFinding_deltaRMinTop = m_seedFinding_deltaRMin;
  if (m_seedFinding_deltaRMaxTop == 0.f)
    m_seedFinding_deltaRMaxTop = m_seedFinding_deltaRMax;
  if (m_seedFinding_deltaRMinBottom == 0.f)
    m_seedFinding_deltaRMinBottom = m_seedFinding_deltaRMin;
  if (m_seedFinding_deltaRMaxBottom == 0.f)
    m_seedFinding_deltaRMaxBottom = m_seedFinding_deltaRMax;

  debug() << "Initialization of ACTSSeededCKFTrackingAlg successful" << endmsg;
  return init;
}

std::tuple<edm4hep::TrackCollection, edm4hep::TrackCollection> ACTSSeededCKFTrackingAlg::operator()(
    const edm4hep::TrackerHitPlaneCollection& trackerHitCollection) const {
  // Prepare output collections
  edm4hep::TrackCollection seedCollection;
  edm4hep::TrackCollection trackCollection;

  // Minimal per-hit information needed to build the seeding space points. The
  // global position and rho/z variances are computed during the hit loop and
  // later transferred (in grid-bin order) into an Acts::SpacePointContainer2.
  struct SeedInput {
    float                    x, y, z, r, phi;
    float                    varR, varZ;
    ACTSTracking::SourceLink sourceLink;
  };

  // Containers
  std::vector<std::pair<Acts::GeometryIdentifier, edm4hep::TrackerHitPlane>> sortedHits;
  ACTSTracking::SourceLinkContainer                                          sourceLinks;
  ACTSTracking::MeasurementContainer                                         measurements;
  ACTSTracking::HitContainer                                                 hits;
  std::vector<SeedInput>                                                     seedInputs;

  // Loop over each hit collections and get a single vector with hits
  // from all of the subdetectors. Also include the Acts GeoId in
  // the vector. It will be important for the sort to speed up the
  // population of the final SourceLink multiset.
  sortedHits.reserve(trackerHitCollection.size());

  for (const auto& hit : trackerHitCollection) {
    sortedHits.push_back(std::make_pair(geoIDMappingTool()->getGeometryID(hit), hit));
  }
  debug() << "Working with " << sortedHits.size() << " hits." << endmsg;

  // Sort by GeoID
  auto compare = [](const auto& a, const auto& b) { return a.first < b.first; };

  tbb::task_arena arena(m_numThreads.value());

  if (m_numThreads > 1) {
    arena.execute([&] { tbb::parallel_sort(sortedHits.begin(), sortedHits.end(), compare); });
  } else {
    std::sort(sortedHits.begin(), sortedHits.end(), compare);
  }

  // Turn the edm4hep TrackerHit's into Acts objects
  // Assumes that the hits are sorted by the GeoID
  sourceLinks.reserve(sortedHits.size());
  hits.reserve(sortedHits.size());
  for (auto& hitPair : sortedHits) {
    // Convert to Acts hit
    const Acts::Surface* surface = trackingGeometry()->findSurface(hitPair.first);
    if (surface == nullptr)
      throw std::runtime_error("Surface not found");

    const edm4hep::Vector3d& edmglobalpos = hitPair.second.getPosition();
    Acts::Vector3            globalPos    = {edmglobalpos.x, edmglobalpos.y, edmglobalpos.z};

    Acts::Result<Acts::Vector2> lpResult = surface->globalToLocal(geometryContext(), globalPos, {0, 0, 0}, 0.5_um);
    if (!lpResult.ok())
      throw std::runtime_error("Global to local transformation did not succeed.");

    Acts::Vector2 loc = lpResult.value();

    Acts::SquareMatrix2            localCov = Acts::SquareMatrix2::Zero();
    const edm4hep::TrackerHitPlane hitplane = hitPair.second;
    localCov(0, 0)                          = std::pow(hitplane.getDu() * Acts::UnitConstants::mm, 2);
    localCov(1, 1)                          = std::pow(hitplane.getDv() * Acts::UnitConstants::mm, 2);

    ACTSTracking::SourceLink  sourceLink(surface->geometryId(), measurements.size());
    Acts::SourceLink          src_wrap{sourceLink};
    ACTSTracking::Measurement meas =
        ACTSTracking::makeMeasurement(src_wrap, loc, localCov, Acts::eBoundLoc0, Acts::eBoundLoc1);

    measurements.push_back(meas);
    hits.push_back(hitPair.second);
    sourceLinks.emplace_hint(sourceLinks.end(), sourceLink);

    // Seed selection and conversion to useful coordinates
    if (m_seedGeometrySelection.check(surface->geometryId())) {
      Acts::RotationMatrix3 rotLocalToGlobal = surface->referenceFrame(geometryContext(), globalPos, {0, 0, 0});

      // Convert to a seed space point
      // the space point requires only the variance of the transverse and
      // longitudinal position. reduce computations by transforming the
      // covariance directly from local to rho/z.
      //
      // compute Jacobian from global coordinates to rho/z
      //
      //         rho = sqrt(x² + y²)
      // drho/d{x,y} = (1 / sqrt(x² + y²)) * 2 * {x,y}
      //             = 2 * {x,y} / r
      //       dz/dz = 1 (duuh!)

      double             x            = globalPos[Acts::ePos0];
      double             y            = globalPos[Acts::ePos1];
      double             scale        = 2 / std::hypot(x, y);
      Acts::Matrix<2, 3> jacXyzToRhoZ = Acts::Matrix<2, 3>::Zero();
      jacXyzToRhoZ(0, Acts::ePos0)    = scale * x;
      jacXyzToRhoZ(0, Acts::ePos1)    = scale * y;
      jacXyzToRhoZ(1, Acts::ePos2)    = 1;
      // compute Jacobian from local coordinates to rho/z
      const auto jac = jacXyzToRhoZ * rotLocalToGlobal.block<3, 2>(Acts::ePos0, Acts::ePos0);
      // compute rho/z variance
      const auto var = (jac * localCov * jac.transpose()).diagonal();

      // Save spacepoint
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

  debug() << "Created " << seedInputs.size() << " space points" << endmsg;

  // Run seeding + tracking algorithms
  // Caches
  Acts::MagneticFieldContext         magFieldContext = Acts::MagneticFieldContext();
  Acts::MagneticFieldProvider::Cache magCache        = magneticField()->makeCache(magFieldContext);

  // std::unique_ptr<const Acts::Logger>
  // logger=Acts::getDefaultLogger("TrackFitting",
  // Acts::Logging::Level::VERBOSE);

  // Finder configuration
  static const Acts::Vector3 zeropos(0, 0, 0);

  const float bFieldInZ       = (*magneticField()->getField(zeropos, magCache))[2];
  const float cotThetaMax     = 7.40627f;  // ~2.7 η
  const float minPt           = m_seedFinding_minPt * Acts::UnitConstants::MeV;
  const float impactMax       = m_seedFinding_impactMax * Acts::UnitConstants::mm;
  const float collisionRegion = m_seedFinding_collisionRegion;

  // -------------------------------------------------------------------------
  // Seeding grid (SoA): bin the seed space points in (phi, z, r).
  // -------------------------------------------------------------------------
  Acts::CylindricalSpacePointGrid::Config gridCfg;
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

  Acts::CylindricalSpacePointGrid grid(gridCfg, Acts::getDefaultLogger("SeededCKFSeedingGrid", Acts::Logging::WARNING));

  for (std::size_t i = 0; i < seedInputs.size(); ++i) {
    const SeedInput& sp = seedInputs[i];
    grid.insert(static_cast<Acts::SpacePointIndex>(i), sp.phi, sp.z, sp.r);
  }

  // Sort each bin by radius, as required by the radius-sorted doublet finders.
  for (std::size_t i = 0; i < grid.numberOfBins(); ++i) {
    std::ranges::sort(grid.at(i), [&](const Acts::SpacePointIndex& a, const Acts::SpacePointIndex& b) {
      return seedInputs[a].r < seedInputs[b].r;
    });
  }

  // -------------------------------------------------------------------------
  // Build the SoA space point container in grid-bin order so that every bin
  // maps to a contiguous index range, as expected by the triplet seeder.
  // -------------------------------------------------------------------------
  Acts::SpacePointContainer spacePoints(Acts::SpacePointColumns::SourceLinks | Acts::SpacePointColumns::PackedXY |
                                        Acts::SpacePointColumns::PackedZR | Acts::SpacePointColumns::VarianceZ |
                                        Acts::SpacePointColumns::VarianceR);
  spacePoints.reserve(grid.numberOfSpacePoints());
  std::vector<Acts::SpacePointIndexRange> gridSpacePointRanges;
  gridSpacePointRanges.reserve(grid.numberOfBins());
  for (std::size_t i = 0; i < grid.numberOfBins(); ++i) {
    std::uint32_t begin = spacePoints.size();
    for (Acts::SpacePointIndex spIndex : grid.at(i)) {
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
  for (const Acts::SpacePointIndexRange& range : gridSpacePointRanges) {
    if (range.first == range.second)
      continue;
    minRange = std::min(spacePoints[range.first].zr()[1], minRange);
    maxRange = std::max(spacePoints[range.second - 1].zr()[1], maxRange);
  }

  // Variable middle space point radial region of interest.
  constexpr float               deltaRMiddleMinSPRange = 10.f * Acts::UnitConstants::mm;
  constexpr float               deltaRMiddleMaxSPRange = 10.f * Acts::UnitConstants::mm;
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
  topFinderCfg.candidateDirection              = Acts::Direction::Forward();
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

  auto seedingLogger = Acts::getDefaultLogger("SeededCKFSeeding", Acts::Logging::WARNING);

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

  // Configurations
  Navigator::Config navigatorCfg{trackingGeometry()};
  navigatorCfg.resolvePassive   = false;
  navigatorCfg.resolveMaterial  = true;
  navigatorCfg.resolveSensitive = true;

  // construct all components for the fitter
  Stepper    stepper(magneticField());
  Navigator  navigator(navigatorCfg);
  Propagator propagator(std::move(stepper), std::move(navigator));
  CKF        trackFinder(std::move(propagator));

  // For extrapolating the fitted track back to the IP (perigee surface)
  Stepper    extrapStepper(magneticField());
  Navigator  extrapNavigator(navigatorCfg);
  Propagator extrapPropagator(std::move(extrapStepper), std::move(extrapNavigator));
  auto       perigeeSurface = Acts::Surface::makeShared<Acts::PerigeeSurface>(Acts::Vector3{0., 0., 0.});

  // Set the options
  Acts::MeasurementSelector::Config measurementSelectorCfg = {
      {Acts::GeometryIdentifier(), {{}, {m_CKF_chi2CutOff}, {(std::size_t)(m_CKF_numMeasurementsCutOff)}}}};

  Acts::PropagatorPlainOptions pOptions{geometryContext(), magneticFieldContext()};
  pOptions.maxSteps = 10000;
  if (m_propagateBackward) {
    pOptions.direction = Acts::Direction::Backward();
  }

  Propagator::Options<> extrapOptions{geometryContext(), magneticFieldContext()};
  extrapOptions.maxSteps = 10000;
  if (m_propagateBackward) {
    extrapOptions.direction = Acts::Direction::Backward();
  }

  Acts::GainMatrixUpdater kfUpdater;

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

  TrackFinderOptions ckfOptions =
      TrackFinderOptions(geometryContext(), magneticFieldContext(), calibrationContext(), extensions, pOptions);

  // -------------------------------------------------------------------------
  // Seeding and CKF track finding, parallelised over the grid groups. Each
  // task owns its seeder cache, seed filter (with its own state/cache), seed
  // container and magnetic-field cache; the shared edm4hep collections are
  // guarded by mutexes (m_seedMutex / m_trackMutex).
  // -------------------------------------------------------------------------
  auto parallelSeedingAndTracking = [&](const tbb::blocked_range<size_t>& r) {
    // The magnetic-field cache is mutated on every field lookup, so each
    // parallel invocation needs its own cache rather than sharing one.
    Acts::MagneticFieldProvider::Cache localMagCache = magneticField()->makeCache(magFieldContext);

    Acts::TripletSeeder::Cache          seederCache;
    Acts::BroadTripletSeedFilter::State filterState;
    Acts::BroadTripletSeedFilter::Cache filterCache;
    Acts::BroadTripletSeedFilter        seedFilter(filterCfg, filterState, filterCache, *seedingLogger);

    Acts::SeedContainer seeds;
    seeds.assignSpacePointContainer(spacePoints);

    std::vector<Acts::SpacePointContainer::ConstRange> bottomSpRanges;
    std::vector<Acts::SpacePointContainer::ConstRange> topSpRanges;

    for (size_t i = r.begin(); i != r.end(); ++i) {
      const auto& [bottom, middle, top] = groups[i];

      Acts::SpacePointContainer::ConstRange middleSpRange =
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
        seedsToParameters(seeds, spacePoints, hits, seedCollection, localMagCache);

    // Find the tracks
    if (!m_runCKF)
      return;

    if (!tracking(paramseeds, trackFinder, ckfOptions, extrapPropagator, *perigeeSurface, extrapOptions, hits,
                  localMagCache, trackCollection)
             .isSuccess()) {
      warning() << "Tracking failed for this event" << endmsg;
    }
  };  // parallelSeedingAndTracking

  // Run in parallel if more than one thread is requested
  if (m_numThreads > 1) {
    arena.execute([&] { tbb::parallel_for(tbb::blocked_range<size_t>(0, groups.size()), parallelSeedingAndTracking); });
  } else {  // Serial execution
    parallelSeedingAndTracking(tbb::blocked_range<size_t>(0, groups.size()));
  }

  debug() << "Track Collection Size: " << trackCollection.size() << endmsg;

  return std::make_tuple(std::move(seedCollection), std::move(trackCollection));
}

std::vector<Acts::BoundTrackParameters> ACTSSeededCKFTrackingAlg::seedsToParameters(
    const Acts::SeedContainer& seeds, const Acts::SpacePointContainer& spacePoints,
    const ACTSTracking::HitContainer& hits, edm4hep::TrackCollection& seedCollection,
    Acts::MagneticFieldProvider::Cache& magCache) const {
  std::vector<Acts::BoundTrackParameters> paramseeds;
  paramseeds.reserve(seeds.size());

  auto position = [](const Acts::ConstSpacePointProxy& sp) {
    return Acts::Vector3(sp.xy()[0], sp.xy()[1], sp.zr()[0]);
  };
  auto sourceLinkOf = [](const Acts::ConstSpacePointProxy& sp) -> const ACTSTracking::SourceLink& {
    return sp.sourceLinks()[0].get<ACTSTracking::SourceLink>();
  };

  for (const Acts::ConstSeedProxy& seed : seeds) {
    const std::span<const Acts::SpacePointIndex> spIndices = seed.spacePointIndices();
    if (spIndices.size() != 3) {
      continue;
    }

    const Acts::ConstSpacePointProxy bottomSp = spacePoints[spIndices[0]];
    const Acts::ConstSpacePointProxy middleSp = spacePoints[spIndices[1]];
    const Acts::ConstSpacePointProxy topSp    = spacePoints[spIndices[2]];

    const ACTSTracking::SourceLink& bottomSL = sourceLinkOf(bottomSp);
    const Acts::GeometryIdentifier  geoId    = bottomSL.geometryId();
    const Acts::Surface*            surface  = trackingGeometry()->findSurface(geoId);
    if (surface == nullptr) {
      warning() << "surface with geoID " << geoId << " is not found in the tracking gemetry" << endmsg;
      continue;
    }

    const Acts::Vector3 bottomPos = position(bottomSp);
    const Acts::Vector3 middlePos = position(middleSp);
    const Acts::Vector3 topPos    = position(topSp);
    const double        t0        = hits[bottomSL.index()].getTime();

    // Get the magnetic field at the bottom space point
    Acts::Result<Acts::Vector3> seedField = magneticField()->getField(bottomPos, magCache);
    if (!seedField.ok()) {
      throw std::runtime_error("Field lookup error: " + seedField.error().message());
    }

    Acts::Result<Acts::BoundVector> optParams =
        Acts::estimateTrackParamsFromSeed(geometryContext(), *surface, bottomPos, t0, middlePos, topPos, *seedField);
    if (!optParams.ok()) {
      debug() << "Failed estimation of track parameters for seed." << endmsg;
      continue;
    }

    const Acts::BoundVector& params = *optParams;

    float p = std::abs(1 / params[Acts::eBoundQOverP]);

    // build the track covariance matrix using the smearing sigmas
    Acts::BoundMatrix cov                       = Acts::BoundMatrix::Zero();
    cov(Acts::eBoundLoc0, Acts::eBoundLoc0)     = std::pow(m_initialTrackError_pos, 2);
    cov(Acts::eBoundLoc1, Acts::eBoundLoc1)     = std::pow(m_initialTrackError_pos, 2);
    cov(Acts::eBoundTime, Acts::eBoundTime)     = std::pow(m_initialTrackError_time, 2);
    cov(Acts::eBoundPhi, Acts::eBoundPhi)       = std::pow(m_initialTrackError_phi, 2);
    cov(Acts::eBoundTheta, Acts::eBoundTheta)   = std::pow(m_initialTrackError_lambda, 2);
    cov(Acts::eBoundQOverP, Acts::eBoundQOverP) = std::pow(m_initialTrackError_relP * p / (p * p), 2);

    Acts::BoundTrackParameters paramseed(surface->getSharedPtr(), params, cov, Acts::ParticleHypothesis::pion());
    paramseeds.push_back(paramseed);

    // Compute seed state before acquiring the lock
    Acts::Vector3 globalPos =
        surface->localToGlobal(geometryContext(), {params[Acts::eBoundLoc0], params[Acts::eBoundLoc1]}, {0, 0, 0});

    Acts::Result<Acts::Vector3> hitField = magneticField()->getField(globalPos, magCache);
    if (!hitField.ok()) {
      throw std::runtime_error("Field lookup error: " + hitField.error().message());
    }

    auto seedTrackState = ACTSTracking::ACTS2edm4hep_trackState(edm4hep::TrackState::AtFirstHit, geometryContext(),
                                                                paramseed, (*hitField)[2] / Acts::UnitConstants::T);

    // Add seed to collection, all building of seed under the lock
    {
      std::lock_guard<std::mutex> lock(m_seedMutex);
      auto                        seedTrack = seedCollection.create();
      seedTrack.addToTrackerHits(hits[bottomSL.index()]);
      seedTrack.addToTrackerHits(hits[sourceLinkOf(middleSp).index()]);
      seedTrack.addToTrackerHits(hits[sourceLinkOf(topSp).index()]);
      seedTrack.addToTrackStates(seedTrackState);
    }

    debug() << "Seed Parameters" << std::endl << paramseed << endmsg;
  }

  debug() << "Seeds found: " << std::endl << paramseeds.size() << endmsg;

  return paramseeds;
}

// CKF tracking,
StatusCode ACTSSeededCKFTrackingAlg::tracking(const std::vector<Acts::BoundTrackParameters>& paramseeds,
                                              const CKF& trackFinder, const TrackFinderOptions& ckfOptions,
                                              const Propagator&                   extrapPropagator,
                                              const Acts::PerigeeSurface&         perigeeSurface,
                                              Propagator::Options<>&              extrapOptions,
                                              const ACTSTracking::HitContainer&   hits,
                                              Acts::MagneticFieldProvider::Cache& magCache,
                                              edm4hep::TrackCollection&           trackCollection) const {
  // Initialize track finder
  debug() << "Starting CKF track finding with " << paramseeds.size() << " seeds." << endmsg;

  auto           trackContainer      = std::make_shared<Acts::VectorTrackContainer>();
  auto           trackStateContainer = std::make_shared<Acts::VectorMultiTrajectory>();
  TrackContainer tracks(trackContainer, trackStateContainer);

  auto trackStart = std::chrono::high_resolution_clock::now();

  for (std::size_t iseed = 0; iseed < paramseeds.size(); ++iseed) {
    tracks.clear();

    auto result = trackFinder.findTracks(paramseeds.at(iseed), ckfOptions, tracks);
    if (result.ok()) {
      const auto& fitOutput = result.value();
      for (const TrackContainer::TrackProxy& trackItem : fitOutput) {
        // Track smoothing
        auto trackTip = tracks.makeTrack();
        trackTip.copyFrom(trackItem);
        auto smoothResult = Acts::smoothTrack(geometryContext(), trackTip);
        if (!smoothResult.ok()) {
          warning() << "Track smoothing error: " << smoothResult.error() << endmsg;
          continue;
        }

        // Extrapolate the fitted track back to the perigee surface at the IP so
        // that the track parameters (in particular D0 and Z0) are expressed
        // there. This reproduces the TrackStateAtIP behaviour from older ACTS.
        auto extrapResult = Acts::extrapolateTrackToReferenceSurface(
            trackTip, perigeeSurface, extrapPropagator, extrapOptions, Acts::TrackExtrapolationStrategy::firstOrLast);
        if (!extrapResult.ok()) {
          warning() << "Track extrapolation to perigee failed: " << extrapResult.error() << endmsg;
          continue;
        }

        // Helpful debug output
        debug() << "Trajectory Summary" << endmsg;
        debug() << "\tchi2Sum       " << trackTip.chi2() << endmsg;
        debug() << "\tNDF           " << trackTip.nDoF() << endmsg;
        debug() << "\tnHoles        " << trackTip.nHoles() << endmsg;
        debug() << "\tnMeasurements " << trackTip.nMeasurements() << endmsg;
        debug() << "\tnOutliers     " << trackTip.nOutliers() << endmsg;
        debug() << "\tnStates       " << trackTip.nTrackStates() << endmsg;

        // Make track object
        auto track = ACTSTracking::ACTS2edm4hep_track(geometryContext(), trackTip, hits, magneticField(), magCache);

        // Save results
        {
          std::lock_guard lock{m_trackMutex};
          trackCollection.push_back(track);
        }
      }
    } else {
      warning() << "Track fit error: " << result.error() << endmsg;
    }
  }

  auto                          trackEnd      = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> trackDuration = trackEnd - trackStart;
  //m_histTrackBuild->Fill(trackDuration.count());

  return StatusCode::SUCCESS;
}
