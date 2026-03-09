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

// Gaudi
#include <GaudiKernel/ITHistSvc.h>
#include <GaudiKernel/MsgStream.h>

// ACTS
#include <Acts/Seeding/SpacePointGrid.hpp>
#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>
#include <Acts/TrackFinding/MeasurementSelector.hpp>
#include <Acts/TrackFinding/TrackStateCreator.hpp>
#include <Acts/TrackFitting/GainMatrixUpdater.hpp>
#include <Acts/Utilities/RangeXD.hpp>

// TBB
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>

#include <chrono>

using namespace Acts::UnitLiterals;

DECLARE_COMPONENT(ACTSSeededCKFTrackingAlg)

// Constructor
ACTSSeededCKFTrackingAlg::ACTSSeededCKFTrackingAlg(const std::string& name, ISvcLocator* svcLoc)
    : ACTSAlgBase(name, svcLoc) {}

StatusCode ACTSSeededCKFTrackingAlg::initialize() {
  // Initialize the base
  StatusCode init = ACTSAlgBase::initialize();

  // Initialize timing histograms
  SmartIF<ITHistSvc> histSvc;
  histSvc = serviceLocator()->service("THistSvc");

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

  // Containers
  std::vector<std::pair<Acts::GeometryIdentifier, edm4hep::TrackerHitPlane>> sortedHits;
  ACTSTracking::SourceLinkContainer                                          sourceLinks;
  ACTSTracking::MeasurementContainer                                         measurements;
  ACTSTracking::SeedSpacePointContainer                                      spacePoints;

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

    ACTSTracking::SourceLink  sourceLink(surface->geometryId(), measurements.size(), hitPair.second);
    Acts::SourceLink          src_wrap{sourceLink};
    ACTSTracking::Measurement meas =
        ACTSTracking::makeMeasurement(src_wrap, loc, localCov, Acts::eBoundLoc0, Acts::eBoundLoc1);

    measurements.push_back(meas);
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

      double                 x            = globalPos[Acts::ePos0];
      double                 y            = globalPos[Acts::ePos1];
      double                 scale        = 2 / std::hypot(x, y);
      Acts::ActsMatrix<2, 3> jacXyzToRhoZ = Acts::ActsMatrix<2, 3>::Zero();
      jacXyzToRhoZ(0, Acts::ePos0)        = scale * x;
      jacXyzToRhoZ(0, Acts::ePos1)        = scale * y;
      jacXyzToRhoZ(1, Acts::ePos2)        = 1;
      // compute Jacobian from local coordinates to rho/z
      Acts::ActsMatrix<2, 2> jac = jacXyzToRhoZ * rotLocalToGlobal.block<3, 2>(Acts::ePos0, Acts::ePos0);
      // compute rho/z variance
      Acts::ActsVector<2> var = (jac * localCov * jac.transpose()).diagonal();

      // Save spacepoint
      spacePoints.push_back(ACTSTracking::SeedSpacePoint(globalPos, var[0], var[1], sourceLink));
    }
  }

  debug() << "Created " << spacePoints.size() << " space points" << endmsg;

  // Run seeding + tracking algorithms
  // Caches
  Acts::MagneticFieldContext         magFieldContext = Acts::MagneticFieldContext();
  Acts::MagneticFieldProvider::Cache magCache        = magneticField()->makeCache(magFieldContext);

  // std::unique_ptr<const Acts::Logger>
  // logger=Acts::getDefaultLogger("TrackFitting",
  // Acts::Logging::Level::VERBOSE);

  // Finder configuration
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
  finderCfg.cotThetaMax              = 7.40627;  // 2.7 eta;
  finderCfg.sigmaScattering          = m_seedFinding_sigmaScattering;
  finderCfg.radLengthPerSeed         = m_seedFinding_radLengthPerSeed;
  finderCfg.minPt                    = m_seedFinding_minPt * Acts::UnitConstants::MeV;
  finderCfg.impactMax                = m_seedFinding_impactMax * Acts::UnitConstants::mm;
  finderCfg.useVariableMiddleSPRange = true;

  Acts::SeedFilterConfig filterCfg;
  filterCfg.maxSeedsPerSpM = finderCfg.maxSeedsPerSpM;

  finderCfg.seedFilter = std::make_unique<Acts::SeedFilter<SSPoint>>(filterCfg);
  finderCfg            = finderCfg.calculateDerivedQuantities();

  Acts::SeedFinderOptions finderOpts;
  finderOpts.bFieldInZ = (*magneticField()->getField(zeropos, magCache))[2];
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
  if (m_seedFinding_zBinEdges.size() > 0) {
    gridCfg.zBinEdges.resize(m_seedFinding_zBinEdges.size());
    for (size_t k = 0; k < m_seedFinding_zBinEdges.size(); k++) {
      float pos = std::atof(m_seedFinding_zBinEdges[k].c_str());
      if (pos >= finderCfg.zMin && pos < finderCfg.zMax) {
        gridCfg.zBinEdges[k] = pos;
      } else {
        warning() << "Wrong parameter SeedFinding_zBinEdges; "
                  << "default used" << endmsg;
        gridCfg.zBinEdges.clear();
        break;
      }
    }
  }

  Acts::CylindricalSpacePointGridOptions gridOpts;
  gridOpts.bFieldInZ = (*magneticField()->getField(zeropos, magCache))[2];

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

  // Set the options
  Acts::MeasurementSelector::Config measurementSelectorCfg = {
      {Acts::GeometryIdentifier(), {{}, {m_CKF_chi2CutOff}, {(std::size_t)(m_CKF_numMeasurementsCutOff)}}}};

  Acts::PropagatorPlainOptions pOptions{geometryContext(), magneticFieldContext()};
  pOptions.maxSteps = 10000;
  if (m_propagateBackward) {
    pOptions.direction = Acts::Direction::Backward();
  }

  // Construct a perigee surface as the target surface
  std::shared_ptr<Acts::PerigeeSurface> perigeeSurface =
      Acts::Surface::makeShared<Acts::PerigeeSurface>(Acts::Vector3{0., 0., 0.});

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

  float minRange = std::numeric_limits<float>::max();
  float maxRange = std::numeric_limits<float>::lowest();
  for (const auto& coll : grid) {
    if (coll.empty())
      continue;

    const auto* firstEl = coll.front();
    const auto* lastEl  = coll.back();
    minRange            = std::min(firstEl->radius(), minRange);
    maxRange            = std::max(lastEl->radius(), maxRange);
  }

  auto spacePointsGrouping = Acts::CylindricalBinnedGroup<SSPoint>(std::move(grid), bottomBinFinder, topBinFinder);

  const Acts::Range1D<float> rMiddleSPRange(std::floor(minRange / 2) * 2 + finderCfg.deltaRMiddleMinSPRange,
                                            std::floor(maxRange / 2) * 2 - finderCfg.deltaRMiddleMaxSPRange);

  // Convert the binned group to a vector for access
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

      // Find the tracks
      if (!m_runCKF)
        continue;

      if (!tracking(paramseeds, trackFinder, ckfOptions, magCache, trackCollection).isSuccess()) {
        warning() << "Tracking failed for this event" << endmsg;
      }
    }
  };  // parallelSeedingAndTracking

  // Run in parallel if more than one thread is requested
  if (m_numThreads > 1) {
    arena.execute(
        [&] { tbb::parallel_for(tbb::blocked_range<size_t>(0, spacePointGroups.size()), parallelSeedingAndTracking); });
  } else {  // Serial execution
    for (size_t i = 0; i < spacePointGroups.size(); ++i) {
      parallelSeedingAndTracking(tbb::blocked_range<size_t>(i, i + 1));
    }
  }

  debug() << "Track Collection Size: " << trackCollection.size() << endmsg;

  return std::make_tuple(std::move(seedCollection), std::move(trackCollection));
}

// CKF tracking,
StatusCode ACTSSeededCKFTrackingAlg::tracking(const std::vector<Acts::BoundTrackParameters>& paramseeds,
                                              const CKF& trackFinder, const TrackFinderOptions& ckfOptions,
                                              Acts::MagneticFieldProvider::Cache& magCache,
                                              edm4hep::TrackCollection&           trackCollection) const {
  // Initialize track finder
  warning() << "Starting CKF track finding with " << paramseeds.size() << " seeds." << endmsg;

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

        // Helpful debug output
        debug() << "Trajectory Summary" << endmsg;
        debug() << "\tchi2Sum       " << trackTip.chi2() << endmsg;
        debug() << "\tNDF           " << trackTip.nDoF() << endmsg;
        debug() << "\tnHoles        " << trackTip.nHoles() << endmsg;
        debug() << "\tnMeasurements " << trackTip.nMeasurements() << endmsg;
        debug() << "\tnOutliers     " << trackTip.nOutliers() << endmsg;
        debug() << "\tnStates       " << trackTip.nTrackStates() << endmsg;

        // Make track object
        auto track = ACTSTracking::ACTS2edm4hep_track(trackTip, magneticField(), magCache);

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
