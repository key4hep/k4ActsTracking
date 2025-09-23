#include "ACTSSeededCKFTrackingAlg.hxx"

// edm4hep
#include <edm4hep/MCParticle.h>
#include <edm4hep/SimTrackerHit.h>
#include <edm4hep/TrackerHitPlane.h>
#include <edm4hep/TrackState.h>
#include <edm4hep/MutableTrack.h>

// Gaudi
#include <GaudiKernel/MsgStream.h>
#include <GaudiKernel/ITHistSvc.h>

// ACTS
#include <Acts/EventData/MultiTrajectory.hpp>
#include <Acts/Propagator/EigenStepper.hpp>
#include <Acts/Propagator/Navigator.hpp>
#include <Acts/Propagator/Propagator.hpp>
#include <Acts/Seeding/EstimateTrackParamsFromSeed.hpp>
#include <Acts/Seeding/SeedFinder.hpp>
#include <Acts/Seeding/SpacePointGrid.hpp>
#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>
#include <Acts/TrackFinding/MeasurementSelector.hpp>
#include <Acts/TrackFitting/GainMatrixSmoother.hpp>
#include <Acts/TrackFitting/GainMatrixUpdater.hpp>

//using namespace Acts::UnitLiterals;

// ACTSTracking
#include "Helpers.hxx"
#include "MeasurementCalibrator.hxx"
#include "SeedSpacePoint.hxx"
#include "SourceLink.hxx"

// Standard
#include <chrono>

// Track fitting definitions
using TrackFinderOptions = Acts::CombinatorialKalmanFilterOptions<
	ACTSTracking::SourceLinkAccessor::Iterator, 
	Acts::VectorMultiTrajectory>;
using SSPoint = ACTSTracking::SeedSpacePoint;

DECLARE_COMPONENT(ACTSSeededCKFTrackingAlg)

// Constructor
ACTSSeededCKFTrackingAlg::ACTSSeededCKFTrackingAlg(const std::string& name, ISvcLocator* svcLoc) 
	: ACTSAlgBase(name, svcLoc) {}

StatusCode ACTSSeededCKFTrackingAlg::initialize() {
	info() << "Initializing ACTSSeededCKFTrackingAlg" << endmsg;
	// Initialize the base
	StatusCode init = ACTSAlgBase::initialize();

	// Initialize timing histograms
	SmartIF<ITHistSvc> histSvc;
	histSvc = serviceLocator()->service("THistSvc");

	m_histHitSetUp = new TH1F("TrackhitTime", "Time to Set Hits;Time", 20, 0, 10);
    m_histEntireReco = new TH1F("RecoTime", "Time for Entire Reco;Time", 20, 0 , 20);
    m_histSeedFinding = new TH1F("SeedTime", "Time to Find Seeds;Time", 20, 0, 10);
    m_histTrackBuild = new TH1F("TrackTime", "Time to Build Tracks;Time", 20, 0, 10);

	(void)histSvc->regHist("/histos/timing/track_hits", m_histHitSetUp);
	(void)histSvc->regHist("/histos/timing/entire_reco", m_histEntireReco);
	(void)histSvc->regHist("/histos/timing/seed_finding", m_histSeedFinding);
	(void)histSvc->regHist("/histos/timing/track_building", m_histTrackBuild);

	// Initialize seeding layers
	std::vector<std::string> seedingLayers;
	std::copy_if(m_seedingLayers.begin(), m_seedingLayers.end(), std::back_inserter(seedingLayers),
	[](const std::string &s) { return !s.empty(); });

	if (seedingLayers.size() % 2 != 0) {
		throw std::runtime_error("SeedingLayers needs an even number of entries");
	}

	std::vector<Acts::GeometryIdentifier> geoSelection;
	for (uint32_t i = 0; i < seedingLayers.size(); i += 2) {
		Acts::GeometryIdentifier geoid;
		if (m_seedingLayers[i + 0] != "*")  // volume
			geoid = geoid.setVolume(std::stoi(m_seedingLayers[i + 0]));
		if (m_seedingLayers[i + 1] != "*")  // layer
			geoid = geoid.setLayer(std::stoi(m_seedingLayers[i + 1]));

		geoSelection.push_back(geoid);
	}

	m_seedGeometrySelection = ACTSTracking::GeometryIdSelector(geoSelection);

	if (m_seedFinding_deltaRMinTop == 0.f) m_seedFinding_deltaRMinTop = m_seedFinding_deltaRMin;
	if (m_seedFinding_deltaRMaxTop == 0.f) m_seedFinding_deltaRMaxTop = m_seedFinding_deltaRMax;
	if (m_seedFinding_deltaRMinBottom == 0.f) m_seedFinding_deltaRMinBottom = m_seedFinding_deltaRMin;
	if (m_seedFinding_deltaRMaxBottom == 0.f) m_seedFinding_deltaRMaxBottom = m_seedFinding_deltaRMax;
	
	return init;
}

std::tuple<edm4hep::TrackCollection,
           edm4hep::TrackCollection> ACTSSeededCKFTrackingAlg::operator()(const edm4hep::TrackerHitPlaneCollection& trackerHitCollection) const{
	// Prepare output collections
	edm4hep::TrackCollection seedCollection;
	edm4hep::TrackCollection trackCollection;
	
	// Loop over each hit collections and get a single vector with hits
	// from all of the subdetectors. Also include the Acts GeoId in
	// the vector. It will be important for the sort to speed up the
	// population of the final SourceLink multiset.
	std::vector<std::pair<Acts::GeometryIdentifier, edm4hep::TrackerHitPlane>> sortedHits;

	for (const auto& hit : trackerHitCollection) {
		sortedHits.push_back(std::make_pair(geoIDMappingTool()->getGeometryID(hit), hit));
	}
	debug() << "Working with " << sortedHits.size() << " hits." <<endmsg;

	// Sort by GeoID
	std::sort(
		sortedHits.begin(), sortedHits.end(),
		[](const std::pair<Acts::GeometryIdentifier, edm4hep::TrackerHitPlane>& hit0,
		   const std::pair<Acts::GeometryIdentifier, edm4hep::TrackerHitPlane>& hit1) -> bool { 
			return hit0.first < hit1.first; 
		});

	// Turn the edm4hep TrackerHit's into Acts objects
	// Assumes that the hits are sorted by the GeoID
	ACTSTracking::SourceLinkContainer sourceLinks;
	ACTSTracking::MeasurementContainer measurements;
	ACTSTracking::SeedSpacePointContainer spacePoints;

	auto hitStart = std::chrono::high_resolution_clock::now();
	sourceLinks.reserve(sortedHits.size());
	for (auto& hitPair : sortedHits) {
		// Convert to Acts hit
		const Acts::Surface* surface = trackingGeometry()->findSurface(hitPair.first);
		if (surface == nullptr) throw std::runtime_error("Surface not found");
		
		const edm4hep::Vector3d& edmglobalpos = hitPair.second.getPosition();
		Acts::Vector3 globalPos = {edmglobalpos.x, edmglobalpos.y, edmglobalpos.z};

		Acts::Result<Acts::Vector2> lpResult = surface->globalToLocal(geometryContext(), globalPos, {0, 0, 0}, 0.5_um);
		if (!lpResult.ok()) throw std::runtime_error("Global to local transformation did not succeed.");

		Acts::Vector2 loc = lpResult.value();

		Acts::SquareMatrix2 localCov = Acts::SquareMatrix2::Zero();
		const edm4hep::TrackerHitPlane hitplane = hitPair.second;
		if (&hitplane) {
			localCov(0, 0) = std::pow(hitplane.getDu() * Acts::UnitConstants::mm, 2);
			localCov(1, 1) = std::pow(hitplane.getDv() * Acts::UnitConstants::mm, 2);
		} else {
			throw std::runtime_error("Currently only support TrackerHitPlane.");
		}
		
		ACTSTracking::SourceLink sourceLink(surface->geometryId(), measurements.size(), &hitPair.second);
		Acts::SourceLink src_wrap { sourceLink };
		Acts::Measurement meas = Acts::makeMeasurement(src_wrap, loc, localCov, Acts::eBoundLoc0, Acts::eBoundLoc1);

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

			double x = globalPos[Acts::ePos0];
			double y = globalPos[Acts::ePos1];
			double scale = 2 / std::hypot(x, y);
			Acts::ActsMatrix<2, 3> jacXyzToRhoZ = Acts::ActsMatrix<2, 3>::Zero();
			jacXyzToRhoZ(0, Acts::ePos0) = scale * x;
			jacXyzToRhoZ(0, Acts::ePos1) = scale * y;
			jacXyzToRhoZ(1, Acts::ePos2) = 1;
			// compute Jacobian from local coordinates to rho/z
			Acts::ActsMatrix<2, 2> jac = jacXyzToRhoZ * rotLocalToGlobal.block<3, 2>(Acts::ePos0, Acts::ePos0);
			// compute rho/z variance
			Acts::ActsVector<2> var = (jac * localCov * jac.transpose()).diagonal();

			// Save spacepoint
			spacePoints.push_back(ACTSTracking::SeedSpacePoint(globalPos, var[0], var[1], sourceLink));
		}
	}
	
	info() << "Created " << spacePoints.size() << " space points" << endmsg;

	auto hitEnd = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> hitDuration = hitEnd - hitStart;
	//m_histHitSetUp->Fill(hitDuration.count());

	// Run seeding + tracking algorithms
	// Caches
	Acts::MagneticFieldContext magFieldContext = Acts::MagneticFieldContext();
	Acts::MagneticFieldProvider::Cache magCache = magneticField()->makeCache(magFieldContext);

	// Initialize track finder
	using Updater = Acts::GainMatrixUpdater;
	using Smoother = Acts::GainMatrixSmoother;
	using Stepper = Acts::EigenStepper<>;
	using Navigator = Acts::Navigator;
	using Propagator = Acts::Propagator<Stepper, Navigator>;
	using CKF = Acts::CombinatorialKalmanFilter<Propagator, Acts::VectorMultiTrajectory>;

	// Configurations
	Navigator::Config navigatorCfg{ trackingGeometry() };
	navigatorCfg.resolvePassive = false;
	navigatorCfg.resolveMaterial = true;
	navigatorCfg.resolveSensitive = true;

	// construct all components for the fitter
	Stepper stepper(magneticField());
	Navigator navigator(navigatorCfg);
	Propagator propagator(std::move(stepper), std::move(navigator));
	CKF trackFinder(std::move(propagator));

	// Set the options
	Acts::MeasurementSelector::Config measurementSelectorCfg = {
		{ Acts::GeometryIdentifier(), { {}, { m_CKF_chi2CutOff }, { (std::size_t)(m_CKF_numMeasurementsCutOff) } } }
	};

	Acts::PropagatorPlainOptions pOptions;
	pOptions.maxSteps = 10000;
	if (m_propagateBackward) {
		pOptions.direction = Acts::Direction::Backward;
	}

	// Construct a perigee surface as the target surface
	std::shared_ptr<Acts::PerigeeSurface> perigeeSurface = Acts::Surface::makeShared<Acts::PerigeeSurface>(Acts::Vector3{0., 0., 0.});

	Acts::GainMatrixUpdater kfUpdater;
	Acts::GainMatrixSmoother kfSmoother;

	Acts::MeasurementSelector measSel { measurementSelectorCfg };
	ACTSTracking::MeasurementCalibrator measCal { measurements };
	Acts::CombinatorialKalmanFilterExtensions<Acts::VectorMultiTrajectory> extensions;
	extensions.calibrator.connect<&ACTSTracking::MeasurementCalibrator::calibrate>(&measCal);
	extensions.updater.connect<&Acts::GainMatrixUpdater::operator()<Acts::VectorMultiTrajectory>>(&kfUpdater);
	extensions.smoother.connect<&Acts::GainMatrixSmoother::operator()<Acts::VectorMultiTrajectory>>(&kfSmoother);
	extensions.measurementSelector.connect<&Acts::MeasurementSelector::select<Acts::VectorMultiTrajectory>>(&measSel);

	using ACTSTracking::SourceLinkAccessor;
	SourceLinkAccessor slAccessor;
	slAccessor.container = &sourceLinks;
	Acts::SourceLinkAccessorDelegate<SourceLinkAccessor::Iterator> slAccessorDelegate;
	slAccessorDelegate.connect<&SourceLinkAccessor::range>(&slAccessor);

	// std::unique_ptr<const Acts::Logger>
	// logger=Acts::getDefaultLogger("TrackFitting",
	// Acts::Logging::Level::VERBOSE);

	TrackFinderOptions ckfOptions = TrackFinderOptions(
		geometryContext(),
		magneticFieldContext(), 
		calibrationContext(),
		slAccessorDelegate, 
		extensions, 
		pOptions, 
		perigeeSurface.get() );

	// Finder configuration
	static const Acts::Vector3 zeropos(0, 0, 0);

	Acts::SeedFinderConfig<ACTSTracking::SeedSpacePoint> finderCfg;
	finderCfg.rMax = m_seedFinding_rMax;
	finderCfg.deltaRMin = m_seedFinding_deltaRMin;
	finderCfg.deltaRMax = m_seedFinding_deltaRMax;
	finderCfg.deltaRMinTopSP = m_seedFinding_deltaRMinTop;
	finderCfg.deltaRMaxTopSP = m_seedFinding_deltaRMaxTop;
	finderCfg.deltaRMinBottomSP = m_seedFinding_deltaRMinBottom;
	finderCfg.deltaRMaxBottomSP = m_seedFinding_deltaRMaxBottom;
	finderCfg.collisionRegionMin = -m_seedFinding_collisionRegion;
	finderCfg.collisionRegionMax = m_seedFinding_collisionRegion;
	finderCfg.zMin = -m_seedFinding_zMax;
	finderCfg.zMax = m_seedFinding_zMax;
	finderCfg.maxSeedsPerSpM = 1;
	finderCfg.cotThetaMax = 7.40627;  // 2.7 eta;
	finderCfg.sigmaScattering = m_seedFinding_sigmaScattering;
	finderCfg.radLengthPerSeed = m_seedFinding_radLengthPerSeed;
	finderCfg.minPt = m_seedFinding_minPt * Acts::UnitConstants::MeV;
	finderCfg.impactMax = m_seedFinding_impactMax * Acts::UnitConstants::mm;
	finderCfg.useVariableMiddleSPRange = true;

	Acts::SeedFilterConfig filterCfg;
	filterCfg.maxSeedsPerSpM = finderCfg.maxSeedsPerSpM;

	finderCfg.seedFilter = std::make_unique<Acts::SeedFilter<ACTSTracking::SeedSpacePoint>>(
		Acts::SeedFilter<ACTSTracking::SeedSpacePoint>(filterCfg.toInternalUnits()));
	finderCfg = finderCfg.toInternalUnits().calculateDerivedQuantities();

	Acts::SeedFinderOptions finderOpts;
	finderOpts.bFieldInZ = (*magneticField()->getField(zeropos, magCache))[2];   // TODO investigate
	finderOpts.beamPos = {0, 0};
	finderOpts = finderOpts.toInternalUnits();
	finderOpts = finderOpts.calculateDerivedQuantities(finderCfg);

	Acts::CylindricalSpacePointGridConfig gridCfg;
	gridCfg.cotThetaMax = finderCfg.cotThetaMax;
	gridCfg.deltaRMax = finderCfg.deltaRMax;
	gridCfg.minPt = finderCfg.minPt;
	gridCfg.rMax = finderCfg.rMax;
	gridCfg.zMax = finderCfg.zMax;
	gridCfg.zMin = finderCfg.zMin;
	gridCfg.impactMax = finderCfg.impactMax;
	if (m_seedFinding_zBinEdges.size() > 0) {
		gridCfg.zBinEdges.resize(m_seedFinding_zBinEdges.size());
		for (int k = 0; k < m_seedFinding_zBinEdges.size(); k++) {
			float pos = std::atof(m_seedFinding_zBinEdges[k].c_str());
			if (pos >= finderCfg.zMin && pos < finderCfg.zMax) {
				gridCfg.zBinEdges[k] = pos;
			}
			else {
				warning() << "Wrong parameter SeedFinding_zBinEdges; " << "default used" <<endmsg;
				gridCfg.zBinEdges.clear();
				break;
			}
		}
	}

	Acts::CylindricalSpacePointGridOptions gridOpts;
	gridOpts.bFieldInZ = (*magneticField()->getField(zeropos, magCache))[2];

	// Create tools
	auto extractGlobalQuantities = [](const SSPoint& sp, float, float, float) {
		Acts::Vector3 position { sp.x(), sp.y(), sp.z() };
		Acts::Vector2 covariance { sp.varianceR(), sp.varianceZ() };
		return std::make_tuple(position, covariance, sp.t());
	};

	std::vector<const ACTSTracking::SeedSpacePoint*> spacePointPtrs(spacePoints.size(), nullptr);
	std::transform(spacePoints.begin(), spacePoints.end(), spacePointPtrs.begin(),
		[](const ACTSTracking::SeedSpacePoint &sp) { return &sp; });

	Acts::Extent rRangeSPExtent;

	Acts::CylindricalSpacePointGrid<SSPoint> grid = 
		Acts::CylindricalSpacePointGridCreator::createGrid<SSPoint>(
				gridCfg.toInternalUnits(), gridOpts.toInternalUnits());
	Acts::CylindricalSpacePointGridCreator::fillGrid(finderCfg, finderOpts, grid, 
			spacePointPtrs.begin(), spacePointPtrs.end(), extractGlobalQuantities, rRangeSPExtent);

	const Acts::GridBinFinder<2ul> bottomBinFinder(m_phiBottomBinLen.value(), m_zBottomBinLen.value());
	const Acts::GridBinFinder<2ul> topBinFinder(m_phiTopBinLen.value(), m_zTopBinLen.value());

	auto spacePointsGrouping = Acts::CylindricalBinnedGroup<SSPoint>(
		std::move(grid), bottomBinFinder, topBinFinder);

	Acts::SeedFinder<SSPoint> finder(finderCfg);
	decltype(finder)::SeedingState state;
	std::vector<Acts::Seed<SSPoint>> seeds;

	state.spacePointData.resize(spacePointPtrs.size(), finderCfg.useDetailedDoubleMeasurementInfo);

	float up = Acts::clampValue<float>(std::floor(rRangeSPExtent.max(Acts::binR) / 2) * 2);
	const Acts::Range1D<float> rMiddleSPRange(
		std::floor(rRangeSPExtent.min(Acts::binR) / 2) * 2 + finderCfg.deltaRMiddleMinSPRange,
		up - finderCfg.deltaRMiddleMaxSPRange);                  // TODO investigate

	auto entireStart = std::chrono::high_resolution_clock::now();
	
	std::vector<Acts::BoundTrackParameters> paramseeds;
	
	for (const auto [bottom, middle, top] : spacePointsGrouping) {
		seeds.clear();

		finder.createSeedsForGroup(finderOpts, state, spacePointsGrouping.grid(),
			std::back_inserter(seeds), bottom, middle, top, rMiddleSPRange);
	
		auto seedStart = std::chrono::high_resolution_clock::now();
		
		// Loop over seeds and get track parameters
		paramseeds.clear();

		for (const Acts::Seed<SSPoint> &seed : seeds) {
			const SSPoint* bottomSP = seed.sp().front();

			const auto& sourceLink = bottomSP->sourceLink();
			const Acts::GeometryIdentifier& geoId = sourceLink.geometryId();
			const Acts::Surface* surface = trackingGeometry()->findSurface(geoId);
			if (surface == nullptr) {
				info() << "surface with geoID " << geoId << " is not found in the tracking gemetry" << endmsg;
				continue;
			}

			// Get the magnetic field at the bottom space point
			const Acts::Vector3 seedPos(bottomSP->x(), bottomSP->y(), bottomSP->z());
			Acts::Result<Acts::Vector3> seedField = magneticField()->getField(seedPos, magCache);
			if (!seedField.ok()) {
				throw std::runtime_error("Field lookup error: " + seedField.error().value());
			}

			std::optional<Acts::BoundVector> optParams = Acts::estimateTrackParamsFromSeed(geometryContext(),
				seed.sp().begin(), seed.sp().end(), *surface, *seedField, 0.1_T);
			if (!optParams.has_value()) {
				info() << "Failed estimation of track parameters for seed." << endmsg;
				continue;
			}

			const Acts::BoundVector &params = optParams.value();

			float charge = std::copysign(1, params[Acts::eBoundQOverP]);
			float p = std::abs(1 / params[Acts::eBoundQOverP]);

			// build the track covariance matrix using the smearing sigmas
			Acts::BoundSquareMatrix cov = Acts::BoundSquareMatrix::Zero();
			cov(Acts::eBoundLoc0, Acts::eBoundLoc0) = std::pow(m_initialTrackError_pos, 2);
			cov(Acts::eBoundLoc1, Acts::eBoundLoc1) = std::pow(m_initialTrackError_pos, 2);
			cov(Acts::eBoundTime, Acts::eBoundTime) = std::pow(m_initialTrackError_time, 2);
			cov(Acts::eBoundPhi, Acts::eBoundPhi) = std::pow(m_initialTrackError_phi, 2);
			cov(Acts::eBoundTheta, Acts::eBoundTheta) = std::pow(m_initialTrackError_lambda, 2);
			cov(Acts::eBoundQOverP, Acts::eBoundQOverP) = std::pow(m_initialTrackError_relP * p / (p * p), 2);

			Acts::BoundTrackParameters paramseed(surface->getSharedPtr(), params, cov, Acts::ParticleHypothesis::pion());
			paramseeds.push_back(paramseed);

			// Add seed to edm4hep collection
			edm4hep::MutableTrack seedTrack = seedCollection->create();

			Acts::Vector3 globalPos = surface->localToGlobal(
				geometryContext(), {params[Acts::eBoundLoc0], params[Acts::eBoundLoc1]}, {0, 0, 0});

			// state
			Acts::Result<Acts::Vector3> hitField = magneticField()->getField(globalPos, magCache);
			if (!hitField.ok()) {
				throw std::runtime_error("Field lookup error: " + hitField.error().value());
			}

			edm4hep::TrackState* seedTrackState = ACTSTracking::ACTS2edm4hep_trackState(
					edm4hep::TrackState::AtFirstHit, paramseed, (*hitField)[2] / Acts::UnitConstants::T);;

			// hits
			for (const ACTSTracking::SeedSpacePoint *sp : seed.sp()) {
				const ACTSTracking::SourceLink& sourceLink = sp->sourceLink();
				seedTrack.addToTrackerHits(*(sourceLink.edm4hepTHitP()));//trackHit);
			}

			seedTrack.addToTrackStates(*seedTrackState);

			debug() << "Seed Paramemeters" << std::endl << paramseed << endmsg;
		}

		debug() << "Seeds found: " << std::endl << paramseeds.size() << endmsg;
		
		auto seedEnd = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> seedDuration = seedEnd - seedStart;
		//m_histSeedFinding->Fill(seedDuration.count());
	
		
		// Find the tracks
		if (!m_runCKF) continue;

		using TrackContainer = Acts::TrackContainer<Acts::VectorTrackContainer,
			Acts::VectorMultiTrajectory, std::shared_ptr>;
		auto trackContainer = std::make_shared<Acts::VectorTrackContainer>();
		auto trackStateContainer = std::make_shared<Acts::VectorMultiTrajectory>();
		TrackContainer tracks(trackContainer, trackStateContainer);

		auto trackStart = std::chrono::high_resolution_clock::now();
		
		for (std::size_t iseed =0; iseed < paramseeds.size(); ++iseed) {
			tracks.clear();

			auto result = trackFinder.findTracks(paramseeds.at(iseed), ckfOptions, tracks);
			if (result.ok()) {
				const auto& fitOutput = result.value();
				for (const TrackContainer::TrackProxy& trackTip : fitOutput) {
					// Helpful debug output
					debug() << "Trajectory Summary" << endmsg;
					debug() << "\tchi2Sum       " << trackTip.chi2() << endmsg;
					debug() << "\tNDF           " << trackTip.nDoF() << endmsg;
					debug() << "\tnHoles        " << trackTip.nHoles() << endmsg;
					debug() << "\tnMeasurements " << trackTip.nMeasurements() << endmsg;
					debug() << "\tnOutliers     " << trackTip.nOutliers() << endmsg;
					debug() << "\tnStates       " << trackTip.nTrackStates() << endmsg;

					// Make track object
					edm4hep::MutableTrack* track = ACTSTracking::ACTS2edm4hep_track(trackTip, magneticField(), magCache);

					// Save results
					trackCollection->push_back(*track);
					
				}
			} else {
				warning() << "Track fit error: " << result.error() << endmsg;
			}
		}
		
		auto trackEnd = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> trackDuration = trackEnd - trackStart;
		//m_histTrackBuild->Fill(trackDuration.count());

	}
	
	auto entireEnd = std::chrono::high_resolution_clock::now();	
	std::chrono::duration<double> entireDuration = entireEnd - entireStart;
	//m_histEntireReco->Fill(entireDuration.count());
	info() << "Track Collection Size: " << trackCollection->size() << endmsg; 
	
	return std::make_tuple(std::move(seedCollection), std::move(trackCollection));
}


