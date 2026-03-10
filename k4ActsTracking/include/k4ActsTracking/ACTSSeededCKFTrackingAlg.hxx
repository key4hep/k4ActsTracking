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

#ifndef ACTSSeededCKFTrackingAlg_h
#define ACTSSeededCKFTrackingAlg_h 1

// edm4hep
#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackerHitPlaneCollection.h>

// gaudi
#include <Gaudi/Property.h>

// ACTS
#include <Acts/Definitions/Units.hpp>
#include <Acts/EventData/SpacePointContainer.hpp>
#include <Acts/EventData/TrackContainer.hpp>
#include <Acts/EventData/TrackParameters.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>
#include <Acts/EventData/VectorTrackContainer.hpp>
#include <Acts/Propagator/EigenStepper.hpp>
#include <Acts/Propagator/Navigator.hpp>
#include <Acts/Propagator/Propagator.hpp>
#include <Acts/Seeding/detail/CylindricalSpacePointGrid.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>

// ROOT
#include <TH1.h>

// ACTSTracking
#include "k4ActsTracking/ACTSAlgBase.hxx"
#include "k4ActsTracking/GeometryIdSelector.hxx"
#include "k4ActsTracking/Measurement.hxx"
#include "k4ActsTracking/SeedSpacePoint.hxx"
#include "k4ActsTracking/SourceLink.hxx"
#include "k4ActsTracking/SpacePointContainer.hxx"

// Standard
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace Acts {
  template <typename T, typename G, typename P> class SeedFinder;
  class SeedFinderOptions;
}  // namespace Acts

/**
 * @brief Reconstruction algorithm for ACTSTracking
 * This code performs reconstruction by looping over all the
 * Tracker Hits and build track seeds. Those seeds are then
 * extended into full tracks.
 *
 * @author Karol Krizka
 * @author Samuel Ferraro
 */
struct ACTSSeededCKFTrackingAlg final : ACTSAlgBase {
  // Track fitting definitions
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

public:
  /**
         * @brief Constructor for ACTSSeededCKFTracking
         * @param name unique string identifier for this instance
         * @param svcLoc a Service Locator passed by the Gaudi AlgManager
         */
  ACTSSeededCKFTrackingAlg(const std::string& name, ISvcLocator* svcLoc);

  /**
         * @brief Initializes the geometry for the detector
         */
  StatusCode initialize();
  /**
         * @brief ACTSSeededCKFTracking operation. The workhorse of this MultiTransformer.
         * @param trackerHitCollection A merged collection of all the tracker hits in the detector
         * @return A tuple of Track Collections: The reconstructed tracks and the seeds that led to those tracks
         */
  std::tuple<edm4hep::TrackCollection, edm4hep::TrackCollection> operator()(
      const edm4hep::TrackerHitPlaneCollection& trackerHitCollection) const;

  std::vector<Acts::BoundTrackParameters> findSeeds(const Acts::SeedFinder<SSPoint, SSPointGrid, void*>& finder,
                                                    const Acts::SeedFinderOptions&                       finderOpts,
                                                    const auto& spacePointGroup, const SSPointGrid& grid,
                                                    const Acts::Range1D<float>          middleSpRange,
                                                    const size_t                        mutSpDataSize,
                                                    edm4hep::TrackCollection&           seedCollection,
                                                    Acts::MagneticFieldProvider::Cache& magCache) const;

  StatusCode tracking(const std::vector<Acts::BoundTrackParameters>& paramseeds, const CKF& trackFinder,
                      const TrackFinderOptions& ckfOptions, Acts::MagneticFieldProvider::Cache& magCache,
                      edm4hep::TrackCollection& trackCollection) const;

protected:
  /**
	 * @brief Run Specifc Settings
	 */
  ///@{
  Gaudi::Property<bool> m_runCKF{this, "RunCKF", true,
                                 "Run tracking using CKF. False means stop at the seeding stage."};
  Gaudi::Property<bool> m_propagateBackward{this, "PropagateBackward", false, "Extrapolates tracks towards beamline."};
  ///@}

  /**
	 * @brief Seed finding configuration
	 */
  ///@{
  Gaudi::Property<float> m_seedFinding_rMax{this, "SeedFinding_RMax", 150, "Maximum radius of hits to consider."};
  Gaudi::Property<float> m_seedFinding_deltaRMin{this, "SeedFinding_DeltaRMin", 5,
                                                 "Minimum dR between hits in a seed."};
  Gaudi::Property<float> m_seedFinding_deltaRMax{this, "SeedFinding_DeltaRMax", 80,
                                                 "Maximum dR between hits in a seed."};
  Gaudi::Property<float> m_seedFinding_deltaRMinTop{this, "SeedFinding_DeltaRMinTop", 0.f,
                                                    "Minimum dR between the reference hit and outer ones in a seed."};
  Gaudi::Property<float> m_seedFinding_deltaRMaxTop{this, "SeedFinding_DeltaRMaxTop", 0.f,
                                                    "Maximum dR between the reference hit and outer ones in a seed."};
  Gaudi::Property<float> m_seedFinding_deltaRMinBottom{
      this, "SeedFinding_DeltaRMinBottom", 0.f, "Minimum dR between the reference hit and inner ones in a seed."};
  Gaudi::Property<float> m_seedFinding_deltaRMaxBottom{
      this, "SeedFinding_DeltaRMaxBottom", 0.f, "Maximum dR between the reference hit and inner ones in a seed."};
  Gaudi::Property<float> m_seedFinding_collisionRegion{
      this, "SeedFinding_CollisionRegion", 75.0, "Size of the collision region in one direction (assumed symmetric)."};
  Gaudi::Property<float> m_seedFinding_zMax{this, "SeedFinding_ZMax", 600.0, "Maximum z of hits to consider."};
  Gaudi::Property<float> m_seedFinding_sigmaScattering{this, "SeedFinding_SigmaScattering", 50.0,
                                                       "Number of sigmas to allow in scattering angle."};
  Gaudi::Property<float> m_seedFinding_radLengthPerSeed{this, "SeedFinding_RadLengthPerSeed", 0.1,
                                                        "Average radiation length per seed."};
  Gaudi::Property<float> m_seedFinding_minPt{this, "SeedFinding_MinPt", 500.0, "Minimum pT of tracks to seed."};
  Gaudi::Property<float> m_seedFinding_impactMax{this, "SeedFinding_ImpactMax", 3.0 * Acts::UnitConstants::mm,
                                                 "Maximum d0 of tracks to seed."};

  std::vector<std::string> default_value;  // Not sure if this is a needed step, but this broke when I did it inline.
  Gaudi::Property<std::vector<std::string>> m_seedFinding_zBinEdges{this, "SeedFinding_zBinEdges", default_value,
                                                                    "Bins placement along Z for seeding."};
  Gaudi::Property<int> m_zTopBinLen{this, "SeedFinding_zTopBinLen", 1, "Number of top bins along Z for seeding."};
  Gaudi::Property<int> m_zBottomBinLen{this, "SeedFinding_zBottomBinLen", 1,
                                       "Number of bottom bins along Z for seeding."};
  Gaudi::Property<int> m_phiTopBinLen{this, "SeedFinding_phiTopBinLen", 1, "Number of top bins along phi for seeding."};
  Gaudi::Property<int> m_phiBottomBinLen{this, "SeedFinding_phiBottomBinLen", 1,
                                         "Number of bottom bins along phi for seeding."};
  ///@}

  /**
	 * @brief Track fit parameters
	 */
  ///@{
  Gaudi::Property<double>  m_initialTrackError_pos{this, "InitialTrackError_Pos", 10 * Acts::UnitConstants::um,
                                                  "Track error estimate, local position (mm)."};
  Gaudi::Property<double>  m_initialTrackError_phi{this, "InitialTrackError_Phi", 1 * Acts::UnitConstants::degree,
                                                  "Track error estimate, phi (radians)."};
  Gaudi::Property<double>  m_initialTrackError_relP{this, "InitialTrackError_RelP", 0.25,
                                                   "Track error estimate, momentum component (relative)."};
  Gaudi::Property<double>  m_initialTrackError_lambda{this, "InitialTrackError_Lambda", 1 * Acts::UnitConstants::degree,
                                                     "Track error estimate, lambda (radians)."};
  Gaudi::Property<double>  m_initialTrackError_time{this, "InitialTrackError_Time", 100 * Acts::UnitConstants::ns,
                                                   "Track error estimate, time (sec)."};
  Gaudi::Property<double>  m_CKF_chi2CutOff{this, "CKF_Chi2CutOff", 15, "Maximum local chi2 contribution."};
  Gaudi::Property<int32_t> m_CKF_numMeasurementsCutOff{
      this, "CKF_NumMeasurementsCutOff", 10, "Maximum number of associated measurements on a single surface."};
  ///@}

  /**
	 * @brief Seeding configuration
	 */
  ///@{
  Gaudi::Property<std::vector<std::string>> m_seedingLayers{this, "SeedingLayers", default_value,
                                                            "Layers to use for seeding in vector."};
  ACTSTracking::GeometryIdSelector          m_seedGeometrySelection;
  ///@}

  /**
   * @brief Multithreading configuration
   */
  ///@{
  Gaudi::Property<int> m_numThreads{this, "NumThreads", 1, "Number of threads to use for internal multithreading."};
  ///@}

  // Thread-safe counter
  // mutable Gaudi::Accumulators::Counter<> m_fitFails{this, "FitFails"};

private:
  // Mutexes for threadsafe container filling of seeds and tracks
  mutable std::mutex m_seedMutex{};
  mutable std::mutex m_trackMutex{};
};

#include "k4ActsTracking/Helpers.hxx"

#include <Acts/Seeding/EstimateTrackParamsFromSeed.hpp>
#include <Acts/Seeding/SeedFinder.hpp>

// TODO: Proper typing
std::vector<Acts::BoundTrackParameters> ACTSSeededCKFTrackingAlg::findSeeds(
    const Acts::SeedFinder<SSPoint, SSPointGrid>& finder, const Acts::SeedFinderOptions& finderOpts,
    const auto& spacePointGroup, const SSPointGrid& grid, const Acts::Range1D<float> middleSpRange,
    const size_t mutSpDataSize, edm4hep::TrackCollection& seedCollection,
    Acts::MagneticFieldProvider::Cache& magCache) const {
  const auto& [bottom, middle, top] = spacePointGroup;
  std::vector<Acts::Seed<SSPoint>>                     seeds;
  std::vector<Acts::BoundTrackParameters>              paramseeds;
  Acts::SeedFinder<SSPoint, SSPointGrid>::SeedingState state;
  state.spacePointMutableData.resize(mutSpDataSize);

  finder.createSeedsForGroup(finderOpts, state, grid, seeds, bottom, middle, top, middleSpRange);

  // Loop over seeds and get track parameters
  std::vector<Acts::Seed<ACTSTracking::SeedSpacePoint>> f_seeds;
  f_seeds.reserve(seeds.size());
  for (const Acts::Seed<SSPoint>& seed : seeds) {
    const auto& sps = seed.sp();
    f_seeds.emplace_back(*sps[0]->externalSpacePoint(), *sps[1]->externalSpacePoint(), *sps[2]->externalSpacePoint());
  }

  for (const auto& seed : f_seeds) {
    const ACTSTracking::SeedSpacePoint* bottomSP = seed.sp().front();

    const auto&                     sourceLink = bottomSP->sourceLink();
    const Acts::GeometryIdentifier& geoId      = sourceLink.geometryId();
    const Acts::Surface*            surface    = trackingGeometry()->findSurface(geoId);
    if (surface == nullptr) {
      warning() << "surface with geoID " << geoId << " is not found in the tracking gemetry" << endmsg;
      continue;
    }

    // Get the magnetic field at the bottom space point
    const Acts::Vector3         seedPos(bottomSP->x(), bottomSP->y(), bottomSP->z());
    Acts::Result<Acts::Vector3> seedField = magneticField()->getField(seedPos, magCache);
    if (!seedField.ok()) {
      throw std::runtime_error("Field lookup error: " + std::to_string(seedField.error().value()));
    }

    Acts::Result<Acts::BoundVector> optParams =
        Acts::estimateTrackParamsFromSeed(geometryContext(), seed.sp(), *surface, *seedField);
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
      throw std::runtime_error("Field lookup error: " + std::to_string(hitField.error().value()));
    }

    auto seedTrackState = ACTSTracking::ACTS2edm4hep_trackState(edm4hep::TrackState::AtFirstHit, paramseed,
                                                                (*hitField)[2] / Acts::UnitConstants::T);

    // Add seed to collection, all building of seed under the lock
    {
      std::lock_guard<std::mutex> lock(m_seedMutex);
      auto                        seedTrack = seedCollection.create();
      for (const ACTSTracking::SeedSpacePoint* sp : seed.sp()) {
        seedTrack.addToTrackerHits(sp->sourceLink().edm4hepHit());
      }
      seedTrack.addToTrackStates(seedTrackState);
    }

    debug() << "Seed Paramemeters" << std::endl << paramseed << endmsg;
  }

  debug() << "Seeds found: " << std::endl << paramseeds.size() << endmsg;

  return paramseeds;
}

#endif
