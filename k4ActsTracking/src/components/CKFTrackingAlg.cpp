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
#include <Acts/EventData/BoundTrackParameters.hpp>
#include <Acts/EventData/FreeTrackParameters.hpp>
#include <Acts/EventData/ParticleHypothesis.hpp>
#include <Acts/EventData/SeedContainer2.hpp>
#include <Acts/EventData/SpacePointContainer2.hpp>
#include <Acts/EventData/TrackContainer.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>
#include <Acts/EventData/TrackStateType.hpp>
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
#include <Acts/Utilities/TrackHelpers.hpp>

// TBB
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>

#include <fmt/ostream.h>

// Standard
#include <atomic>
#include <cstdint>
#include <limits>
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

  StatusCode finalize() override;

  std::tuple<edm4hep::TrackCollection, edm4hep::TrackCollection> operator()(
      const edm4hep::TrackerHitPlaneCollection&             trackerHitCollection,
      const edm4hep::TrackerHitSimTrackerHitLinkCollection& trackerHitRelations) const override;

  // ----- private helpers ---------------------------------------------------
private:
  /// Convert the seeds found by the triplet seeder into ACTS bound track
  /// parameters and create the corresponding edm4hep seed tracks. The space
  /// point indices stored in @p seeds reference @p spacePoints.
  std::vector<Acts::BoundTrackParameters> seedsToParameters(const Acts::SeedContainer2&         seeds,
                                                            const Acts::SpacePointContainer2&   spacePoints,
                                                            edm4hep::TrackCollection&           seedCollection,
                                                            Acts::MagneticFieldProvider::Cache& magCache) const;

  StatusCode tracking(const std::vector<Acts::BoundTrackParameters>& paramseeds, std::size_t begin, std::size_t end,
                      const CKF& trackFinder, const TrackFinderOptions& ckfOptions,
                      Acts::MagneticFieldProvider::Cache& magCache, edm4hep::TrackCollection& trackCollection) const;

  // ----- Gaudi properties --------------------------------------------------

  /// @name Run control
  ///@{
  Gaudi::Property<bool> m_runCKF{this, "RunCKF", true, "Run tracking using CKF. False means stop at seeding."};
  Gaudi::Property<bool> m_propagateBackward{this, "PropagateBackward", false, "Extrapolates tracks towards beamline."};
  Gaudi::Property<bool> m_extrapolateToCalo{
      this, "ExtrapolateToCalo", true,
      "Extrapolate fitted tracks to the calorimeter face and add an AtCalorimeter track state."};
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

  Gaudi::Property<float> m_seedFinding_deltaRMiddleMinSPRange{
      this, "SeedFinding_DeltaRMiddleMinSPRange", 10 * Acts::UnitConstants::mm,
      "Lower offset added to the rounded minimum radius when computing the variable middle space-point range."};
  Gaudi::Property<float> m_seedFinding_deltaRMiddleMaxSPRange{
      this, "SeedFinding_DeltaRMiddleMaxSPRange", 10 * Acts::UnitConstants::mm,
      "Upper offset subtracted from the rounded maximum radius when computing the variable middle space-point range."};
  ///@}

  /// @name Seed filter configuration
  ///@{
  Gaudi::Property<float> m_seedFilter_deltaInvHelixDiameter{
      this, "SeedFilter_DeltaInvHelixDiameter", 0.00003 * (1 / Acts::UnitConstants::mm),
      "Allowed difference in curvature (inverted seed radii) between two compatible seeds."};
  Gaudi::Property<float> m_seedFilter_compatSeedWeight{
      this, "SeedFilter_CompatSeedWeight", 200, "Weight added to a seed when a compatible seed is found (c1 factor)."};
  Gaudi::Property<float> m_seedFilter_impactWeightFactor{
      this, "SeedFilter_ImpactWeightFactor", 1,
      "Factor multiplying the transverse impact parameter d0, subtracted from the seed weight (c2 factor)."};
  Gaudi::Property<float> m_seedFilter_zOriginWeightFactor{
      this, "SeedFilter_ZOriginWeightFactor", 1,
      "Factor multiplying the longitudinal impact parameter z0, subtracted from the seed weight (c3 factor)."};
  Gaudi::Property<unsigned int> m_seedFilter_maxSeedsPerSpM{
      this, "SeedFilter_MaxSeedsPerSpM", 1, "Maximum number (minus one) of accepted seeds per middle space-point."};
  Gaudi::Property<unsigned int> m_seedFilter_compatSeedLimit{
      this, "SeedFilter_CompatSeedLimit", 2,
      "Maximum number of compatible space-points used in the score calculation."};
  Gaudi::Property<float> m_seedFilter_seedWeightIncrement{
      this, "SeedFilter_SeedWeightIncrement", 0,
      "Weight increment applied when the number of compatible seeds exceeds NumSeedIncrement."};
  Gaudi::Property<float> m_seedFilter_numSeedIncrement{
      this, "SeedFilter_NumSeedIncrement", std::numeric_limits<float>::max(),
      "Number of compatible seeds above which SeedWeightIncrement is applied (default: effectively disabled)."};
  Gaudi::Property<float> m_seedFilter_absDeltaEtaWeightFactor{
      this, "SeedFilter_AbsDeltaEtaWeightFactor", 0,
      "Weight factor for the abs(delta-eta) penalty between seed direction and beamspot-to-pca vector (0 disables)."};
  Gaudi::Property<float> m_seedFilter_absDeltaEtaMinImpact{
      this, "SeedFilter_AbsDeltaEtaMinImpact", 2 * Acts::UnitConstants::mm,
      "Minimum impact parameter required to apply the abs(delta-eta) weight."};
  Gaudi::Property<bool> m_seedFilter_seedConfirmation{
      this, "SeedFilter_SeedConfirmation", false,
      "Enable quality seed confirmation, defined per (r, z) detector region by the seed confirmation ranges."};
  Gaudi::Property<unsigned int> m_seedFilter_maxSeedsPerSpMConf{
      this, "SeedFilter_MaxSeedsPerSpMConf", 5,
      "Maximum number of normal-quality seeds per middle space-point in seed confirmation."};
  Gaudi::Property<unsigned int> m_seedFilter_maxQualitySeedsPerSpMConf{
      this, "SeedFilter_MaxQualitySeedsPerSpMConf", 5,
      "Maximum number of high-quality seeds per inner-middle space-point doublet in seed confirmation."};
  Gaudi::Property<bool> m_seedFilter_useDeltaRinsteadOfTopRadius{
      this, "SeedFilter_UseDeltaRinsteadOfTopRadius", false,
      "Use deltaR between top and middle space-point instead of the top radius to search for compatible SPs."};
  ///@}

  /// @name Seed confirmation ranges (only used when SeedFilter_SeedConfirmation is true)
  ///@{
  Gaudi::Property<float> m_centralSeedConf_zMin{
      this, "SeedFilter_CentralSeedConf_ZMin", std::numeric_limits<float>::lowest(),
      "Central region: minimum z of the middle space-point splitting the seed confirmation region."};
  Gaudi::Property<float> m_centralSeedConf_zMax{
      this, "SeedFilter_CentralSeedConf_ZMax", std::numeric_limits<float>::max(),
      "Central region: maximum z of the middle space-point splitting the seed confirmation region."};
  Gaudi::Property<float> m_centralSeedConf_rMax{
      this, "SeedFilter_CentralSeedConf_RMax", std::numeric_limits<float>::max(),
      "Central region: inner space-point radius splitting the seed confirmation region."};
  Gaudi::Property<unsigned int> m_centralSeedConf_nTopForLargeR{
      this, "SeedFilter_CentralSeedConf_NTopForLargeR", 0,
      "Central region: minimum compatible outer SPs required if the inner SP radius is larger than RMax."};
  Gaudi::Property<unsigned int> m_centralSeedConf_nTopForSmallR{
      this, "SeedFilter_CentralSeedConf_NTopForSmallR", 0,
      "Central region: minimum compatible outer SPs required if the inner SP radius is smaller than RMax."};
  Gaudi::Property<float> m_centralSeedConf_minBottomRadius{
      this, "SeedFilter_CentralSeedConf_MinBottomRadius", 60 * Acts::UnitConstants::mm,
      "Central region: minimum radius for the inner seed component in quality seed confirmation."};
  Gaudi::Property<float> m_centralSeedConf_maxZOrigin{
      this, "SeedFilter_CentralSeedConf_MaxZOrigin", 150 * Acts::UnitConstants::mm,
      "Central region: maximum longitudinal impact parameter of the seed in quality seed confirmation."};
  Gaudi::Property<float> m_centralSeedConf_minImpact{
      this, "SeedFilter_CentralSeedConf_MinImpact", 1 * Acts::UnitConstants::mm,
      "Central region: minimum impact parameter of the seed in quality seed confirmation."};

  Gaudi::Property<float> m_forwardSeedConf_zMin{
      this, "SeedFilter_ForwardSeedConf_ZMin", std::numeric_limits<float>::lowest(),
      "Forward region: minimum z of the middle space-point splitting the seed confirmation region."};
  Gaudi::Property<float> m_forwardSeedConf_zMax{
      this, "SeedFilter_ForwardSeedConf_ZMax", std::numeric_limits<float>::max(),
      "Forward region: maximum z of the middle space-point splitting the seed confirmation region."};
  Gaudi::Property<float> m_forwardSeedConf_rMax{
      this, "SeedFilter_ForwardSeedConf_RMax", std::numeric_limits<float>::max(),
      "Forward region: inner space-point radius splitting the seed confirmation region."};
  Gaudi::Property<unsigned int> m_forwardSeedConf_nTopForLargeR{
      this, "SeedFilter_ForwardSeedConf_NTopForLargeR", 0,
      "Forward region: minimum compatible outer SPs required if the inner SP radius is larger than RMax."};
  Gaudi::Property<unsigned int> m_forwardSeedConf_nTopForSmallR{
      this, "SeedFilter_ForwardSeedConf_NTopForSmallR", 0,
      "Forward region: minimum compatible outer SPs required if the inner SP radius is smaller than RMax."};
  Gaudi::Property<float> m_forwardSeedConf_minBottomRadius{
      this, "SeedFilter_ForwardSeedConf_MinBottomRadius", 60 * Acts::UnitConstants::mm,
      "Forward region: minimum radius for the inner seed component in quality seed confirmation."};
  Gaudi::Property<float> m_forwardSeedConf_maxZOrigin{
      this, "SeedFilter_ForwardSeedConf_MaxZOrigin", 150 * Acts::UnitConstants::mm,
      "Forward region: maximum longitudinal impact parameter of the seed in quality seed confirmation."};
  Gaudi::Property<float> m_forwardSeedConf_minImpact{
      this, "SeedFilter_ForwardSeedConf_MinImpact", 1 * Acts::UnitConstants::mm,
      "Forward region: minimum impact parameter of the seed in quality seed confirmation."};
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

  // Geometry-aware propagator used to extrapolate fitted tracks out to the
  // calorimeter face. The calo inner-face surfaces are part of the tracking
  // geometry (passive surfaces of dedicated calo volumes), so the propagation
  // follows the real trajectory through the detector and terminates on the
  // surface actually reached. Built once in initialize(), reused across threads.
  std::optional<ACTSTracking::CaloFacePropagator> m_caloPropagator{};

  // Propagator (with tracking-geometry navigator) used to extrapolate the
  // fitted track back to the perigee surface at the IP, so the track parameters
  // (in particular D0/Z0) are expressed there. Built once in initialize() and
  // reused read-only across threads.
  std::optional<Propagator>             m_extrapPropagator{};
  std::shared_ptr<Acts::PerigeeSurface> m_perigeeSurface{};

  k4ActsTracking::CellIDSelector m_seedSelector{};

  // Calorimeter-face extrapolation monitoring. operator() is const and runs on
  // many threads, so the counters are mutable and atomic. Summarised in
  // finalize() to report the rate at which the extrapolation fails.
  mutable std::atomic<std::size_t> m_caloAttempts{0};     ///< tracks with a usable start state
  mutable std::atomic<std::size_t> m_caloNoStartState{0}; ///< tracks without a measured smoothed state
  mutable std::atomic<std::size_t> m_caloNotReached{0};   ///< propagation did not reach a calo face
  mutable std::atomic<std::size_t> m_caloPropFailed{0};   ///< propagation itself failed
  mutable std::atomic<std::size_t> m_caloOk{0};           ///< reached a calo face

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

  // Build the propagator used to extrapolate fitted tracks back to the IP
  // perigee surface. It shares the tracking-geometry navigator configuration
  // with the CKF so it can navigate through the detector to the beam line.
  Stepper    extrapStepper(m_actsGeoSvc->magneticField());
  Navigator  extrapNavigator(navigatorCfg);
  Propagator extrapPropagator(std::move(extrapStepper), std::move(extrapNavigator));
  m_extrapPropagator.emplace(std::move(extrapPropagator));
  m_perigeeSurface = Acts::Surface::makeShared<Acts::PerigeeSurface>(Acts::Vector3{0., 0., 0.});

  // The calorimeter inner-face surfaces are part of the tracking geometry, so
  // extrapolation there uses a geometry-aware propagator. The calo surfaces are
  // passive, so the navigator must resolve passive surfaces. Only build it when
  // requested and when the geometry service actually provides calo surfaces.
  if (m_extrapolateToCalo) {
    if (m_actsGeoSvc->caloSurfaceGeoIds().empty()) {
      warning() << "ExtrapolateToCalo requested but ActsGeoSvc provides no calorimeter-face surfaces; "
                   "no AtCalorimeter track states will be produced."
                << endmsg;
    } else {
      Navigator::Config caloNavigatorCfg{m_actsGeoSvc->trackingGeometry()};
      caloNavigatorCfg.resolvePassive   = true;
      caloNavigatorCfg.resolveMaterial  = true;
      caloNavigatorCfg.resolveSensitive = true;

      Stepper   caloStepper(m_actsGeoSvc->magneticField());
      Navigator caloNavigator(caloNavigatorCfg);
      m_caloPropagator.emplace(std::move(caloStepper), std::move(caloNavigator));
    }
  }

  return StatusCode::SUCCESS;
}

StatusCode CKFTrackingAlg::finalize() {
  if (m_extrapolateToCalo) {
    const std::size_t attempts   = m_caloAttempts.load();
    const std::size_t ok         = m_caloOk.load();
    const std::size_t notReached = m_caloNotReached.load();
    const std::size_t propFailed = m_caloPropFailed.load();
    const std::size_t noStart    = m_caloNoStartState.load();
    const std::size_t failed     = notReached + propFailed;
    const double      failRate   = attempts > 0 ? static_cast<double>(failed) / static_cast<double>(attempts) : 0.0;

    info() << fmt::format(
                  "Calorimeter-face extrapolation summary: {} attempts, {} reached the face, {} failed "
                  "({:.2f}%: {} not reached, {} propagation errors); {} tracks had no measured smoothed start state.",
                  attempts, ok, failed, 100.0 * failRate, notReached, propFailed, noStart)
           << endmsg;
  }
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

  const float bFieldInZ       = (*m_actsGeoSvc->magneticField()->getField(zeropos, magCache))[2];
  const float cotThetaMax     = 7.40627f;  // ~2.7 η
  const float minPt           = m_seedFinding_minPt * Acts::UnitConstants::MeV;
  const float impactMax       = m_seedFinding_impactMax * Acts::UnitConstants::mm;
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
  const float                   deltaRMiddleMinSPRange = m_seedFinding_deltaRMiddleMinSPRange;
  const float                   deltaRMiddleMaxSPRange = m_seedFinding_deltaRMiddleMaxSPRange;
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
  filterCfg.deltaInvHelixDiameter       = m_seedFilter_deltaInvHelixDiameter;
  filterCfg.deltaRMin                   = m_seedFinding_deltaRMin;
  filterCfg.compatSeedWeight            = m_seedFilter_compatSeedWeight;
  filterCfg.impactWeightFactor          = m_seedFilter_impactWeightFactor;
  filterCfg.zOriginWeightFactor         = m_seedFilter_zOriginWeightFactor;
  filterCfg.maxSeedsPerSpM              = m_seedFilter_maxSeedsPerSpM;
  filterCfg.compatSeedLimit             = m_seedFilter_compatSeedLimit;
  filterCfg.seedWeightIncrement         = m_seedFilter_seedWeightIncrement;
  filterCfg.numSeedIncrement            = m_seedFilter_numSeedIncrement;
  filterCfg.absDeltaEtaWeightFactor     = m_seedFilter_absDeltaEtaWeightFactor;
  filterCfg.absDeltaEtaMinImpact        = m_seedFilter_absDeltaEtaMinImpact;
  filterCfg.seedConfirmation            = m_seedFilter_seedConfirmation;
  filterCfg.maxSeedsPerSpMConf          = m_seedFilter_maxSeedsPerSpMConf;
  filterCfg.maxQualitySeedsPerSpMConf   = m_seedFilter_maxQualitySeedsPerSpMConf;
  filterCfg.useDeltaRinsteadOfTopRadius = m_seedFilter_useDeltaRinsteadOfTopRadius;

  filterCfg.centralSeedConfirmationRange.zMinSeedConf            = m_centralSeedConf_zMin;
  filterCfg.centralSeedConfirmationRange.zMaxSeedConf            = m_centralSeedConf_zMax;
  filterCfg.centralSeedConfirmationRange.rMaxSeedConf            = m_centralSeedConf_rMax;
  filterCfg.centralSeedConfirmationRange.nTopForLargeR           = m_centralSeedConf_nTopForLargeR;
  filterCfg.centralSeedConfirmationRange.nTopForSmallR           = m_centralSeedConf_nTopForSmallR;
  filterCfg.centralSeedConfirmationRange.seedConfMinBottomRadius = m_centralSeedConf_minBottomRadius;
  filterCfg.centralSeedConfirmationRange.seedConfMaxZOrigin      = m_centralSeedConf_maxZOrigin;
  filterCfg.centralSeedConfirmationRange.minImpactSeedConf       = m_centralSeedConf_minImpact;

  filterCfg.forwardSeedConfirmationRange.zMinSeedConf            = m_forwardSeedConf_zMin;
  filterCfg.forwardSeedConfirmationRange.zMaxSeedConf            = m_forwardSeedConf_zMax;
  filterCfg.forwardSeedConfirmationRange.rMaxSeedConf            = m_forwardSeedConf_rMax;
  filterCfg.forwardSeedConfirmationRange.nTopForLargeR           = m_forwardSeedConf_nTopForLargeR;
  filterCfg.forwardSeedConfirmationRange.nTopForSmallR           = m_forwardSeedConf_nTopForSmallR;
  filterCfg.forwardSeedConfirmationRange.seedConfMinBottomRadius = m_forwardSeedConf_minBottomRadius;
  filterCfg.forwardSeedConfirmationRange.seedConfMaxZOrigin      = m_forwardSeedConf_maxZOrigin;
  filterCfg.forwardSeedConfirmationRange.minImpactSeedConf       = m_forwardSeedConf_minImpact;

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

        // Extrapolate the fitted track back to the perigee surface at the IP so
        // that the track parameters (in particular D0 and Z0) are expressed
        // there. This reproduces the TrackStateAtIP behaviour from older ACTS
        // and must happen before ACTS2edm4hep_track, which fills the AtIP state
        // from the track-level parameters.
        if (m_extrapPropagator) {
          const Acts::MagneticFieldContext magCtx{};
          Propagator::Options<>            extrapOptions{geoCtx, magCtx};
          extrapOptions.maxSteps = 10000;
          if (m_propagateBackward) {
            extrapOptions.direction = Acts::Direction::Backward();
          }
          auto extrapResult =
              Acts::extrapolateTrackToReferenceSurface(trackTip, *m_perigeeSurface, *m_extrapPropagator, extrapOptions,
                                                       Acts::TrackExtrapolationStrategy::firstOrLast);
          if (!extrapResult.ok()) {
            warning() << "Track extrapolation to perigee failed: " << extrapResult.error() << endmsg;
            continue;
          }
        }

        auto track = ACTSTracking::ACTS2edm4hep_track(trackTip, m_actsGeoSvc->magneticField(), magCache);

        // Extrapolate the fitted track to the calorimeter face and add an
        // AtCalorimeter track state for Pandora / ParticleFlow. Start from the
        // outermost smoothed state that carries a real measurement (closest to
        // the calorimeter), so the extrapolation begins from a genuine fitted
        // hit rather than a hole, outlier or material-only state.
        if (m_caloPropagator) {
          std::optional<Acts::BoundTrackParameters> startParams;
          for (const auto& state : trackTip.trackStatesReversed()) {
            const auto flags = state.typeFlags();
            if (state.hasSmoothed() && flags.test(Acts::TrackStateFlag::HasMeasurement) &&
                !flags.test(Acts::TrackStateFlag::IsOutlier)) {
              startParams.emplace(state.referenceSurface().getSharedPtr(), state.smoothed(), state.smoothedCovariance(),
                                  trackTip.particleHypothesis());
              break;
            }
          }

          if (!startParams) {
            ++m_caloNoStartState;
            debug() << "No measured smoothed state available; no AtCalorimeter state added for this track." << endmsg;
          } else {
            ++m_caloAttempts;
            const Acts::MagneticFieldContext magCtx{};
            const auto caloResult = ACTSTracking::extrapolateToCaloFace(
                *m_caloPropagator, *startParams, m_actsGeoSvc->caloSurfaceGeoIds(), geoCtx, magCtx);

            using ACTSTracking::CaloExtrapolationStatus;
            switch (caloResult.status) {
            case CaloExtrapolationStatus::Ok: {
              ++m_caloOk;
              const Acts::Vector3 caloPos  = caloResult.params->position(geoCtx);
              auto                fieldRes = m_actsGeoSvc->magneticField()->getField(caloPos, magCache);
              const double        Bz       = fieldRes.ok() ? (*fieldRes)[2] / Acts::UnitConstants::T : 0.0;
              auto                caloState =
                  ACTSTracking::ACTS2edm4hep_trackState(edm4hep::TrackState::AtCalorimeter, *caloResult.params, Bz);
              // The calo-face parameters are local to the target surface, so the
              // edm4hep D0/Z0 from the generic conversion are not meaningful here.
              // Express the state at the impact point instead: set the reference
              // point to the global calo-face position (D0 = Z0 = 0 there).
              caloState.referencePoint = edm4hep::Vector3f(caloPos.x(), caloPos.y(), caloPos.z());
              caloState.D0             = 0;
              caloState.Z0             = 0;
              track.addToTrackStates(caloState);
              break;
            }
            case CaloExtrapolationStatus::NotReached:
            case CaloExtrapolationStatus::NoSurfaces:
              ++m_caloNotReached;
              debug() << "Extrapolation to the calorimeter face did not reach a surface; "
                         "no AtCalorimeter state added for this track."
                      << endmsg;
              break;
            case CaloExtrapolationStatus::PropagationError:
              ++m_caloPropFailed;
              debug() << "Extrapolation to the calorimeter face failed during propagation; "
                         "no AtCalorimeter state added for this track."
                      << endmsg;
              break;
            }
          }
        }

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
