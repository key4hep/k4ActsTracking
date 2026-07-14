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
#include "k4ActsTracking/CKFRunner.hxx"
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
#include <Acts/EventData/SeedContainer.hpp>
#include <Acts/EventData/SpacePointContainer.hpp>
#include <Acts/EventData/TrackContainer.hpp>
#include <Acts/EventData/TrackStateType.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>
#include <Acts/EventData/VectorTrackContainer.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Propagator/EigenStepper.hpp>
#include <Acts/Propagator/Navigator.hpp>
#include <Acts/Propagator/Propagator.hpp>
#include <Acts/Seeding/BroadTripletSeedFilter.hpp>
#include <Acts/Seeding/CylindricalSpacePointGrid.hpp>
#include <Acts/Seeding/DoubletSeedFinder.hpp>
#include <Acts/Seeding/EstimateTrackParamsFromSeed.hpp>
#include <Acts/Seeding/TripletSeedFinder.hpp>
#include <Acts/Seeding/TripletSeeder.hpp>
#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/Surfaces/PlaneSurface.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>
#include <Acts/TrackFinding/MeasurementSelector.hpp>
#include <Acts/TrackFinding/TrackStateCreator.hpp>
#include <Acts/TrackFitting/GainMatrixUpdater.hpp>
#include <Acts/Utilities/Logger.hpp>
#include <Acts/Utilities/RangeXD.hpp>
#include <Acts/Utilities/TrackHelpers.hpp>
#include <Acts/Utilities/VectorHelpers.hpp>

// TBB
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>

#include <fmt/ostream.h>

// Standard
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <span>
#include <string>
#include <vector>

using namespace Acts::UnitLiterals;

template <> struct fmt::formatter<podio::ObjectID> : fmt::ostream_formatter {};

namespace {
  /// Build straight-line bound track parameters for a telescope seed.
  ///
  /// In a field-free tracker (the ACTS constant field is 0 for a telescope like
  /// LUXE) the trajectory through the planar layers is a straight line, so the
  /// conformal/helix estimate used by the cylindrical seeder does not apply.
  /// The seed direction is taken from the bottom->top space points; the momentum
  /// magnitude is not measurable without curvature, so a nominal value is used
  /// (it does not affect the straight-line propagation, only the covariance
  /// scale and the reported q/p).
  std::optional<Acts::BoundTrackParameters> estimateStraightLineSeedParameters(
      const Acts::GeometryContext& geoCtx, const Acts::Surface& bottomSurface, const Acts::Vector3& bottomPos,
      const Acts::Vector3& topPos, double t0, double nominalP, double errPos, double errPhi, double errLambda,
      double errRelP, double errTime) {
    const Acts::Vector3 dir = (topPos - bottomPos).normalized();

    // The bottom position is derived from the surface, so it lies on it; use a
    // generous tolerance to guard against round-off in the global<->local map.
    Acts::Result<Acts::Vector2> local =
        bottomSurface.globalToLocal(geoCtx, bottomPos, dir, 1 * Acts::UnitConstants::mm);
    if (!local.ok()) {
      return std::nullopt;
    }

    Acts::BoundVector params   = Acts::BoundVector::Zero();
    params[Acts::eBoundLoc0]   = (*local)[0];
    params[Acts::eBoundLoc1]   = (*local)[1];
    params[Acts::eBoundPhi]    = Acts::VectorHelpers::phi(dir);
    params[Acts::eBoundTheta]  = Acts::VectorHelpers::theta(dir);
    params[Acts::eBoundQOverP] = 1.0 / nominalP;  // charge sign is irrelevant at B = 0
    params[Acts::eBoundTime]   = t0;

    Acts::BoundMatrix cov = ACTSTracking::makeInitialCovariance(nominalP, errPos, errPhi, errLambda, errRelP, errTime);
    return Acts::BoundTrackParameters(bottomSurface.getSharedPtr(), params, cov, Acts::ParticleHypothesis::pion());
  }
}  // namespace

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
  CKFTrackingAlg(const std::string& name, ISvcLocator* svcLoc);

  StatusCode initialize() override;

  StatusCode finalize() override;

  std::tuple<edm4hep::TrackCollection, edm4hep::TrackCollection> operator()(
      const edm4hep::TrackerHitPlaneCollection&             trackerHitCollection,
      const edm4hep::TrackerHitSimTrackerHitLinkCollection& trackerHitRelations) const override;

  // ----- private helpers ---------------------------------------------------
private:
  /// Minimal per-hit information needed to build the seeding space points. The
  /// global position and rho/z variances are computed during the hit loop; the
  /// cylindrical seeder additionally uses r/phi/varR/varZ, while the telescope
  /// seeder only needs the global position and source link.
  struct SeedInput {
    float                    x, y, z, r, phi;
    float                    varR, varZ;
    ACTSTracking::SourceLink sourceLink;
  };

  /// Convert the seeds found by the triplet seeder into ACTS bound track
  /// parameters and create the corresponding edm4hep seed tracks. The space
  /// point indices stored in @p seeds reference @p spacePoints.
  std::vector<Acts::BoundTrackParameters> seedsToParameters(const Acts::SeedContainer&          seeds,
                                                            const Acts::SpacePointContainer&    spacePoints,
                                                            const ACTSTracking::HitContainer&   hits,
                                                            edm4hep::TrackCollection&           seedCollection,
                                                            Acts::MagneticFieldProvider::Cache& magCache) const;

  /// Straight-line telescope seeding for field-free planar geometries (e.g.
  /// LUXE). Groups the seed-selected space points into layers along z, forms
  /// collinear triplets, builds straight-line seed parameters + edm4hep seed
  /// tracks and, when RunCKF is set, runs the CKF over them. Only used when
  /// SeedingMode == "Telescope"; the cylindrical clients never reach it.
  void runTelescopeSeeding(const std::vector<SeedInput>&             seedInputs,
                           const ACTSTracking::MeasurementContainer& measurements,
                           const ACTSTracking::SourceLinkContainer& sourceLinks, const ACTSTracking::HitContainer& hits,
                           edm4hep::TrackCollection& seedCollection, edm4hep::TrackCollection& trackCollection,
                           Acts::MagneticFieldProvider::Cache& magCache) const;

  // ----- Gaudi properties --------------------------------------------------

  /// @name Run control
  ///@{
  Gaudi::Property<bool> m_runCKF{this, "RunCKF", true, "Run tracking using CKF. False means stop at seeding."};
  Gaudi::Property<bool> m_propagateBackward{this, "PropagateBackward", false, "Extrapolates tracks towards beamline."};
  Gaudi::Property<bool> m_extrapolateToCalo{
      this, "ExtrapolateToCalo", true,
      "Extrapolate fitted tracks to the calorimeter face and add an AtCalorimeter track state."};
  ///@}

  /// @name Seeding mode
  ///@{
  /// Selects the seeding strategy. "Cylindrical" (default) is the collider/
  /// barrel triplet helix seeder and is used by all existing clients. "Telescope"
  /// switches to the straight-line, layer-based seeder for field-free planar
  /// geometries (e.g. LUXE); it does not touch the cylindrical code path.
  Gaudi::Property<std::string> m_seedingMode{this, "SeedingMode", "Cylindrical",
                                             "Seeding strategy: \"Cylindrical\" (default) or \"Telescope\"."};
  ///@}

  /// @name Telescope seeding configuration (only used when SeedingMode == "Telescope")
  ///@{
  Gaudi::Property<float> m_telescope_layerZTolerance{
      this, "Telescope_LayerZTolerance", 20.0,
      "Maximum |dz| [mm] between space points assigned to the same telescope layer."};
  Gaudi::Property<float> m_telescope_collinearityCut{
      this, "Telescope_CollinearityCut", 2.0,
      "Maximum transverse distance [mm] of the middle space point from the bottom-top line for a triplet seed."};
  Gaudi::Property<float> m_telescope_nominalMomentum{
      this, "Telescope_NominalMomentum", 5.0,
      "Nominal seed momentum [GeV] used for the straight-line (field-free) seed parameters."};
  Gaudi::Property<float> m_telescope_referenceZ{
      this, "Telescope_ReferenceZ", 0.0,
      "z [mm] of the beam-perpendicular reference plane the fitted tracks are extrapolated to for the AtIP track "
      "state. Replaces the beamline perigee, which is unreachable for beam-parallel telescope tracks."};
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
  Gaudi::Property<double>  m_CKF_chi2CutOffOutlier{
      this, "CKF_Chi2CutOffOutlier", std::numeric_limits<double>::max(),
      "Maximum local chi2 for a failing hit to be kept as an outlier; above this it becomes a hole."};

  /// @name CKF branch stopper (early termination of candidate branches)
  ///@{
  Gaudi::Property<bool> m_useBranchStopper{this, "UseBranchStopper", false, "Enable the CKF branch stopper."};
  Gaudi::Property<int>  m_bsMaxHoles{this, "BranchStopper_MaxHoles", 2, "Stop a branch above this many holes."};
  Gaudi::Property<int> m_bsMaxOutliers{this, "BranchStopper_MaxOutliers", 2, "Stop a branch above this many outliers."};
  Gaudi::Property<int> m_bsMinMeasurements{
      this, "BranchStopper_MinMeasurements", 6,
      "When stopping, keep the branch if it has at least this many measurements, otherwise drop it."};
  Gaudi::Property<double> m_bsPtMin{this, "BranchStopper_PtMin", 0.0,
                                    "Drop a branch with |pT| (GeV) below this; <=0 disables."};
  Gaudi::Property<int>    m_bsPtMinMeasurements{this, "BranchStopper_PtMinMeasurements", 3,
                                             "Minimum measurements before the pT branch stop is applied."};
  ///@}
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

  // Shared Combinatorial Kalman Filter, built once in initialize() (the
  // propagators/CKF depend only on geometry+field) and reused read-only across
  // events and threads.
  std::optional<ACTSTracking::CKFRunner> m_ckfRunner{};

  // Calorimeter-face extrapolation monitoring, updated by the shared CKFRunner
  // and summarised in finalize().
  ACTSTracking::CaloExtrapMonitor m_caloMonitor{};

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

  if (m_seedingMode.value() != "Cylindrical" && m_seedingMode.value() != "Telescope") {
    error() << "Unknown SeedingMode '" << m_seedingMode.value() << "'; expected \"Cylindrical\" or \"Telescope\"."
            << endmsg;
    return StatusCode::FAILURE;
  }

  // Apply deltaR fallback defaults
  if (m_seedFinding_deltaRMinTop == 0.f)
    m_seedFinding_deltaRMinTop = m_seedFinding_deltaRMin;
  if (m_seedFinding_deltaRMaxTop == 0.f)
    m_seedFinding_deltaRMaxTop = m_seedFinding_deltaRMax;
  if (m_seedFinding_deltaRMinBottom == 0.f)
    m_seedFinding_deltaRMinBottom = m_seedFinding_deltaRMin;
  if (m_seedFinding_deltaRMaxBottom == 0.f)
    m_seedFinding_deltaRMaxBottom = m_seedFinding_deltaRMax;

  // The CKF, IP-perigee and calorimeter-face propagators are built per event by
  // the shared ACTSTracking::CKFRunner (they are cheap to wire up and bind to
  // the event-local measurements/source links). Warn once here if calorimeter
  // extrapolation was requested but the geometry provides no calo-face surfaces.
  if (m_extrapolateToCalo && m_actsGeoSvc->caloSurfaceGeoIds().empty()) {
    warning() << "ExtrapolateToCalo requested but ActsGeoSvc provides no calorimeter-face surfaces; "
                 "no AtCalorimeter track states will be produced."
              << endmsg;
  }

  // In telescope mode the tracks run almost parallel to the beamline, so the
  // default perigee (a line along z) is unreachable. Extrapolate the AtIP state
  // to a plane perpendicular to the beam at Telescope_ReferenceZ instead; every
  // forward track crosses it, and D0/Z0 then read as the track's (x, y) there.
  std::shared_ptr<const Acts::Surface> referenceSurface;  // null => CKFRunner keeps the beamline perigee
  if (m_seedingMode.value() == "Telescope") {
    // Unbounded plane at z = Telescope_ReferenceZ with its normal along the beam
    // (identity rotation => local z = global z), so any forward track crosses it.
    Acts::Transform3 refTransform = Acts::Transform3::Identity();
    refTransform.translation()    = Acts::Vector3(0, 0, m_telescope_referenceZ * Acts::UnitConstants::mm);
    referenceSurface              = Acts::Surface::makeShared<Acts::PlaneSurface>(refTransform);
  }

  m_ckfRunner.emplace(*m_actsGeoSvc,
                      ACTSTracking::CKFRunner::Config{.chi2CutOff            = m_CKF_chi2CutOff,
                                                      .numMeasurementsCutOff = m_CKF_numMeasurementsCutOff,
                                                      .chi2CutOffOutlier     = m_CKF_chi2CutOffOutlier,
                                                      .propagateBackward     = m_propagateBackward,
                                                      .extrapolateToCalo     = m_extrapolateToCalo,
                                                      .useBranchStopper      = m_useBranchStopper,
                                                      .bsMaxHoles            = m_bsMaxHoles,
                                                      .bsMaxOutliers         = m_bsMaxOutliers,
                                                      .bsMinMeasurements     = m_bsMinMeasurements,
                                                      .bsPtMin               = m_bsPtMin,
                                                      .bsPtMinMeasurements   = m_bsPtMinMeasurements,
                                                      .referenceSurface      = referenceSurface});

  return StatusCode::SUCCESS;
}

StatusCode CKFTrackingAlg::finalize() {
  if (m_extrapolateToCalo) {
    info() << m_caloMonitor.summary() << endmsg;
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

  ACTSTracking::SourceLinkContainer  sourceLinks;
  ACTSTracking::MeasurementContainer measurements;
  ACTSTracking::HitContainer         hits;
  std::vector<SeedInput>             seedInputs;

  // Build measurements + source links for all hits (shared with the other CKF
  // algorithms); for seed-selected surfaces also record a seeding space point
  // (global position + rho/z variance) for the triplet seeder.
  ACTSTracking::prepareTrackerHits(
      *this, *m_actsGeoSvc, geoCtx, trackerHitCollection, measurements, sourceLinks, hits, m_numThreads.value(),
      [&](const edm4hep::TrackerHitPlane& hit, const ACTSTracking::SourceLink& sourceLink,
          const Acts::Vector3& globalPos, const Acts::Surface& surface, const Acts::SquareMatrix2& localCov) {
        if (!m_seedSelector.accept(hit.getCellID())) {
          return;
        }
        Acts::RotationMatrix3 rotLocalToGlobal = surface.referenceFrame(geoCtx, globalPos, {0, 0, 0});

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
      });

  debug() << fmt::format("Created {} sourceLinks and {} space points for seeding", sourceLinks.size(),
                         seedInputs.size())
          << endmsg;

  Acts::MagneticFieldProvider::Cache magCache = m_actsGeoSvc->magneticField()->makeCache(magCtx);

  // Telescope (straight-line) seeding path. Kept fully separate from the
  // cylindrical seeder below so collider/barrel clients see no behaviour change.
  if (m_seedingMode.value() == "Telescope") {
    runTelescopeSeeding(seedInputs, measurements, sourceLinks, hits, seedCollection, trackCollection, magCache);
    debug() << "Track Collection Size: " << trackCollection.size() << endmsg;
    return std::make_tuple(std::move(seedCollection), std::move(trackCollection));
  }

  static const Acts::Vector3 zeropos(0, 0, 0);

  const float bFieldInZ       = (*m_actsGeoSvc->magneticField()->getField(zeropos, magCache))[2];
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

  Acts::CylindricalSpacePointGrid grid(gridCfg, Acts::getDefaultLogger("CKFSeedingGrid", Acts::Logging::WARNING));

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

    if (!m_runCKF)
      return;

    m_ckfRunner->findTracks(*this, measurements, sourceLinks, hits, paramseeds, localMagCache, trackCollection,
                            m_trackMutex, &m_caloMonitor);
  };

  tbb::task_arena arena(m_numThreads.value());
  if (m_numThreads > 1) {
    arena.execute([&] { tbb::parallel_for(tbb::blocked_range<size_t>(0, groups.size()), parallelSeedingAndTracking); });
  } else {
    parallelSeedingAndTracking(tbb::blocked_range<size_t>(0, groups.size()));
  }

  debug() << "Track Collection Size: " << trackCollection.size() << endmsg;
  return std::make_tuple(std::move(seedCollection), std::move(trackCollection));
}

std::vector<Acts::BoundTrackParameters> CKFTrackingAlg::seedsToParameters(
    const Acts::SeedContainer& seeds, const Acts::SpacePointContainer& spacePoints,
    const ACTSTracking::HitContainer& hits, edm4hep::TrackCollection& seedCollection,
    Acts::MagneticFieldProvider::Cache& magCache) const {
  const Acts::GeometryContext geoCtx = Acts::GeometryContext::dangerouslyDefaultConstruct();

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
    const Acts::Surface*            surface  = m_actsGeoSvc->trackingGeometry()->findSurface(bottomSL.geometryId());
    if (surface == nullptr) {
      warning() << "Surface with geoID " << bottomSL.geometryId() << " not found in tracking geometry" << endmsg;
      continue;
    }

    std::optional<Acts::BoundTrackParameters> paramseed = ACTSTracking::estimateSeedParameters(
        *this, *m_actsGeoSvc, geoCtx, *surface, position(bottomSp), position(middleSp), position(topSp),
        hits[bottomSL.index()].getTime(), magCache, m_initialTrackError_pos, m_initialTrackError_phi,
        m_initialTrackError_lambda, m_initialTrackError_relP, m_initialTrackError_time);
    if (!paramseed) {
      continue;
    }
    paramseeds.push_back(*paramseed);

    auto seedTrackState = ACTSTracking::makeSeedTrackState(*this, *m_actsGeoSvc, geoCtx, *paramseed, magCache);

    {
      std::lock_guard<std::mutex> lock(m_seedMutex);
      auto                        seedTrack = seedCollection.create();
      seedTrack.addToTrackerHits(hits[bottomSL.index()]);
      seedTrack.addToTrackerHits(hits[sourceLinkOf(middleSp).index()]);
      seedTrack.addToTrackerHits(hits[sourceLinkOf(topSp).index()]);
      seedTrack.addToTrackStates(seedTrackState);
    }

    debug() << "Seed Parameters" << std::endl << *paramseed << endmsg;
  }

  debug() << "Seeds found: " << paramseeds.size() << endmsg;
  return paramseeds;
}

void CKFTrackingAlg::runTelescopeSeeding(const std::vector<SeedInput>&             seedInputs,
                                         const ACTSTracking::MeasurementContainer& measurements,
                                         const ACTSTracking::SourceLinkContainer&  sourceLinks,
                                         const ACTSTracking::HitContainer&         hits,
                                         edm4hep::TrackCollection&                 seedCollection,
                                         edm4hep::TrackCollection&                 trackCollection,
                                         Acts::MagneticFieldProvider::Cache&       magCache) const {
  const Acts::GeometryContext geoCtx = Acts::GeometryContext::dangerouslyDefaultConstruct();

  // -------------------------------------------------------------------------
  // 1. Group the seed space points into layers along the beam (z) axis. The
  //    points are sorted by z and split into a new layer whenever the gap to
  //    the previous point exceeds the layer tolerance (telescope layers are
  //    well separated in z, so this greedy clustering is unambiguous).
  // -------------------------------------------------------------------------
  std::vector<std::size_t> order(seedInputs.size());
  std::iota(order.begin(), order.end(), 0);
  std::ranges::sort(order, [&](std::size_t a, std::size_t b) { return seedInputs[a].z < seedInputs[b].z; });

  std::vector<std::vector<std::size_t>> layers;
  for (std::size_t k : order) {
    if (layers.empty() ||
        std::abs(seedInputs[k].z - seedInputs[layers.back().back()].z) > m_telescope_layerZTolerance) {
      layers.emplace_back();
    }
    layers.back().push_back(k);
  }

  if (layers.size() < 3) {
    debug() << "Telescope seeding: only " << layers.size() << " layer(s) with seed hits (need >= 3); no seeds."
            << endmsg;
    return;
  }

  const double collinearityCut = m_telescope_collinearityCut;
  const double nominalP        = m_telescope_nominalMomentum * Acts::UnitConstants::GeV;

  auto pos = [&](std::size_t i) { return Acts::Vector3(seedInputs[i].x, seedInputs[i].y, seedInputs[i].z); };

  std::vector<Acts::BoundTrackParameters> paramseeds;

  // -------------------------------------------------------------------------
  // 2. Form straight-line triplets over consecutive layer triplets. For each
  //    bottom/top pair the middle hit closest to the bottom->top line is kept,
  //    accepted only if within the collinearity cut. This replaces the helix
  //    compatibility used by the cylindrical seeder.
  // -------------------------------------------------------------------------
  for (std::size_t l = 0; l + 2 < layers.size(); ++l) {
    const std::vector<std::size_t>& botLayer = layers[l];
    const std::vector<std::size_t>& midLayer = layers[l + 1];
    const std::vector<std::size_t>& topLayer = layers[l + 2];

    for (std::size_t bi : botLayer) {
      const Acts::Vector3 bottom = pos(bi);
      for (std::size_t ti : topLayer) {
        const Acts::Vector3 top = pos(ti);
        const double        dz  = top.z() - bottom.z();
        if (std::abs(dz) < 1e-6) {
          continue;
        }

        double      bestResidual = collinearityCut;
        std::size_t bestMid      = std::numeric_limits<std::size_t>::max();
        for (std::size_t mi : midLayer) {
          const Acts::Vector3 mid   = pos(mi);
          const double        t     = (mid.z() - bottom.z()) / dz;
          const Acts::Vector3 pred  = bottom + t * (top - bottom);
          const double        resid = std::hypot(mid.x() - pred.x(), mid.y() - pred.y());
          if (resid < bestResidual) {
            bestResidual = resid;
            bestMid      = mi;
          }
        }
        if (bestMid == std::numeric_limits<std::size_t>::max()) {
          continue;
        }

        const ACTSTracking::SourceLink& bottomSL = seedInputs[bi].sourceLink;
        const Acts::Surface*            surface  = m_actsGeoSvc->trackingGeometry()->findSurface(bottomSL.geometryId());
        if (surface == nullptr) {
          warning() << "Surface with geoID " << bottomSL.geometryId() << " not found in tracking geometry" << endmsg;
          continue;
        }

        std::optional<Acts::BoundTrackParameters> paramseed = estimateStraightLineSeedParameters(
            geoCtx, *surface, bottom, top, hits[bottomSL.index()].getTime(), nominalP, m_initialTrackError_pos,
            m_initialTrackError_phi, m_initialTrackError_lambda, m_initialTrackError_relP, m_initialTrackError_time);
        if (!paramseed) {
          continue;
        }
        paramseeds.push_back(*paramseed);

        auto seedTrackState = ACTSTracking::makeSeedTrackState(*this, *m_actsGeoSvc, geoCtx, *paramseed, magCache);
        {
          std::lock_guard<std::mutex> lock(m_seedMutex);
          auto                        seedTrack = seedCollection.create();
          seedTrack.addToTrackerHits(hits[bottomSL.index()]);
          seedTrack.addToTrackerHits(hits[seedInputs[bestMid].sourceLink.index()]);
          seedTrack.addToTrackerHits(hits[seedInputs[ti].sourceLink.index()]);
          seedTrack.addToTrackStates(seedTrackState);
        }
      }
    }
  }

  debug() << "Telescope seeding: found " << paramseeds.size() << " seeds across " << layers.size() << " layers."
          << endmsg;

  if (!m_runCKF) {
    return;
  }

  m_ckfRunner->findTracks(*this, measurements, sourceLinks, hits, paramseeds, magCache, trackCollection, m_trackMutex,
                          &m_caloMonitor);
}

DECLARE_COMPONENT(CKFTrackingAlg);
