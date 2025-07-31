#ifndef ACTSSeededCKFTrackingAlg_h
#define ACTSSeededCKFTrackingAlg_h 1

// edm4hep
#include <edm4hep/TrackCollection.h>
#include <edm4hep/TrackerHitPlaneCollection.h>

// gaudi
#include <Gaudi/Property.h>

// ACTS
#include <Acts/Definitions/Units.hpp>

// ROOT
#include <TH1.h>

// ACTSTracking
#include "ACTSAlgBase.hxx"
#include "GeometryIdSelector.hxx"

// Standard
#include <string>
#include <vector>

using namespace Acts::UnitLiterals;

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
        std::tuple<edm4hep::TrackCollection, 
		           edm4hep::TrackCollection> operator()(const edm4hep::TrackerHitPlaneCollection& trackerHitCollection) const;

protected:
	/**
	 * @brief Run Specifc Settings
	 */
	///@{
	Gaudi::Property<bool> m_runCKF{this, "RunCKF", true, "Run tracking using CKF. False means stop at the seeding stage."};
	Gaudi::Property<bool> m_propagateBackward{this, "PropagateBackward", false, "Extrapolates tracks towards beamline."};
	///@}
	
	/**
	 * @brief Seed finding configuration
	 */
	///@{
	Gaudi::Property<float> m_seedFinding_rMax{this, "SeedFinding_RMax", 150, "Maximum radius of hits to consider."};
	Gaudi::Property<float> m_seedFinding_deltaRMin{this, "SeedFinding_DeltaRMin", 5, "Minimum dR between hits in a seed."};
	Gaudi::Property<float> m_seedFinding_deltaRMax{this, "SeedFinding_DeltaRMax", 80, "Maximum dR between hits in a seed."};
	Gaudi::Property<float> m_seedFinding_deltaRMinTop{this, "SeedFinding_DeltaRMinTop", 0.f, "Minimum dR between the reference hit and outer ones in a seed."};
	Gaudi::Property<float> m_seedFinding_deltaRMaxTop{this, "SeedFinding_DeltaRMaxTop", 0.f, "Maximum dR between the reference hit and outer ones in a seed."};
	Gaudi::Property<float> m_seedFinding_deltaRMinBottom{this, "SeedFinding_DeltaRMinBottom", 0.f, "Minimum dR between the reference hit and inner ones in a seed."};
	Gaudi::Property<float> m_seedFinding_deltaRMaxBottom{this, "SeedFinding_DeltaRMaxBottom", 0.f, "Maximum dR between the reference hit and inner ones in a seed."};
	Gaudi::Property<float> m_seedFinding_collisionRegion{this, "SeedFinding_CollisionRegion", 75.0, "Size of the collision region in one direction (assumed symmetric)."};
	Gaudi::Property<float> m_seedFinding_zMax{this, "SeedFinding_ZMax", 600.0, "Maximum z of hits to consider."};
	Gaudi::Property<float> m_seedFinding_sigmaScattering{this, "SeedFinding_SigmaScattering", 50.0, "Number of sigmas to allow in scattering angle."};
	Gaudi::Property<float> m_seedFinding_radLengthPerSeed{this, "SeedFinding_RadLengthPerSeed", 0.1, "Average radiation length per seed."};
	Gaudi::Property<float> m_seedFinding_minPt{this, "SeedFinding_MinPt", 500.0, "Minimum pT of tracks to seed."};
	Gaudi::Property<float> m_seedFinding_impactMax{this, "SeedFinding_ImpactMax", 3.0 * Acts::UnitConstants::mm, "Maximum d0 of tracks to seed."};

	std::vector<std::string> default_value;	// Not sure if this is a needed step, but this broke when I did it inline.
	Gaudi::Property<std::vector<std::string>> m_seedFinding_zBinEdges{this, "SeedFinding_zBinEdges", default_value, "Bins placement along Z for seeding."};
	Gaudi::Property<int> m_zTopBinLen{this, "SeedFinding_zTopBinLen", 1, "Number of top bins along Z for seeding."};
	Gaudi::Property<int> m_zBottomBinLen{this, "SeedFinding_zBottomBinLen", 1, "Number of bottom bins along Z for seeding."};
	Gaudi::Property<int> m_phiTopBinLen{this, "SeedFinding_phiTopBinLen", 1, "Number of top bins along phi for seeding."};
	Gaudi::Property<int> m_phiBottomBinLen{this, "SeedFinding_phiBottomBinLen", 1, "Number of bottom bins along phi for seeding."};
	///@}
	
	/**
	 * @brief Track fit parameters
	 */
	///@{
	Gaudi::Property<double> m_initialTrackError_pos{this, "InitialTrackError_Pos", 10_um, "Track error estimate, local position (mm)."};
	Gaudi::Property<double> m_initialTrackError_phi{this, "InitialTrackError_Phi", 1_degree, "Track error estimate, phi (radians)."};
	Gaudi::Property<double> m_initialTrackError_relP{this, "InitialTrackError_RelP", 0.25, "Track error estimate, momentum component (relative)."};
	Gaudi::Property<double> m_initialTrackError_lambda{this, "InitialTrackError_Lambda", 1_degree, "Track error estimate, lambda (radians)."};
	Gaudi::Property<double> m_initialTrackError_time{this, "InitialTrackError_Time", 100 * Acts::UnitConstants::ns, "Track error estimate, time (sec)."};
	
	Gaudi::Property<double> m_CKF_chi2CutOff{this, "CKF_Chi2CutOff", 15, "Maximum local chi2 contribution."};
	Gaudi::Property<int32_t> m_CKF_numMeasurementsCutOff{this, "CKF_NumMeasurementsCutOff", 10, "Maximum number of associated measurements on a single surface."};
	///@}
	
	/**
	 * @brief Seeding configuration
	 */
	///@{
	Gaudi::Property<std::vector<std::string>> m_seedingLayers{this, "SeedingLayers", default_value, "Layers to use for seeding in vector."};
	ACTSTracking::GeometryIdSelector m_seedGeometrySelection;
	///@}

	/**
 	 * @brief Timing Histograms
 	 */ 
	///@{
	TH1* m_histHitSetUp;
	TH1* m_histEntireReco;
	TH1* m_histSeedFinding;
	TH1* m_histTrackBuild;
	///@}
	//uint32_t m_fitFails; // Counting fails across events is not parallization friendly :(
};

#endif
