#include "k4ActsTracking/ActsGaudiLogger.h"
#include "k4ActsTracking/IActsGeoSvc.h"

#include <k4FWCore/GaudiChecks.h>
#include <k4FWCore/Producer.h>
#include <k4Interface/IGeoSvc.h>

#include <podio/UserDataCollection.h>

#include <Acts/EventData/GenericBoundTrackParameters.hpp>
#include <Acts/EventData/ParticleHypothesis.hpp>
#include <Acts/EventData/TrackParameters.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/MagneticField/ConstantBField.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/Material/MaterialInteraction.hpp>
#include <Acts/Propagator/EigenStepper.hpp>
#include <Acts/Propagator/MaterialInteractor.hpp>
#include <Acts/Propagator/Navigator.hpp>
#include <Acts/Propagator/detail/SteppingLogger.hpp>

#include <DD4hep/Detector.h>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <cmath>
#include <memory>
#include <random>

struct ActsTestPropagator final : public k4FWCore::Producer<std::vector<podio::UserDataCollection<double>>()> {
  explicit ActsTestPropagator(const std::string& name, ISvcLocator* svcLoc)
      : Producer(name, svcLoc, {},
                 {KeyValues("OutputCollections", {"step_x", "step_y", "step_z", "step_geoID", "step_size"})}) {}

  ~ActsTestPropagator() = default;

  StatusCode initialize() override;

  std::vector<podio::UserDataCollection<double>> operator()() const override;

  Gaudi::Property<std::string> m_outFileName{this, "StepsOutputFile", "acts_steps.csv",
                                             "Output file for writing step positions and geometry id"};
  Gaudi::Property<int>         m_numTracks{this, "NumTracks", 100, "Number of random tracks to propagate"};
  Gaudi::Property<double>      m_minMomentum{this, "MinMomentum", 10.0, "Minimum particle momentum in GeV"};
  Gaudi::Property<double>      m_maxMomentum{this, "MaxMomentum", 1000.0, "Maximum particle momentum in GeV"};
  Gaudi::Property<double>      m_minEta{this, "MinEta", -2.5, "Minimum pseudorapidity"};
  Gaudi::Property<double>      m_maxEta{this, "MaxEta", 2.5, "Maximum pseudorapidity"};

private:
  SmartIF<IActsGeoSvc> m_actsGeoSvc;
  SmartIF<IGeoSvc>     m_geoSvc;

  std::shared_ptr<Acts::ConstantBField> m_magneticField{nullptr};

  std::unique_ptr<const Acts::Logger> m_actsLogger{nullptr};

  // Random number generator for generating different start parameters
  mutable std::mt19937                           m_gen;
  mutable std::uniform_real_distribution<double> m_posDist;
  mutable std::uniform_real_distribution<double> m_dirDist;
  mutable std::uniform_real_distribution<double> m_qOverPDist;
};

StatusCode ActsTestPropagator::initialize() {
  m_geoSvc = svcLoc()->service<IGeoSvc>("GeoSvc");
  K4_GAUDI_CHECK(m_geoSvc);

  m_actsGeoSvc = svcLoc()->service<IActsGeoSvc>("ActsGeoSvc");
  K4_GAUDI_CHECK(m_actsGeoSvc);

  m_actsLogger = makeActsGaudiLogger(this);

  // Initialize random number generator
  std::random_device rd;
  m_gen.seed(rd());
  m_posDist = std::uniform_real_distribution<double>(-100.0, 100.0);  // Position range in mm

  // Compute distributions from particle momenta and eta direction range
  // Convert eta range to theta range for direction generation
  double minTheta = 2.0 * std::atan(std::exp(-m_maxEta));  // theta from max eta
  double maxTheta = 2.0 * std::atan(std::exp(-m_minEta));  // theta from min eta

  m_dirDist = std::uniform_real_distribution<double>(minTheta, maxTheta);  // theta range

  // q/p distribution based on momentum range (assuming charge ±1)
  double maxQOverP = 1.0 / m_minMomentum;   // 1/GeV
  double minQOverP = -1.0 / m_maxMomentum;  // 1/GeV (negative charge)
  m_qOverPDist     = std::uniform_real_distribution<double>(minQOverP, maxQOverP);

  return StatusCode::SUCCESS;
}

std::vector<podio::UserDataCollection<double>> ActsTestPropagator::operator()() const {
  // Largely taken from ActsExamples/Propagation/PropagatorInterface

  // The step length logger for testing & end of world aborter
  using MaterialInteractor = Acts::MaterialInteractor;
  using SteppingLogger     = Acts::detail::SteppingLogger;
  using EndOfWorld         = Acts::EndOfWorldReached;

  using Stepper    = Acts::EigenStepper<>;
  using Navigator  = Acts::Navigator;
  using Propagator = Acts::Propagator<Stepper, Navigator>;

  using ActorList         = Acts::ActorList<SteppingLogger, MaterialInteractor, EndOfWorld>;
  using PropagatorOptions = Propagator::template Options<ActorList>;

  // Configurations
  Navigator::Config navigatorCfg{m_actsGeoSvc->trackingGeometry()};
  navigatorCfg.resolvePassive   = false;
  navigatorCfg.resolveMaterial  = true;
  navigatorCfg.resolveSensitive = true;

  Stepper    stepper(m_actsGeoSvc->magneticField());
  Navigator  navigator(navigatorCfg, m_actsLogger->cloneWithSuffix(":Nav"));
  Propagator propagator(std::move(stepper), std::move(navigator), m_actsLogger->cloneWithSuffix(":Prop"));

  auto options = PropagatorOptions{Acts::GeometryContext{}, Acts::MagneticFieldContext{}};

  std::vector<podio::UserDataCollection<double>> stepOutputs(5);
  auto&                                          stepsX      = stepOutputs[0].vec();
  auto&                                          stepsY      = stepOutputs[1].vec();
  auto&                                          stepsZ      = stepOutputs[2].vec();
  auto&                                          stepsGeoID  = stepOutputs[3].vec();
  auto&                                          stepsLength = stepOutputs[4].vec();

  for (int i = 0; i < m_numTracks; ++i) {
    // Generate random start position
    Acts::Vector4 startPos{m_posDist(m_gen), m_posDist(m_gen), m_posDist(m_gen), 0};

    // Generate random direction using theta from eta range and uniform phi
    double theta = m_dirDist(m_gen);
    double phi   = std::uniform_real_distribution<double>(0.0, 2.0 * M_PI)(m_gen);

    Acts::Vector3 direction{std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta)};

    // Generate random charge over momentum
    double qOverP = m_qOverPDist(m_gen);

    verbose() << fmt::format(
                     "Track {}: Initial state - Position: ({:.2f}, {:.2f}, {:.2f}) mm, "
                     "Direction: ({:.3f}, {:.3f}, {:.3f}), q/p: {:.4f} 1/GeV",
                     i, startPos.x(), startPos.y(), startPos.z(), direction.x(), direction.y(), direction.z(), qOverP)
              << endmsg;

    const auto startParameters = Acts::BoundTrackParameters::createCurvilinear(
        startPos, direction, qOverP, std::nullopt, Acts::ParticleHypothesis::pion());

    auto state = propagator.makeState(options);

    auto initResult = propagator.initialize(state, startParameters);
    if (!initResult.ok()) {
      warning() << "Failed to initialize propagator for track " << i << ": " << initResult.error() << endmsg;
      continue;
    }

    // Propagate using the propagator
    auto resultTmp = propagator.propagate(state);
    if (!resultTmp.ok()) {
      warning() << "Propagation failed for track " << i << ": " << resultTmp.error() << endmsg;
      continue;
    }

    auto result = propagator.makeResult(std::move(state), resultTmp, options, true);
    if (!result.ok()) {
      warning() << "Failed to make result for track " << i << ": " << result.error() << endmsg;
      continue;
    }

    const auto& steppingResults = result.value().get<SteppingLogger::result_type>();

    // Store step results from this track
    for (const auto& step : steppingResults.steps) {
      stepsX.push_back(step.position.x());
      stepsY.push_back(step.position.y());
      stepsZ.push_back(step.position.z());
      stepsGeoID.push_back(step.geoID.value());
      stepsLength.push_back(step.stepSize.value());
    }
  }

  debug() << "Completed propagation of " << m_numTracks << " tracks with total " << stepsX.size() << " steps" << endmsg;

  return stepOutputs;
}

DECLARE_COMPONENT(ActsTestPropagator);
