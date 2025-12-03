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

#include <memory>

struct ActsTestPropagator final : public k4FWCore::Producer<std::vector<podio::UserDataCollection<double>>()> {
  explicit ActsTestPropagator(const std::string& name, ISvcLocator* svcLoc)
      : Producer(name, svcLoc, {},
                 {KeyValues("OutputCollections", {"step_x", "step_y", "step_z", "step_geoID", "step_size"})}) {}

  ~ActsTestPropagator() = default;

  StatusCode initialize() override;

  std::vector<podio::UserDataCollection<double>> operator()() const override;

  Gaudi::Property<std::string> m_outFileName{this, "StepsOutputFile", "acts_steps.csv",
                                             "Output file for writing step positions and geometry id"};

private:
  SmartIF<IActsGeoSvc> m_actsGeoSvc;
  SmartIF<IGeoSvc>     m_geoSvc;

  std::shared_ptr<Acts::ConstantBField> m_magneticField{nullptr};

  std::unique_ptr<const Acts::Logger> m_actsLogger{nullptr};
};

StatusCode ActsTestPropagator::initialize() {
  m_geoSvc = svcLoc()->service<IGeoSvc>("GeoSvc");
  K4_GAUDI_CHECK(m_geoSvc);

  m_actsGeoSvc = svcLoc()->service<IActsGeoSvc>("ActsGeoSvc");
  K4_GAUDI_CHECK(m_actsGeoSvc);

  m_actsLogger = makeActsGaudiLogger(this);

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

  auto state = propagator.makeState(options);

  const auto startParameters = Acts::BoundTrackParameters::createCurvilinear(
      Acts::Vector4{0, 0, 0, 0}, Acts::Vector3{0, 0.5, 0.5}, 0.5, std::nullopt, Acts::ParticleHypothesis::pion());

  std::vector<podio::UserDataCollection<double>> stepOutputs(5);

  auto initResult = propagator.initialize(state, startParameters);
  if (!initResult.ok()) {
    error() << initResult.error() << endmsg;
    return stepOutputs;
  }
  debug() << "Initialized propagator" << endmsg;

  // Propagate using the propagator
  debug() << "Starting propagation" << endmsg;
  auto resultTmp = propagator.propagate(state);
  if (!resultTmp.ok()) {
    error() << resultTmp.error() << endmsg;
    return stepOutputs;
  }
  debug() << "Done with propagation" << endmsg;

  auto result = propagator.makeResult(std::move(state), resultTmp, options, true);
  if (!result.ok()) {
    error() << result.error() << endmsg;
    return stepOutputs;
  }
  const auto& steppingResults = result.value().get<SteppingLogger::result_type>();

  auto& stepsX      = stepOutputs[0].vec();
  auto& stepsY      = stepOutputs[1].vec();
  auto& stepsZ      = stepOutputs[2].vec();
  auto& stepsGeoID  = stepOutputs[3].vec();
  auto& stepsLength = stepOutputs[4].vec();

  for (const auto& step : steppingResults.steps) {
    stepsX.push_back(step.position.x());
    stepsY.push_back(step.position.y());
    stepsZ.push_back(step.position.z());
    stepsGeoID.push_back(step.geoID.value());
    stepsLength.push_back(step.stepSize.value());
  }

  return stepOutputs;
}

DECLARE_COMPONENT(ActsTestPropagator);
