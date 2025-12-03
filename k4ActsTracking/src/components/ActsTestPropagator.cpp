#include "k4ActsTracking/ActsGaudiLogger.h"
#include "k4ActsTracking/IActsGeoSvc.h"

#include <k4FWCore/GaudiChecks.h>
#include <k4Interface/IGeoSvc.h>

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

#include <Gaudi/Algorithm.h>
#include <GaudiKernel/StatusCode.h>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <fstream>
#include <memory>

struct ActsTestPropagator : public Gaudi::Algorithm {
  explicit ActsTestPropagator(const std::string& name, ISvcLocator* svcLoc) : Gaudi::Algorithm(name, svcLoc) {}

  ~ActsTestPropagator() = default;

  StatusCode initialize() override;

  StatusCode execute(const EventContext&) const override;

private:
  SmartIF<IActsGeoSvc> m_actsGeoSvc;
  SmartIF<IGeoSvc>     m_geoSvc;

  std::shared_ptr<Acts::ConstantBField> m_magneticField{nullptr};

  std::unique_ptr<const Acts::Logger> m_actsLogger{nullptr};

  std::ofstream m_outputFile;
};

StatusCode ActsTestPropagator::initialize() {
  m_geoSvc = svcLoc()->service<IGeoSvc>("GeoSvc");
  K4_GAUDI_CHECK(m_geoSvc);

  m_actsGeoSvc = svcLoc()->service<IActsGeoSvc>("ActsGeoSvc");
  K4_GAUDI_CHECK(m_actsGeoSvc);

  m_actsLogger = makeActsGaudiLogger(this);

  return StatusCode::SUCCESS;
}

StatusCode ActsTestPropagator::execute(const EventContext&) const {
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

  auto initResult = propagator.initialize(state, startParameters);
  if (!initResult.ok()) {
    error() << initResult.error() << endmsg;
    return StatusCode::FAILURE;
  }
  debug() << "Initialized propagator" << endmsg;

  // Propagate using the propagator
  debug() << "Starting propagation" << endmsg;
  auto resultTmp = propagator.propagate(state);
  if (!resultTmp.ok()) {
    error() << resultTmp.error() << endmsg;
    return StatusCode::FAILURE;
  }
  debug() << "Done with propagation" << endmsg;

  auto result = propagator.makeResult(std::move(state), resultTmp, options, true);
  if (!result.ok()) {
    error() << result.error() << endmsg;
    return StatusCode::FAILURE
  }

  return StatusCode::SUCCESS;
}

DECLARE_COMPONENT(ActsTestPropagator);
