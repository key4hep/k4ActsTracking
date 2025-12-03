#include "k4ActsTracking/ActsGaudiLogger.h"
#include "k4ActsTracking/IActsGeoSvc.h"

#include <Evaluator/DD4hepUnits.h>
#include <k4FWCore/GaudiChecks.h>
#include <k4Interface/IGeoSvc.h>

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

#include <array>
#include <memory>

struct ActsTestPropagator : public Gaudi::Algorithm {
  explicit ActsTestPropagator(const std::string& name, ISvcLocator* svcLoc);

  StatusCode initialize() override;

  StatusCode execute(const EventContext&) const override;

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

  std::array<double, 3> magneticFieldVector = {0, 0, 0};
  std::array<double, 3> position            = {0, 0, 0};
  m_geoSvc->getDetector()->field().magneticField(position.data(), magneticFieldVector.data());
  debug() << fmt::format("Retrieved magnetic field at position {}: {}", position, magneticFieldVector) << endmsg;
  m_magneticField = std::make_shared<Acts::ConstantBField>(
      Acts::Vector3(magneticFieldVector[0] / dd4hep::tesla * Acts::UnitConstants::T,
                    magneticFieldVector[1] / dd4hep::tesla * Acts::UnitConstants::T,
                    magneticFieldVector[2] / dd4hep::tesla * Acts::UnitConstants::T));

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
  // Navigator::Config navigatorCfg{trackingGeometry()};
  // navigatorCfg.resolvePassive   = false;
  // navigatorCfg.resolveMaterial  = true;
  // navigatorCfg.resolveSensitive = true;

  // Stepper    stepper(m_magneticField);
  // Navigator  navigator(navigatorCfg);
  // Propagator propagator(std::move(stepper), std::move(navigator));

  auto options = PropagatorOptions{Acts::GeometryContext{}, Acts::MagneticFieldContext{}};

  return StatusCode::SUCCESS;
}
