#include "k4ActsTracking/ActsGaudiLogger.h"
#include "k4ActsTracking/IActsGeoSvc.h"

#include <k4FWCore/GaudiChecks.h>

#include <Acts/Material/MaterialInteraction.hpp>
#include <Acts/Propagator/MaterialInteractor.hpp>
#include <Acts/Propagator/detail/SteppingLogger.hpp>

#include <Gaudi/Algorithm.h>
#include <GaudiKernel/StatusCode.h>

struct ActsTestPropagator : public Gaudi::Algorithm {
  explicit ActsTestPropagator(const std::string& name, ISvcLocator* svcLoc);

  StatusCode initialize() override;

  StatusCode execute(const EventContext&) const override;

private:
  SmartIF<IActsGeoSvc> m_actsGeoSvc;

  std::unique_ptr<const Acts::Logger> m_actsLogger{nullptr};
};

StatusCode ActsTestPropagator::initialize() {
  m_actsGeoSvc = svcLoc()->service<IActsGeoSvc>("ActsGeoSvc");
  K4_GAUDI_CHECK(m_actsGeoSvc);

  m_actsLogger = makeActsGaudiLogger(this);

  return StatusCode::SUCCESS;
}

StatusCode ActsTestPropagator::execute(const EventContext&) const {
  // The step length logger for testing & end of world aborter
  using MaterialInteractor = Acts::MaterialInteractor;
  using SteppingLogger     = Acts::detail::SteppingLogger;
  using EndOfWorld         = Acts::EndOfWorldReached;

  return StatusCode::SUCCESS;
}
