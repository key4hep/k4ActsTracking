#include "ActsAlg.h"

#include <DD4hep/DetFactoryHelper.h>
#include <DD4hep/Factories.h>

DECLARE_COMPONENT(ActsAlg)

ActsAlg::ActsAlg(const std::string& aName, ISvcLocator* aSvcLoc) : GaudiAlgorithm(aName, aSvcLoc) {}

ActsAlg::~ActsAlg() {}

StatusCode ActsAlg::initialize() {
  info() << "Initializing the ACTS algorithm" << endmsg;
  int                 argc = 0;
  char*               argv = {};
  dd4hep::PluginDebug dbg;
  std::string         name   = "k4acts_addActsExtensions";
  long                result = dd4hep::PluginService::Create<long>(name, &dd4hep::Detector::getInstance(), argc, &argv);
  if (0 == result) {
    error() << "Failed to locate plugin " << name << "\n" << dbg.missingFactory(name) << endmsg;
    return StatusCode::FAILURE;
  }

  info() << "Successfully loaded the plugin!" << endmsg;
  return StatusCode::SUCCESS;
}

StatusCode ActsAlg::execute() { return StatusCode::SUCCESS; }

StatusCode ActsAlg::finalize() { return StatusCode::SUCCESS; }
