#include "EmptyAlg.h"

DECLARE_COMPONENT(EmptyAlg)

EmptyAlg::EmptyAlg(const std::string& aName, ISvcLocator* aSvcLoc) : GaudiAlgorithm(aName, aSvcLoc) {}

EmptyAlg::~EmptyAlg() {}

StatusCode EmptyAlg::initialize() { return StatusCode::SUCCESS; }

StatusCode EmptyAlg::execute() {
  std::cout << "HALLO WELT!" << std::endl;
  return StatusCode::SUCCESS;
}

StatusCode EmptyAlg::finalize() { return StatusCode::SUCCESS; }
