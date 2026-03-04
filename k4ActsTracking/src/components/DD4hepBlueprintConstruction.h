#ifndef K4ACTSTRACKING_DD4HEPBLUEPRINTCONSTRUCTION_H
#define K4ACTSTRACKING_DD4HEPBLUEPRINTCONSTRUCTION_H

#include <Acts/Definitions/Units.hpp>
#include <ActsPlugins/DD4hep/BlueprintBuilder.hpp>

#include <string>

namespace Acts::Experimental {
  class ContainerBlueprintNode;
  class Blueprint;
}  // namespace Acts::Experimental

namespace Blueprints {
  using namespace Acts::UnitLiterals;
  void addCylindricalBeampipe(Acts::Experimental::ContainerBlueprintNode& root, double rMax = 10_mm,
                              double halfZ = 1000_mm);
}  // namespace Blueprints

namespace MuColl {
  namespace MAIA_v0 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder);
  }
}  // namespace MuColl

namespace FCCee {
  namespace ILD_FCCee {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder);
  }
  // namespace FCCee
}  // namespace FCCee

#endif  // K4ACTSTRACKING_DD4HEPBLUEPRINTCONSTRUCTION_H
