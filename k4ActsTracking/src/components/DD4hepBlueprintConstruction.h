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
#ifndef K4ACTSTRACKING_DD4HEPBLUEPRINTCONSTRUCTION_H
#define K4ACTSTRACKING_DD4HEPBLUEPRINTCONSTRUCTION_H

#include <Acts/Definitions/Units.hpp>
#include <ActsPlugins/DD4hep/BlueprintBuilder.hpp>

#include <string>

namespace Acts::Experimental {
  class ContainerBlueprintNode;
  class Blueprint;
}  // namespace Acts::Experimental

namespace MuColl {
  namespace MAIA_v0 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder);
  }
}  // namespace MuColl

namespace FCCee {
  namespace ILD_FCCee_v01 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder);
  }

  namespace ILD_FCCee_v02 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder);
  }

  namespace CLD_o2_v07 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder);
  }

}  // namespace FCCee

namespace LUXE {
  namespace LUXE_v0 {
    void populateBlueprint(const std::string& detName, Acts::Experimental::Blueprint& root,
                           ActsPlugins::DD4hep::BlueprintBuilder& builder);
  }
}  // namespace LUXE

#endif  // K4ACTSTRACKING_DD4HEPBLUEPRINTCONSTRUCTION_H
