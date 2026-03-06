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
#ifndef K4ACTSTRACKING_ACTSGEOSVC_H
#define K4ACTSTRACKING_ACTSGEOSVC_H

#include "k4ActsTracking/IActsGeoSvc.h"

#include <Gaudi/Property.h>
#include <k4Interface/IGeoSvc.h>

#include <Parsers/Primitives.h>

#include <ActsPlugins/DD4hep/BlueprintBuilder.hpp>

#include "GaudiKernel/Service.h"

#include <memory>
#include <string>
#include <unordered_map>

namespace Acts {
  class TrackingGeometry;
  class MagneticFieldProvider;
  class Surface;
  namespace Experimental {
    class Blueprint;
  }
}  // namespace Acts

namespace dd4hep {
  class Detector;
}

class ActsGeoSvc : public extends<Service, IActsGeoSvc> {
public:
  std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry() const override;

  std::shared_ptr<const Acts::MagneticFieldProvider> magneticField() const override;

  ActsGeoSvc(const std::string& name, ISvcLocator* svcLoc);

  ~ActsGeoSvc() = default;

  StatusCode initialize() override;

  Gaudi::Property<std::string> m_objDumpFileName{this, "ObjVisFileName", "dump_acts_geo.obj",
                                                 "Name of the 3D visualization file"};
  Gaudi::Property<bool>        m_dumpVisualization{this, "DumpVisualization", false,
                                            "Whether or not to create a 3D visualization dump"};

  const CellIDSurfaceMap& cellIdToSurfaceMap() const override { return m_cellIDToSurface; }

private:
  using BlueprintBuilder = ActsPlugins::DD4hep::BlueprintBuilder;

  using BlueprintPopulationFunc = void(const std::string&, Acts::Experimental::Blueprint&, BlueprintBuilder&);

  dd4hep::Detector*                                         m_dd4hepGeo{nullptr};
  SmartIF<IGeoSvc>                                          m_geoSvc;
  std::shared_ptr<const Acts::TrackingGeometry>             m_trackingGeo{nullptr};
  std::shared_ptr<const Acts::MagneticFieldProvider>        m_magneticField{nullptr};
  std::unordered_map<dd4hep::CellID, const Acts::Surface*>  m_cellIDToSurface{};
  std::unordered_map<std::string, BlueprintPopulationFunc*> m_bluePrintPopulationFuncs{};
};

inline std::shared_ptr<const Acts::TrackingGeometry> ActsGeoSvc::trackingGeometry() const { return m_trackingGeo; }

inline std::shared_ptr<const Acts::MagneticFieldProvider> ActsGeoSvc::magneticField() const { return m_magneticField; }

#endif  // K4ACTSTRACKING_ACTSGEOSVC_H
