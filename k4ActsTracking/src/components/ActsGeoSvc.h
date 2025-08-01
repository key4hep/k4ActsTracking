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

#ifndef ACTSGEOSVC_H
#define ACTSGEOSVC_H

#include <Acts/Material/IMaterialDecorator.hpp>
#include "Acts/Definitions/Common.hpp"
#include "Acts/Definitions/Units.hpp"
#include "Acts/Geometry/GeometryContext.hpp"
#include "Acts/Geometry/TrackingGeometry.hpp"
#include "Acts/Plugins/DD4hep/DD4hepDetectorElement.hpp"
#include "Acts/Surfaces/Surface.hpp"
#include "Acts/Utilities/Logger.hpp"
#include "DD4hep/DD4hepUnits.h"
#include "DD4hep/Detector.h"
#include "DDRec/Surface.h"
#include "DDRec/SurfaceManager.h"
#include "GaudiKernel/MsgStream.h"
#include "GaudiKernel/Service.h"
#include "GaudiKernel/ServiceHandle.h"
#include "IActsGeoSvc.h"

class ActsGeoSvc : public extends<Service, IActsGeoSvc> {
public:
  using VolumeSurfaceMap = std::unordered_map<uint64_t, const Acts::Surface*>;

private:
  dd4hep::Detector* m_dd4hepGeo = nullptr;

  /// DD4hep surface map
  std::map<int64_t, dd4hep::rec::Surface*> m_surfaceMap;

  /// ACTS Logging Level
  Acts::Logging::Level m_actsLoggingLevel = Acts::Logging::INFO;

  /// ACTS Tracking Geometry Context
  Acts::GeometryContext m_trackingGeoCtx;

  /// ACTS Tracking Geometry
  std::unique_ptr<const Acts::TrackingGeometry> m_trackingGeo{nullptr};

  /// ACTS Material Decorator
  std::shared_ptr<const Acts::IMaterialDecorator> m_materialDeco{nullptr};

  /// ACTS surface lookup container for hit surfaces that generate smeared hits
  VolumeSurfaceMap m_surfaces;

  Gaudi::Property<std::string> m_geoSvcName{this, "GeoSvcName", "GeoSvc", "The name of the GeoSvc instance"};

  /// Option for the Debug Geometry
  Gaudi::Property<bool> m_debugGeometry{this, "debugGeometry", false, "Option for geometry debugging"};
  /// Output file name
  Gaudi::Property<std::string> m_outputFileName{this, "outputFileName", "", "Output file name"};

  /// Gaudi logging output
  MsgStream m_log;

public:
  ActsGeoSvc(const std::string& name, ISvcLocator* svc);

  virtual ~ActsGeoSvc();

  virtual StatusCode initialize() final;

  virtual StatusCode execute() final;

  virtual StatusCode finalize() final;

  StatusCode createGeoObj();

  virtual const Acts::TrackingGeometry& trackingGeometry() const;
};

inline const Acts::TrackingGeometry& ActsGeoSvc::trackingGeometry() const { return *m_trackingGeo; }
#endif
