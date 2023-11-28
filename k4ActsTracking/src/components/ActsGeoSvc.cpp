/*
 * Copyright (c) 2014-2023 Key4hep-Project.
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

#include "ActsGeoSvc.h"
#include "Acts/Geometry/TrackingGeometry.hpp"
#include "Acts/MagneticField/MagneticFieldContext.hpp"
#include "Acts/Plugins/DD4hep/ConvertDD4hepDetector.hpp"
#include "Acts/Surfaces/PlaneSurface.hpp"
#include "Acts/Utilities/Logger.hpp"
#include "Acts/Visualization/GeometryView3D.hpp"
#include "Acts/Visualization/ObjVisualization3D.hpp"
#include "DD4hep/Printout.h"
#include "GaudiKernel/Service.h"
#include "TGeoManager.h"
#include "k4Interface/IGeoSvc.h"

using namespace Gaudi;

DECLARE_COMPONENT(ActsGeoSvc)

ActsGeoSvc::ActsGeoSvc(const std::string& name, ISvcLocator* svc) : base_class(name, svc), m_log(msgSvc(), name) {}

ActsGeoSvc::~ActsGeoSvc(){};

StatusCode ActsGeoSvc::initialize() {
  m_dd4hepGeo = svcLocator()->service<IGeoSvc>(m_geoSvcName)->getDetector();
  // necessary?
  // m_dd4hepGeo->addExtension<IActsGeoSvc>(this);

  Acts::BinningType bTypePhi              = Acts::equidistant;
  Acts::BinningType bTypeR                = Acts::equidistant;
  Acts::BinningType bTypeZ                = Acts::equidistant;
  double            layerEnvelopeR        = Acts::UnitConstants::mm;
  double            layerEnvelopeZ        = Acts::UnitConstants::mm;
  double            defaultLayerThickness = Acts::UnitConstants::fm;
  using Acts::sortDetElementsByID;
  auto logger   = Acts::getDefaultLogger("k4ActsTracking", m_actsLoggingLevel);
  m_trackingGeo = Acts::convertDD4hepDetector(m_dd4hepGeo->world(), *logger, bTypePhi, bTypeR, bTypeZ, layerEnvelopeR,
                                              layerEnvelopeZ, defaultLayerThickness, sortDetElementsByID,
                                              m_trackingGeoCtx, m_materialDeco);

  /// Setting geometry debug option
  if (m_debugGeometry == true) {
    m_log << MSG::INFO << "Geometry debugging is ON." << endmsg;

    if (createGeoObj().isFailure()) {
      m_log << MSG::ERROR << "Could not create geometry OBJ" << endmsg;
      return StatusCode::FAILURE;
    } else {
      m_log << MSG::INFO << "Geometry OBJ SUCCESSFULLY created" << endmsg;
    }
  } else {
    m_log << MSG::VERBOSE << "Geometry debugging is OFF." << endmsg;
    return StatusCode::SUCCESS;
  }
  std::cout << "works!" << std::endl;

  return StatusCode::SUCCESS;
}

StatusCode ActsGeoSvc::execute() { return StatusCode::SUCCESS; }

StatusCode ActsGeoSvc::finalize() { return StatusCode::SUCCESS; }

/// Create a geometry OBJ file
StatusCode ActsGeoSvc::createGeoObj() {
  // Convert DD4Hep geometry to acts

  Acts::ObjVisualization3D m_obj;

  if (!m_trackingGeo) {
    return StatusCode::FAILURE;
  }
  m_trackingGeo->visitSurfaces([&](const Acts::Surface* surface) {
    if (surface == nullptr) {
      info() << "no surface??? " << endmsg;
      return;
    }
    Acts::GeometryView3D::drawSurface(m_obj, *surface, m_trackingGeoCtx);
  });
  m_obj.write(m_outputFileName);
  m_log << MSG::INFO << m_outputFileName << " SUCCESSFULLY written." << endmsg;

  return StatusCode::SUCCESS;
}
