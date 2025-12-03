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
#include "ActsGeoSvc.h"

#include "k4ActsTracking/ActsGaudiLogger.h"

#include "k4Interface/IGeoSvc.h"

#include "Acts/Geometry/TrackingGeometry.hpp"
#include "Acts/MagneticField/ConstantBField.hpp"
#include "Acts/Visualization/GeometryView3D.hpp"
#include "Acts/Visualization/ObjVisualization3D.hpp"
#if __has_include("ActsPlugins/DD4hep/ConvertDD4hepDetector.hpp")
#include "ActsPlugins/DD4hep/ConvertDD4hepDetector.hpp"
#else
#include "Acts/Plugins/DD4hep/ConvertDD4hepDetector.hpp"
namespace ActsPlugins {
  using Acts::convertDD4hepDetector;
  using Acts::sortDetElementsByID;

}  // namespace ActsPlugins
#endif

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <array>

using namespace Gaudi;

DECLARE_COMPONENT(ActsGeoSvc)

ActsGeoSvc::ActsGeoSvc(const std::string& name, ISvcLocator* svc) : base_class(name, svc) {}

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

  auto logger   = makeActsGaudiLogger(this);
  m_trackingGeo = ActsPlugins::convertDD4hepDetector(
      m_dd4hepGeo->world(), *logger, bTypePhi, bTypeR, bTypeZ, layerEnvelopeR, layerEnvelopeZ, defaultLayerThickness,
      ActsPlugins::sortDetElementsByID, m_trackingGeoCtx, m_materialDeco);

  std::array<double, 3> magneticFieldVector = {0, 0, 0};
  std::array<double, 3> position            = {0, 0, 0};
  m_dd4hepGeo->field().magneticField(position.data(), magneticFieldVector.data());
  debug() << fmt::format("Retrieved magnetic field at position {}: {}", position, magneticFieldVector) << endmsg;
  m_magneticField = std::make_shared<Acts::ConstantBField>(
      Acts::Vector3(magneticFieldVector[0] / dd4hep::tesla * Acts::UnitConstants::T,
                    magneticFieldVector[1] / dd4hep::tesla * Acts::UnitConstants::T,
                    magneticFieldVector[2] / dd4hep::tesla * Acts::UnitConstants::T));

  /// Setting geometry debug option
  if (m_debugGeometry == true) {
    info() << "Geometry debugging is ON." << endmsg;

    if (createGeoObj().isFailure()) {
      error() << "Could not create geometry OBJ" << endmsg;
      return StatusCode::FAILURE;
    } else {
      info() << "Geometry OBJ SUCCESSFULLY created" << endmsg;
    }
  } else {
    info() << "Geometry converted without checking if GeoObj can be created" << endmsg;
  }

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
  m_obj.write(m_outputFileName.value());
  info() << m_outputFileName << " SUCCESSFULLY written." << endmsg;

  return StatusCode::SUCCESS;
}
