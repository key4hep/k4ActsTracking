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
#include "DD4hepBlueprintConstruction.h"

#include "k4ActsTracking/ActsGaudiLogger.h"

#include <k4FWCore/GaudiChecks.h>

#include <Acts/Definitions/Algebra.hpp>
#include <Acts/Definitions/Units.hpp>
#include <Acts/Geometry/Blueprint.hpp>
#include <Acts/Geometry/BlueprintOptions.hpp>
#include <Acts/Geometry/CylinderVolumeBounds.hpp>
#include <Acts/Geometry/Extent.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Geometry/GeometryIdentifier.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/MagneticField/ConstantBField.hpp>
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/Utilities/AxisDefinitions.hpp>
#include <Acts/Visualization/ObjVisualization3D.hpp>
#include <ActsPlugins/DD4hep/BlueprintBuilder.hpp>
#include <ActsPlugins/DD4hep/DD4hepDetectorElement.hpp>

#include <DD4hep/DD4hepUnits.h>
#include <DD4hep/DetElement.h>
#include <DD4hep/Detector.h>

#include <GaudiKernel/StatusCode.h>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <array>

template <> struct fmt::formatter<Acts::GeometryIdentifier> : fmt::ostream_formatter {};

DECLARE_COMPONENT(ActsGeoSvc)

ActsGeoSvc::ActsGeoSvc(const std::string& name, ISvcLocator* svcLoc) : base_class(name, svcLoc) {
  m_bluePrintPopulationFuncs = {{"MAIA_v0", MuColl::MAIA_v0::populateBlueprint},
                                {"ILD_FCCee_v01", FCCee::ILD_FCCee::populateBlueprint}};
}

StatusCode ActsGeoSvc::initialize() {
  m_geoSvc = Gaudi::svcLocator()->service<IGeoSvc>("GeoSvc");
  K4_GAUDI_CHECK(m_geoSvc);

  std::array<double, 3> magneticFieldVector = {0, 0, 0};
  std::array<double, 3> position            = {0, 0, 0};
  m_geoSvc->getDetector()->field().magneticField(position.data(), magneticFieldVector.data());
  debug() << fmt::format("Retrieved magnetic field at position {}: {}", position, magneticFieldVector) << endmsg;
  m_magneticField = std::make_shared<Acts::ConstantBField>(
      Acts::Vector3(magneticFieldVector[0] / dd4hep::tesla * Acts::UnitConstants::T,
                    magneticFieldVector[1] / dd4hep::tesla * Acts::UnitConstants::T,
                    magneticFieldVector[2] / dd4hep::tesla * Acts::UnitConstants::T));

  auto gaudiLogger = makeActsGaudiLogger(this);

  info() << fmt::format("Acts::cm: {}, dd4hep::cm: {}", Acts::UnitConstants::cm, dd4hep::cm) << endmsg;

  auto gctxt = Acts::GeometryContext::dangerouslyDefaultConstruct();

  const auto* dd4hepDet = m_geoSvc->getDetector();
  const auto  detName   = dd4hepDet->header().name();
  info() << fmt::format("Constructing detector with name: {}", dd4hepDet->header().name()) << endmsg;

  BlueprintBuilder builder{
      {.dd4hepDetector = dd4hepDet, .lengthScale = Acts::UnitConstants::cm / dd4hep::cm, .gctx = gctxt},
      gaudiLogger->cloneWithSuffix("|BlpBld")};

  using Acts::Experimental::Blueprint;
  using Acts::Experimental::BlueprintOptions;
  using namespace Acts::UnitLiterals;
  using enum Acts::AxisDirection;

  Blueprint::Config cfg;
  // Padding around subvolumes of the world volume
  cfg.envelope[AxisZ] = {20_mm, 20_mm};
  cfg.envelope[AxisR] = {0_mm, 20_mm};
  Blueprint root{cfg};

  debug() << fmt::format("Getting Blueprint construction function for detector: {}", detName) << endmsg;
  if (const auto it = m_bluePrintPopulationFuncs.find(detName); it != m_bluePrintPopulationFuncs.end()) {
    auto bluePrintFunc = it->second;
    bluePrintFunc(detName, root, builder);
  } else {
    error() << fmt::format("Cannot find a Blueprint construction function for detector: {}", detName) << endmsg;
    return StatusCode::FAILURE;
  }

  BlueprintOptions options;

  debug() << "Constructing tracking geometry" << endmsg;
  m_trackingGeo = root.construct(options, gctxt, *gaudiLogger->cloneWithSuffix("|Construct"));

  std::size_t nSurfaces = 0;
  m_trackingGeo->visitSurfaces([&](const Acts::Surface* surface) {
    nSurfaces++;
    const auto& actsDetElem = dynamic_cast<const ActsPlugins::DD4hepDetectorElement&>(*surface->surfacePlacement());
    const auto& detElem     = actsDetElem.sourceElement();
    verbose() << fmt::format("Adding Acts surface {} pointing to dd4hep DetElement {}", surface->geometryId(),
                             detElem.volumeID())
              << endmsg;
    const auto& [existing, inserted] = m_cellIDToSurface.emplace(detElem.volumeID(), surface);
    if (!inserted) {
      error() << fmt::format(
                     "The Acts surface {} pointing to dd4hep DetElement with cellID {} is already registered in the "
                     "map for Acts surface {}",
                     surface->geometryId(), detElem.volumeID(), existing->second->geometryId())
              << endmsg;
    }
  });

  info() << fmt::format("Visited {} Surfaces and inserted {} pairs of CellID -> Acts::Surface* into the map.",
                        nSurfaces, m_cellIDToSurface.size())
         << endmsg;
  if (m_dumpVisualization.value()) {
    info() << "Creating visualiztion" << endmsg;
    // Adjust the scale here to make it easier to import in blender
    Acts::ObjVisualization3D vis{4, 0.001};
    m_trackingGeo->visualize(vis, gctxt);
    vis.write(m_objDumpFileName.value());
  }

  return StatusCode::SUCCESS;
}
