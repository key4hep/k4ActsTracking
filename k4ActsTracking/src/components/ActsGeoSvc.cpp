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
#include <Acts/Geometry/BlueprintNode.hpp>
#include <Acts/Geometry/BlueprintOptions.hpp>
#include <Acts/Geometry/ContainerBlueprintNode.hpp>
#include <Acts/Geometry/CylinderVolumeBounds.hpp>
#include <Acts/Geometry/Extent.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Geometry/GeometryIdentifier.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/Geometry/VolumeAttachmentStrategy.hpp>
#include <Acts/MagneticField/ConstantBField.hpp>
#include <Acts/Surfaces/DiscSurface.hpp>
#include <Acts/Surfaces/PlaneSurface.hpp>
#include <Acts/Surfaces/RectangleBounds.hpp>
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/Utilities/AxisDefinitions.hpp>
#include <Acts/Visualization/ObjVisualization3D.hpp>
#include <ActsPlugins/DD4hep/BlueprintBuilder.hpp>
#include <ActsPlugins/DD4hep/DD4hepDetectorElement.hpp>

#include <DD4hep/DD4hepUnits.h>
#include <DD4hep/DetElement.h>
#include <DD4hep/DetType.h>
#include <DD4hep/Detector.h>
#include <DDRec/DetectorData.h>

#include <GaudiKernel/StatusCode.h>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <array>
#include <cmath>
#include <numbers>

template <> struct fmt::formatter<Acts::GeometryIdentifier> : fmt::ostream_formatter {};

DECLARE_COMPONENT(ActsGeoSvc)

ActsGeoSvc::ActsGeoSvc(const std::string& name, ISvcLocator* svcLoc) : base_class(name, svcLoc) {
  m_bluePrintPopulationFuncs = {{"MAIA_v0", MuColl::MAIA_v0::populateBlueprint},
                                {"MuSIC_v2", MuColl::MAIA_v0::populateBlueprint},
                                {"ILD_FCCee_v01", FCCee::ILD_FCCee_v01::populateBlueprint},
                                {"ILD_FCCee_v02", FCCee::ILD_FCCee_v02::populateBlueprint},
                                {"CLD_o2_v07", FCCee::CLD_o2_v07::populateBlueprint},
                                {"CLD_o2_v08", FCCee::CLD_o2_v07::populateBlueprint},
                                {"LUXE_v0", LUXE::LUXE_v0::populateBlueprint}};
}

StatusCode ActsGeoSvc::initialize() {
  m_geoSvc = Gaudi::svcLocator()->service<IGeoSvc>("GeoSvc");
  K4_GAUDI_CHECK(m_geoSvc);

  m_cellIDEncodingString = m_geoSvc->getDetector()->constantAsString(m_encodingStringConstant.value());
  debug() << "CellID encoding string: " << m_cellIDEncodingString << endmsg;

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
  if (nSurfaces != m_cellIDToSurface.size()) {
    error() << fmt::format("{} Surfaces in the Tracking geometry but only {} distinct CellIDs found.", nSurfaces,
                           m_cellIDToSurface.size());
    return StatusCode::FAILURE;
  }

  if (m_dumpVisualization.value()) {
    info() << "Creating visualization" << endmsg;
    // Adjust the scale here to make it easier to import in blender
    Acts::ObjVisualization3D vis{4, 0.001};
    m_trackingGeo->visualize(vis, gctxt);
    vis.write(m_objDumpFileName.value());
  }

  if (m_buildCaloSurfaces.value()) {
    buildCaloFaceSurfaces();
  }

  return StatusCode::SUCCESS;
}

void ActsGeoSvc::buildCaloFaceSurfaces() {
  using dd4hep::DetType;
  using dd4hep::rec::LayeredCalorimeterData;
  namespace UC = Acts::UnitConstants;

  // LayeredCalorimeterData::extent[] is stored in DD4hep native length units
  // (cm), so convert to ACTS units with the same scale the blueprint uses.
  const double lengthScale = Acts::UnitConstants::cm / dd4hep::cm;

  const auto* dd4hepDet = m_geoSvc->getDetector();

  // Locate the electromagnetic calorimeter sub-detectors purely via DetType
  // flags, so that this works across detector concepts without hard-coding
  // DetElement names. The dimensions themselves come from the standard DDRec
  // LayeredCalorimeterData extension (the same source Pandora uses).
  const auto ecalBarrel =
      dd4hepDet->detectors(DetType::CALORIMETER | DetType::ELECTROMAGNETIC | DetType::BARREL, DetType::FORWARD);
  const auto ecalEndcap =
      dd4hepDet->detectors(DetType::CALORIMETER | DetType::ELECTROMAGNETIC | DetType::ENDCAP, 0);

  // Barrel circumradius (corner radius), needed both for the barrel faces and to
  // guarantee the endcap discs reach far enough to avoid hermeticity gaps at the
  // barrel/endcap junction.
  double barrelCircumradius = 0.0;
  double barrelHalfZ        = 0.0;

  // --- Barrel: one planar surface per polygon side --------------------------
  if (ecalBarrel.empty()) {
    warning() << "No electromagnetic barrel calorimeter found via DetType flags; "
                 "no barrel calo-face surfaces will be built."
              << endmsg;
  } else {
    const auto* caloData = ecalBarrel.front().extension<LayeredCalorimeterData>(false);
    if (caloData == nullptr) {
      warning() << "ECAL barrel DetElement has no LayeredCalorimeterData extension; "
                   "skipping barrel calo-face surfaces."
                << endmsg;
    } else {
      const int nSides = caloData->inner_symmetry > 0 ? caloData->inner_symmetry : 0;
      // For a BarrelLayout the calorimeter is centred on z = 0 and extent[] is
      // {rmin, rmax, zmin=0, zmax=half_length}, so the half-length is extent[3].
      const double apothem = caloData->extent[0] * lengthScale;  // perpendicular distance to inner face
      barrelHalfZ          = caloData->extent[3] * lengthScale;  // barrel half-length
      const double phi0    = caloData->inner_phi0;               // azimuth of first inner-face normal

      if (nSides < 3) {
        warning() << fmt::format("ECAL barrel has unusable inner_symmetry={}; skipping barrel faces.",
                                 caloData->inner_symmetry)
                  << endmsg;
      } else {
        const double dPhi      = 2 * std::numbers::pi / nSides;
        barrelCircumradius     = apothem / std::cos(std::numbers::pi / nSides);
        const double halfWidth = apothem * std::tan(std::numbers::pi / nSides);

        m_caloFaceSurfaces.barrelFaces.reserve(nSides);
        for (int i = 0; i < nSides; ++i) {
          const double  phi    = phi0 + i * dPhi;
          const double  cphi   = std::cos(phi);
          const double  sphi   = std::sin(phi);
          Acts::Vector3 normal{cphi, sphi, 0};               // local z (surface normal, radial)
          Acts::Vector3 localX{-sphi, cphi, 0};              // tangential
          Acts::Vector3 localY{0, 0, 1};                     // along global z
          Acts::Vector3 center = apothem * normal;           // barrel centred on z = 0

          Acts::Transform3 transform = Acts::Transform3::Identity();
          transform.linear().col(0) = localX;
          transform.linear().col(1) = localY;
          transform.linear().col(2) = normal;
          transform.translation()   = center;

          auto bounds = std::make_shared<Acts::RectangleBounds>(halfWidth, barrelHalfZ);
          m_caloFaceSurfaces.barrelFaces.push_back(Acts::Surface::makeShared<Acts::PlaneSurface>(transform, bounds));
        }
        info() << fmt::format(
                      "Built ECAL barrel calo face: {} planar faces, apothem={:.1f} mm, circumradius={:.1f} mm, "
                      "halfZ={:.1f} mm, phi0={:.4f}",
                      nSides, apothem / UC::mm, barrelCircumradius / UC::mm, barrelHalfZ / UC::mm, phi0)
               << endmsg;
      }
    }
  }

  // --- Endcaps: flat discs, widened to stay hermetic with the barrel --------
  if (ecalEndcap.empty()) {
    warning() << "No electromagnetic endcap calorimeter found via DetType flags; "
                 "no endcap calo-face surfaces will be built."
              << endmsg;
  } else {
    const auto* caloData = ecalEndcap.front().extension<LayeredCalorimeterData>(false);
    if (caloData == nullptr) {
      warning() << "ECAL endcap DetElement has no LayeredCalorimeterData extension; "
                   "skipping endcap calo-face surfaces."
                << endmsg;
    } else {
      const double rMin    = caloData->extent[0] * lengthScale;
      double       rMax    = caloData->extent[1] * lengthScale;
      const double zEndcap = caloData->extent[2] * lengthScale;  // inner-face z (zmin)

      // Guarantee the disc reaches at least the barrel corner radius so there is
      // no gap at the barrel/endcap junction.
      if (barrelCircumradius > rMax) {
        warning() << fmt::format(
                         "ECAL endcap rMax ({:.1f} mm) is smaller than the barrel circumradius ({:.1f} mm); "
                         "extending the endcap disc to close the hermeticity gap.",
                         rMax / UC::mm, barrelCircumradius / UC::mm)
                  << endmsg;
        rMax = barrelCircumradius;
      }

      Acts::Transform3 tPos = Acts::Transform3::Identity();
      tPos.translation()    = Acts::Vector3{0, 0, zEndcap};
      Acts::Transform3 tNeg = Acts::Transform3::Identity();
      tNeg.translation()    = Acts::Vector3{0, 0, -zEndcap};

      m_caloFaceSurfaces.endcapPos = Acts::Surface::makeShared<Acts::DiscSurface>(tPos, rMin, rMax);
      m_caloFaceSurfaces.endcapNeg = Acts::Surface::makeShared<Acts::DiscSurface>(tNeg, rMin, rMax);

      info() << fmt::format("Built ECAL endcap calo faces: discs at z=+/-{:.1f} mm, rMin={:.1f} mm, rMax={:.1f} mm",
                            zEndcap / UC::mm, rMin / UC::mm, rMax / UC::mm)
             << endmsg;
    }
  }

  info() << fmt::format("Calo-face surfaces built: {} barrel faces, {} endcap discs",
                        m_caloFaceSurfaces.barrelFaces.size(),
                        (m_caloFaceSurfaces.endcapPos ? 1 : 0) + (m_caloFaceSurfaces.endcapNeg ? 1 : 0))
         << endmsg;
}
