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

#include <k4Interface/IGeoSvc.h>

#include <Gaudi/Property.h>
#include <GaudiKernel/Service.h>

#include <Parsers/Primitives.h>

#include <ActsPlugins/DD4hep/BlueprintBuilder.hpp>

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
  class DetElement;
}  // namespace dd4hep

class ActsGeoSvc : public extends<Service, IActsGeoSvc> {
public:
  std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry() const override;

  std::shared_ptr<const Acts::MagneticFieldProvider> magneticField() const override;

  const CaloFaceSurfaces& caloFaceSurfaces() const override { return m_caloFaceSurfaces; }

  const std::vector<Acts::GeometryIdentifier>& caloSurfaceGeoIds() const override { return m_caloSurfaceGeoIds; }

  ActsGeoSvc(const std::string& name, ISvcLocator* svcLoc);

  ~ActsGeoSvc() = default;

  StatusCode initialize() override;

  Gaudi::Property<std::string> m_objDumpFileName{this, "ObjVisFileName", "dump_acts_geo.obj",
                                                 "Name of the 3D visualization file"};
  Gaudi::Property<bool>        m_dumpVisualization{this, "DumpVisualization", false,
                                            "Whether or not to create a 3D visualization dump"};
  Gaudi::Property<std::string> m_encodingStringConstant{
      this, "EncodingStringVariable", "GlobalTrackerReadoutID",
      "Name of the DD4hep constant holding the CellID encoding string."};
  Gaudi::Property<bool> m_buildCaloSurfaces{
      this, "BuildCaloSurfaces", true,
      "Whether to build the ECAL inner-face surfaces (for track extrapolation to the calorimeter face)."};
  Gaudi::Property<bool> m_useDD4hepField{
      this, "UseDD4hepBField", false,
      "Use the real, position-dependent DD4hep magnetic field (via ACTS' DD4hepFieldAdapter) for all propagation, "
      "instead of a uniform Acts::ConstantBField sampled at the origin. Needed for localized fields such as the LUXE "
      "dipole; the default (false) preserves the constant-field behaviour of the collider/barrel clients."};

  const CellIDSurfaceMap& cellIdToSurfaceMap() const override { return m_cellIDToSurface; }
  std::string             cellIDEncodingString() const override { return m_cellIDEncodingString; }

private:
  using BlueprintBuilder = ActsPlugins::DD4hep::BlueprintBuilder;

  using BlueprintPopulationFunc = void(const std::string&, Acts::Experimental::Blueprint&, BlueprintBuilder&,
                                       const CaloFaceSurfaces&);

  /// Build the ECAL inner-face surfaces (m_caloFaceSurfaces) from the DD4hep
  /// geometry. Surfaces are located via DetType flags and dimensioned from the
  /// dd4hep::rec::LayeredCalorimeterData extension. Missing ECAL sub-detectors
  /// or extensions are skipped with a warning rather than treated as an error.
  /// Must run before the blueprint is constructed, since the surfaces are
  /// inserted into the tracking geometry as passive calo volumes.
  void buildCaloFaceSurfaces();

  /// Build a single rectangular calo-face surface (m_caloFaceSurfaces.planarFace)
  /// for a telescope-style detector (e.g. LUXE), where the electromagnetic
  /// calorimeter is one rectangular slab rather than a cylindrical barrel/endcap.
  /// The face geometry is taken from the DetElement's box envelope and world
  /// placement. @p lengthScale converts DD4hep native lengths to ACTS units.
  void buildPlanarCaloFace(const dd4hep::DetElement& ecal, double lengthScale);

  SmartIF<IGeoSvc>                                          m_geoSvc;
  std::shared_ptr<const Acts::TrackingGeometry>             m_trackingGeo{nullptr};
  std::shared_ptr<const Acts::MagneticFieldProvider>        m_magneticField{nullptr};
  std::unordered_map<dd4hep::CellID, const Acts::Surface*>  m_cellIDToSurface{};
  std::unordered_map<std::string, BlueprintPopulationFunc*> m_bluePrintPopulationFuncs{};
  std::string                                               m_cellIDEncodingString{};
  CaloFaceSurfaces                                          m_caloFaceSurfaces{};
  std::vector<Acts::GeometryIdentifier>                     m_caloSurfaceGeoIds{};
};

inline std::shared_ptr<const Acts::TrackingGeometry> ActsGeoSvc::trackingGeometry() const { return m_trackingGeo; }

inline std::shared_ptr<const Acts::MagneticFieldProvider> ActsGeoSvc::magneticField() const { return m_magneticField; }

#endif  // K4ACTSTRACKING_ACTSGEOSVC_H
