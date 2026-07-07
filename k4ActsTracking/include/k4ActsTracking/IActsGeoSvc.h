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

#ifndef IACTSGEOSVC_H
#define IACTSGEOSVC_H

#include <GaudiKernel/IService.h>

#include <DD4hep/Primitives.h>

#include <Acts/Geometry/GeometryIdentifier.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace dd4hep {
  namespace rec {
    class Surface;
  }
}  // namespace dd4hep

namespace Acts {
  class TrackingGeometry;
  class Surface;
  class MagneticFieldProvider;
}  // namespace Acts

class GAUDI_API IActsGeoSvc : virtual public IService {
public:
  using CellIDSurfaceMap = std::unordered_map<dd4hep::CellID, const Acts::Surface*>;

  /// Surfaces approximating the inner face of the electromagnetic calorimeter,
  /// derived from the DD4hep geometry. They are inserted into the tracking
  /// geometry as passive surfaces of dedicated calo volumes, so that the
  /// standard geometry-aware propagation can navigate to them (see
  /// ACTSTracking::extrapolateToCaloFace).
  ///
  /// The barrel is a regular polygon, so its inner face is modelled as one
  /// planar surface per polygon side rather than a cylinder. The endcaps are
  /// flat discs at constant z. The struct also carries the bounding cylinder
  /// dimensions used to build the enclosing TrackingVolumes (ACTS units).
  struct CaloFaceSurfaces {
    std::vector<std::shared_ptr<Acts::Surface>> barrelFaces;  ///< one plane per polygon side
    std::shared_ptr<Acts::Surface>              endcapPos;    ///< disc at +z, may be null
    std::shared_ptr<Acts::Surface>              endcapNeg;    ///< disc at -z, may be null

    // Bounding-cylinder dimensions for the enclosing TrackingVolumes.
    double barrelRMin  = 0;  ///< inner radius of the barrel calo volume (~apothem)
    double barrelRMax  = 0;  ///< outer radius of the barrel calo volume (~circumradius)
    double barrelHalfZ = 0;  ///< barrel half-length along z
    double endcapRMin  = 0;  ///< inner radius of the endcap disc volumes
    double endcapRMax  = 0;  ///< outer radius of the endcap disc volumes
    double endcapZ     = 0;  ///< |z| of the endcap inner face

    bool empty() const { return barrelFaces.empty() && !endcapPos && !endcapNeg; }
  };

public:
  DeclareInterfaceID(IActsGeoSvc, 1, 0);

  virtual std::shared_ptr<const Acts::TrackingGeometry>      trackingGeometry() const     = 0;
  virtual std::shared_ptr<const Acts::MagneticFieldProvider> magneticField() const        = 0;
  virtual const CellIDSurfaceMap&                            cellIdToSurfaceMap() const   = 0;
  virtual std::string                                        cellIDEncodingString() const = 0;
  virtual const CaloFaceSurfaces&                            caloFaceSurfaces() const     = 0;

  /// Geometry identifiers of the calorimeter-face surfaces, valid after the
  /// tracking geometry has been constructed. Used by the extrapolation to
  /// recognise when the propagation has reached the calo face.
  virtual const std::vector<Acts::GeometryIdentifier>& caloSurfaceGeoIds() const = 0;

  virtual ~IActsGeoSvc() = default;
};

#endif  // IACTSGEOSVC_H
