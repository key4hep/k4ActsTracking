// D. Elitez, July 2022
// Based on eic/juggler

#ifndef IACTSGEOSVC_H
#define IACTSGEOSVC_H

#include <GaudiKernel/IService.h>
#include <unordered_map>

namespace dd4hep {
  namespace rec {
    class Surface;
  }
}  // namespace dd4hep

namespace Acts {
  class TrackingGeometry;
  class Surface;
}  // namespace Acts

class GAUDI_API IActsGeoSvc : virtual public IService {
public:
  using VolumeSurfaceMap = std::unordered_map<uint64_t, const Acts::Surface*>;

public:
  DeclareInterfaceID(IActsGeoSvc, 1, 0);

  virtual const Acts::TrackingGeometry& trackingGeometry() const = 0;

  virtual ~IActsGeoSvc() {}
};

#endif  // IACTSGEOSVC_H
