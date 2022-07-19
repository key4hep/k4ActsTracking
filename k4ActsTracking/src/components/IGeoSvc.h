// D. Elitez, July 2022
// Based on eic/juggler

#ifndef IGEOSVC_H
#define IGEOSVC_H

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

class GAUDI_API IGeoSvc : virtual public IService {
public:
  using VolumeSurfaceMap = std::unordered_map<uint64_t, const Acts::Surface*>;

public:
  DeclareInterfaceID(IGeoSvc, 1, 0);

  virtual const Acts::TrackingGeometry& trackingGeometry() const = 0;

  virtual ~IGeoSvc() {}
};

#endif  // IGEOSVC_H
