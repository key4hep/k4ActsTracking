// Service interface: CellID (DD4hep VolumeID) -> Acts::Surface mapping

#pragma once

#include <cstdint>

#include <GaudiKernel/IInterface.h>

namespace Acts {
class Surface;
}

class GAUDI_API ITrackerMappingSvc : virtual public IInterface {
public:
  DeclareInterfaceID(ITrackerMappingSvc, 1, 0);

  virtual const Acts::Surface* surface(std::uint64_t cellID) const = 0;
  virtual bool hasSurface(std::uint64_t cellID) const = 0;
  virtual std::size_t size() const = 0;

protected:
  ~ITrackerMappingSvc() override = default;
};
