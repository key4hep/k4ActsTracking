// CellID (DD4hep VolumeID) -> Acts::Surface mapping

#pragma once

#include "k4ActsTracking/ITrackerMappingSvc.h"
#include "k4ActsTracking/IActsGeoSvc.h"

#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <memory>
#include <string>

#include <Gaudi/Property.h>
#include <GaudiKernel/Service.h>
#include <GaudiKernel/StatusCode.h>

#include "DDSegmentation/BitFieldCoder.h"

namespace Acts {
class Surface;
}

class TrackerMappingSvc final : public extends<Service, ITrackerMappingSvc> {
public:
  TrackerMappingSvc(const std::string& name, ISvcLocator* svcLoc);

  StatusCode initialize() override;
  StatusCode finalize() override;

  const Acts::Surface* surface(std::uint64_t cellID) const override;
  bool hasSurface(std::uint64_t cellID) const override;
  std::size_t size() const override;

private:
  StatusCode buildMapping();

  static std::uint64_t cellIDFromSurface(const Acts::Surface& surface);
  // normalize hit cellID (mask x/y) -> sensor-level key
  std::uint64_t normalizeCellID(std::uint64_t cellID) const;

private:
  Gaudi::Property<std::string> m_actsGeoSvcName{this, "ActsGeoSvc", "ActsGeoPlaneSvc",
                                                "Name of the IActsGeoSvc provider."};

  // bitfield layout string (same as LUXE XML constant GlobalTrackerReadoutID)
  Gaudi::Property<std::string> m_idLayout{
      this,
      "IDLayout",
      "system:1,side:1,layer:2,module:1,sensor:5,x:32:-16,y:-16",
      "DD4hep BitFieldCoder layout string for masking x/y"};
  // whether to mask x/y before lookup
  Gaudi::Property<bool> m_maskXY{
      this, "MaskXY", true, "Mask x/y fields before CellID->Surface lookup"};  
  
  SmartIF<IActsGeoSvc> m_actsGeoSvc{};

  std::unordered_map<std::uint64_t, const Acts::Surface*> m_cellIDToSurface{};

  mutable std::mutex m_mutex{};

  std::size_t m_nSurfacesVisited{0};
  std::size_t m_nWithAssociatedDetElem{0};
  std::size_t m_nDD4hepDetElem{0};
  std::size_t m_nInserted{0};

  // decoder instance + field availability flags
  std::unique_ptr<dd4hep::DDSegmentation::BitFieldCoder> m_decoder{};
  bool m_hasX{false};
  bool m_hasY{false};
};
