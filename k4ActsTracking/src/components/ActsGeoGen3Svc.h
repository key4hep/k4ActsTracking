#ifndef K4ACTSTRACKING_ACTSGEOGEN3SVC_H
#define K4ACTSTRACKING_ACTSGEOGEN3SVC_H

#include "k4ActsTracking/IActsGeoSvc.h"

#include <Gaudi/Property.h>
#include <k4Interface/IGeoSvc.h>

#include "GaudiKernel/Service.h"

#include <memory>
#include <string>

namespace Acts {
  class TrackingGeometry;
  class MagneticFieldProvider;
}  // namespace Acts

namespace dd4hep {
  class Detector;
}

class ActsGeoGen3Svc : public extends<Service, IActsGeoSvc> {
public:
  std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry() const override;

  std::shared_ptr<const Acts::MagneticFieldProvider> magneticField() const override;

  ActsGeoGen3Svc(const std::string& name, ISvcLocator* svcLoc);

  ~ActsGeoGen3Svc() = default;

  StatusCode initialize() override;

  Gaudi::Property<std::string> m_detElementName{this, "DetElementName", "InnerTrackerBarrel", "Name of the DetElement"};
  Gaudi::Property<std::string> m_layerPattern{this, "LayerPatternExpr", "layer", "Layer pattern match expression"};

private:
  dd4hep::Detector*                                  m_dd4hepGeo{nullptr};
  SmartIF<IGeoSvc>                                   m_geoSvc;
  std::shared_ptr<const Acts::TrackingGeometry>      m_trackingGeo{nullptr};
  std::shared_ptr<const Acts::MagneticFieldProvider> m_magneticField{nullptr};
};

inline std::shared_ptr<const Acts::TrackingGeometry> ActsGeoGen3Svc::trackingGeometry() const { return m_trackingGeo; }

inline std::shared_ptr<const Acts::MagneticFieldProvider> ActsGeoGen3Svc::magneticField() const {
  return m_magneticField;
}

#endif  // K4ACTSTRACKING_ACTSGEOGEN3SVC_H
