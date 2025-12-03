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
}

namespace dd4hep {
  class Detector;
}

class ActsGeoGen3Svc : public extends<Service, IActsGeoSvc> {
public:
  const Acts::TrackingGeometry& trackingGeometry() const override;

  ActsGeoGen3Svc(const std::string& name, ISvcLocator* svcLoc);

  ~ActsGeoGen3Svc() = default;

  StatusCode initialize() override;

  Gaudi::Property<std::string> m_detElementName{this, "DetElementName", "Name of the DetElement", "InnerTrackerBarrel"};
  Gaudi::Property<std::string> m_layerPattern{this, "LayerPatternExpr", "Layer pattern match expression", "layer"};

private:
  dd4hep::Detector*                             m_dd4hepGeo{nullptr};
  SmartIF<IGeoSvc>                              m_geoSvc;
  std::unique_ptr<const Acts::TrackingGeometry> m_trackingGeo{nullptr};
};

inline const Acts::TrackingGeometry& ActsGeoGen3Svc::trackingGeometry() const { return *m_trackingGeo; }

#endif  // K4ACTSTRACKING_ACTSGEOGEN3SVC_H
