#ifndef K4ACTSTRACKING_ACTSGEOSVC_H
#define K4ACTSTRACKING_ACTSGEOSVC_H

#include "k4ActsTracking/IActsGeoSvc.h"

#include <Gaudi/Property.h>
#include <k4Interface/IGeoSvc.h>

#include <Parsers/Primitives.h>

#include "GaudiKernel/Service.h"

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

namespace ActsPlugins::DD4hep {
  class BlueprintBuilder;
}

namespace dd4hep {
  class Detector;
}

class ActsGeoSvc : public extends<Service, IActsGeoSvc> {
public:
  std::shared_ptr<const Acts::TrackingGeometry> trackingGeometry() const override;

  std::shared_ptr<const Acts::MagneticFieldProvider> magneticField() const override;

  ActsGeoSvc(const std::string& name, ISvcLocator* svcLoc);

  ~ActsGeoSvc() = default;

  StatusCode initialize() override;

  Gaudi::Property<std::string> m_objDumpFileName{this, "ObjVisFileName", "dump_acts_geo.obj",
                                                 "Name of the 3D visualization file"};
  Gaudi::Property<bool>        m_dumpVisualization{this, "DumpVisualization", false,
                                            "Whether or not to create a 3D visualization dump"};

  const CellIDSurfaceMap& cellIdToSurfaceMap() const override { return m_cellIDToSurface; }

private:
  using BlueprintPopulationFunc = void(const std::string&, Acts::Experimental::Blueprint&,
                                       ActsPlugins::DD4hep::BlueprintBuilder&);

  dd4hep::Detector*                                         m_dd4hepGeo{nullptr};
  SmartIF<IGeoSvc>                                          m_geoSvc;
  std::shared_ptr<const Acts::TrackingGeometry>             m_trackingGeo{nullptr};
  std::shared_ptr<const Acts::MagneticFieldProvider>        m_magneticField{nullptr};
  std::unordered_map<dd4hep::CellID, const Acts::Surface*>  m_cellIDToSurface{};
  std::unordered_map<std::string, BlueprintPopulationFunc*> m_bluePrintPopulationFuncs{};
};

inline std::shared_ptr<const Acts::TrackingGeometry> ActsGeoSvc::trackingGeometry() const { return m_trackingGeo; }

inline std::shared_ptr<const Acts::MagneticFieldProvider> ActsGeoSvc::magneticField() const { return m_magneticField; }

#endif  // K4ACTSTRACKING_ACTSGEOSVC_H
