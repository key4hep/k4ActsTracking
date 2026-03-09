#pragma once

#include <Gaudi/Algorithm.h>
#include <GaudiKernel/ServiceHandle.h>
#include <k4FWCore/DataHandle.h>

#include <edm4hep/SimTrackerHitCollection.h>

#include "k4ActsTracking/ActsGaudiLogger.h"
#include "k4ActsTracking/IActsGeoSvc.h"
#include "k4ActsTracking/ITrackerMappingSvc.h"

#include <Acts/Utilities/Logger.hpp>
#include <string>

class PropagateToHitSurfaceAlg : public Gaudi::Algorithm {
public:
  PropagateToHitSurfaceAlg(const std::string& name, ISvcLocator* svcLoc);

  StatusCode initialize() override;
  StatusCode execute(const EventContext& ctx) const override;

private:
  Gaudi::Property<std::string> m_inputColName{this, "InputCollection", "SiHits"};
  Gaudi::Property<int>    m_maxHits{this, "MaxHits", 10};
  Gaudi::Property<double> m_backstepMm{this, "BackstepMm", 1.0}; // in mm
  Gaudi::Property<std::string> m_csvFile{this, "CsvFile", "propagate_hits.csv"};
  Gaudi::Property<std::string> m_objFile{this, "ObjFile", "propagate_segments.obj"};
  // if no charge in hit
  Gaudi::Property<double> m_assumeCharge{this, "AssumeCharge", 0.0};

  mutable k4FWCore::DataHandle<edm4hep::SimTrackerHitCollection> m_inHits{
    "", Gaudi::DataHandle::Reader, this};

  ServiceHandle<ITrackerMappingSvc> m_mappingSvc{this, "MappingSvc", "TrackerMappingSvc"};
  ServiceHandle<IActsGeoSvc>        m_actsGeoSvc{this, "ActsGeoSvc", "ActsGeoPlaneSvc"};

  std::unique_ptr<const Acts::Logger> m_actsLogger{nullptr};
};

