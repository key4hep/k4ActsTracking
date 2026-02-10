// a tiny alg for cellID checking
//

#pragma once

#include "k4ActsTracking/ITrackerMappingSvc.h"

#include <Gaudi/Property.h>
#include <GaudiKernel/Algorithm.h>
#include <GaudiKernel/ServiceHandle.h>

#include <k4FWCore/DataHandle.h>

#include <edm4hep/SimTrackerHitCollection.h>

#include <string>

class DumpSimTrackerHitCellIDAlg final : public Algorithm {
public:
  DumpSimTrackerHitCellIDAlg(const std::string& name, ISvcLocator* svcLoc);

  StatusCode initialize() override;
  StatusCode execute() override;

private:
  /// Input collection name (edm4hep::SimTrackerHitCollection)
  Gaudi::Property<std::string> m_inputColName{this, "InputCollection", "SiHits",
                                              "Input edm4hep::SimTrackerHit collection name"};

  /// Tracker mapping service name
  Gaudi::Property<std::string> m_mappingSvcName{this, "MappingSvc", "TrackerMappingSvc",
                                                "Name of ITrackerMappingSvc implementation"};

  /// Max number of hits to print per event
  Gaudi::Property<int> m_maxHits{this, "MaxHits", 50, "Maximum hits printed per event"};

  k4FWCore::DataHandle<edm4hep::SimTrackerHitCollection> m_inHits{"SiHits", Gaudi::DataHandle::Reader, this};

  ServiceHandle<ITrackerMappingSvc> m_mappingSvc{this, "TrackerMappingSvc", "TrackerMappingSvc"};
};
