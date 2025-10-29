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
#pragma once

// edm4hep
#include <edm4hep/TrackCollection.h>

// Gaudi
#include <Gaudi/Property.h>

// k4FWCore
#include <k4FWCore/DataHandle.h>
#include <k4FWCore/Transformer.h>

// Standard
#include <memory>

namespace TrackPerf {}

/**
 * @brief A Transformer that Filters Out Reconstructed Tracks Given a Set of Requirements
 *
 * @author Samuel Ferraro
 * @author Unknown
 */
struct FilterTracksAlg final : k4FWCore::Transformer<edm4hep::TrackCollection(const edm4hep::TrackCollection&)> {
public:
  /**
         	* @brief Constructor for FilterTracksAlg
         	* @param name unique string identifier for this instance
         	* @param svcLoc a Service Locator passed by the Gaudi AlgManager
         	*/
  FilterTracksAlg(const std::string& name, ISvcLocator* pSvcLocator);

  /**
		 * @brief Sets up the Magnetic Field of the detector
		 */
  StatusCode initialize();

  /**
         	* @brief FilterTracksAlg operation. The workhorse of this Transformer.
         	* @param trackCollection A collection of deduped tracks.
         	* @return A Track Collection with filters applied
         	*/
  edm4hep::TrackCollection operator()(const edm4hep::TrackCollection& tracks) const;

private:
  /**
		 * @brief Sets up the Magnetic Field of the Detector from dd4hep
		 * @TODO: This can be done better with a GeoSvc.
		 */
  void buildBfield();

  //! Cut off for total number of hits
  Gaudi::Property<int> m_NHitsTotal{this, "NHitsTotal", 7, "Minimum number of hits on track"};
  //! Cut off for number of hits in vertex detector (barrel and endcap combined)
  Gaudi::Property<int> m_NHitsVertex{this, "NHitsVertex", 3, "Minimum number of hits on vertex detector"};
  //! Cut off for number of hits in inner tracker (barrel and endcap combined)
  Gaudi::Property<int> m_NHitsInner{this, "NHitsInner", 2, "Minimum number of hits on inner tracker"};
  //! Cut off for number of hits in outer tracker (barrel and endcap combined)
  Gaudi::Property<int> m_NHitsOuter{this, "NHitsOuter", 1, "Minimum number of hits on outer tracker"};
  //! Cut off for D0
  Gaudi::Property<float> m_MaxD0{this, "MaxD0", 5, "Maximum D0 value for a track"};
  //! Cut off for Z0
  Gaudi::Property<float> m_MaxZ0{this, "MaxZ0", 5, "Maximum Z0 value for a track"};
  //! Cut off for momentum
  Gaudi::Property<float> m_MinPt{this, "MinPt", 1.0, "Minimum transverse momentum"};  // units GeV

  //! Default magnetic field value
  float m_Bz = 3.57;  // units Tesla
};
