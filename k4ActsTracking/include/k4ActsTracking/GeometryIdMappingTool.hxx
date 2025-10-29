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
#include <edm4hep/SimTrackerHit.h>
#include <edm4hep/TrackerHit.h>
#include <edm4hep/TrackerHitPlane.h>

// DD4hep
#include <DDSegmentation/BitFieldCoder.h>

// ACTs
#include <Acts/Geometry/GeometryIdentifier.hpp>

// Standard
#include <string>
#include <unordered_map>
#include <vector>

namespace ACTSTracking {

  /**
 * @brief Maps DD4hep cell ID's to ACTS geometry ID's
 *
 * Lots of hardcoded values that match up to mod5 geometry
 * dumped to TGeo.
 *
 * @author Karol Krizka
 * @author Samuel Ferraro
 * @version $Id$
 */
  class GeometryIdMappingTool {
  public:
    enum class DetSchema : char { MuColl_v1, MAIA_v0, MuSIC_v1, MuSIC_v2 };

    using modules_map = std::unordered_map<uint32_t, uint32_t>;
    using det_mod_map = std::unordered_map<DetSchema, modules_map>;

    /**
 	 * @brief Create a mapping tool using the provided encoderString to interpret cell ID's.
 	 * @param encoderString string used to encode CellIDs
	 */
    GeometryIdMappingTool(const std::string& encoderString, DetSchema dType = DetSchema::MuColl_v1);

    /**
 	* @brief Decode Sim Tracker Hit Cell ID
 	* @param hit A Sim Tracker Hit
 	* @return decoded Cell ID ready to be passed to ACTS
	*/
	Acts::GeometryIdentifier getGeometryID(const edm4hep::SimTrackerHit& hit);
	/** 
        * @brief Decode Tracker Hit Cell ID
	* @TODO: This method and the one for TrackerHitPlanes only exist separately due to the current inheritance issues in edm4hep
        * @param hit A Tracker Hit
        * @return decoded Cell ID ready to be passed to ACTS
        */
	Acts::GeometryIdentifier getGeometryID(const edm4hep::TrackerHit& hit);
	/** 
        * @brief Decode Tracker Hit Plane Cell ID
        * @TODO: This method and the one for TrackerHitPlanes only exist separately due to the current inheritance issues in ed
m4hep
        * @param hit A Tracker Hit Plane
        * @return decoded Cell ID ready to be passed to ACTS
        */
	Acts::GeometryIdentifier getGeometryID(const edm4hep::TrackerHitPlane& hit);
	/**
	 * @brief A helper method to decode cell IDs
	 * @TODO: Once inhertance issue is fixed, this can be combined into the Tracker Hit (Plane) method
	 * @param cellID from Tracker Hit or Tracker Hit Plane
	 * @return decoded Cell ID ready to be passed to ACTS
	 */
	Acts::GeometryIdentifier getGeometryIDTrack(uint64_t cellID);
	
	/**
	 * @brief Takes decoded Cell ID and turns it into ACTS format
	 * @param *ID the IDs specific to each part of the detector
	 * @return Cell ID ready to be passed to ACTS
	 */
	Acts::GeometryIdentifier getGeometryID(uint32_t systemID, uint32_t layerID, int32_t sideID,
		uint32_t ladderID, uint32_t moduleID);

  private:
    /// Tool used to decode Cell IDs with encoder string
    dd4hep::DDSegmentation::BitFieldCoder m_decoder;
    
    const DetSchema                       det_type;

    /// Volume map to detector sections
    static const std::unordered_map<int32_t, uint32_t> VolumeMap;

    /// IDs for each part of the detector
    ///@{
    static const int32_t VertexEndCapNegative;
    static const int32_t VertexBarrel;
    static const int32_t VertexEndCapPositive;
    static const int32_t InnerTrackerEndCapNegative;
    static const int32_t InnerTrackerBarrel;
    static const int32_t InnerTrackerEndCapPositive;
    static const int32_t OuterInnerTrackerEndCapNegative;
    static const int32_t OuterInnerTrackerBarrel;
    static const int32_t OuterInnerTrackerEndCapPositive;
    static const int32_t OuterTrackerEndCapNegative;
    static const int32_t OuterTrackerBarrel;
    static const int32_t OuterTrackerEndCapPositive;
    ///@}

    /// Modules in phi ladder per layer
    ///@{
    static const det_mod_map NLad_VertexBarrel;
    static const det_mod_map NLad_InnerTrackerBarrel;
    static const det_mod_map NLad_OuterInnerTrackerBarrel;
    static const det_mod_map NLad_OuterTrackerBarrel;
    ///@}

    /// Modules in ring per layer
    ///@{
    static const det_mod_map NRng_VertexEndCap;
    static const det_mod_map NRng_InnerTrackerEndCap;
    static const det_mod_map NRng_OuterInnerTrackerEndCap;
    static const det_mod_map NRng_OuterTrackerEndCap;
    ///@}
  };

}  // namespace ACTSTracking
