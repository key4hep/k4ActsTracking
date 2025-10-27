#include "k4ActsTracking/GeometryIdMappingTool.hxx"

#include <iomanip>
#include <sstream>
#include <iostream>

using namespace ACTSTracking;

using det_mod_map = GeometryIdMappingTool::det_mod_map;

// Dector part identifications
const int32_t GeometryIdMappingTool::VertexEndCapNegative = -2;
const int32_t GeometryIdMappingTool::VertexBarrel = 1;
const int32_t GeometryIdMappingTool::VertexEndCapPositive = 2;
const int32_t GeometryIdMappingTool::InnerTrackerEndCapNegative = -4;
const int32_t GeometryIdMappingTool::InnerTrackerBarrel = 3;
const int32_t GeometryIdMappingTool::InnerTrackerEndCapPositive = 4;
const int32_t GeometryIdMappingTool::OuterInnerTrackerEndCapNegative = -8;
const int32_t GeometryIdMappingTool::OuterInnerTrackerBarrel = 7;
const int32_t GeometryIdMappingTool::OuterInnerTrackerEndCapPositive = 8;
const int32_t GeometryIdMappingTool::OuterTrackerEndCapNegative = -6;
const int32_t GeometryIdMappingTool::OuterTrackerBarrel = 5;
const int32_t GeometryIdMappingTool::OuterTrackerEndCapPositive = 6;

const det_mod_map GeometryIdMappingTool::NLad_VertexBarrel = {
  { GeometryIdMappingTool::DetSchema::MuColl_v1, {
    {0, 5}, {1, 5}, {2, 5}, {3, 5}, {4, 5}, {5, 5}, {6, 5}, {7, 5}} },
  { GeometryIdMappingTool::DetSchema::MAIA_v0, {
    {0, 5}, {1, 5}, {2, 5}, {3, 5}, {4, 5} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v1, {
    {0, 5}, {2, 5}, {4, 5}, {6, 5}} },
  { GeometryIdMappingTool::DetSchema::MuSIC_v2, {
    {0, 5}, {2, 5}, {4, 5}, {6, 5}, {8, 5}} },
};


const det_mod_map GeometryIdMappingTool::NRng_VertexEndCap = {
  { GeometryIdMappingTool::DetSchema::MuColl_v1, {
    {0, 16}, {1, 16}, {2, 16}, {3, 16}, {4, 16}, {5, 16}, {6, 16}, {7, 16}} },
  { GeometryIdMappingTool::DetSchema::MAIA_v0, {
    {0, 16}, {1, 16}, {2, 16}, {3, 16}, {4, 16}, {5, 16}, {6, 16}, {7, 16}} },
  { GeometryIdMappingTool::DetSchema::MuSIC_v1, {
    {0, 16}, {2, 16}, {4, 16}, {6,16} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v2, {
    {0, 16}, {2, 16}, {4, 16}, {6,16} } }
};

const det_mod_map GeometryIdMappingTool::NLad_InnerTrackerBarrel = {
  { GeometryIdMappingTool::DetSchema::MuColl_v1, { {0, 32}, {1, 32} } },
  { GeometryIdMappingTool::DetSchema::MAIA_v0, { {0, 32}, {1, 32} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v1, { {0, 32}, {1, 32} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v2, { {0, 32}, {1, 32} } }
};

const det_mod_map GeometryIdMappingTool::NRng_InnerTrackerEndCap = {
  { GeometryIdMappingTool::DetSchema::MuColl_v1, { {0, 26} } },
  { GeometryIdMappingTool::DetSchema::MAIA_v0, { {0, 26} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v1, { {0, 26} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v2, { {0, 26} } }
};

const det_mod_map GeometryIdMappingTool::NLad_OuterInnerTrackerBarrel = {
  { GeometryIdMappingTool::DetSchema::MuColl_v1, { {2, 46} } },
  { GeometryIdMappingTool::DetSchema::MAIA_v0, { {2, 46} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v1, { {2, 46} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v2, { {2, 46} } }
};

const det_mod_map GeometryIdMappingTool::NRng_OuterInnerTrackerEndCap = {
  { GeometryIdMappingTool::DetSchema::MuColl_v1, { 
    {1, 26}, {2, 26}, {3, 26}, {4, 26}, {5, 26}, {6, 26} } },
  { GeometryIdMappingTool::DetSchema::MAIA_v0, { 
    {1, 26}, {2, 26}, {3, 26}, {4, 26}, {5, 26}, {6, 26} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v1, { 
    {1, 26}, {2, 26}, {3, 26}, {4, 26}, {5, 26}, {6, 26} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v2, { 
    {1, 26}, {2, 26}, {3, 26}, {4, 26}, {5, 26}, {6, 26} } }
};

const det_mod_map GeometryIdMappingTool::NLad_OuterTrackerBarrel = {
  { GeometryIdMappingTool::DetSchema::MuColl_v1, { 
    {0, 84}, {1, 84}, {2, 84} } },
  { GeometryIdMappingTool::DetSchema::MAIA_v0, { 
    {0, 84}, {1, 84}, {2, 84} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v1, { 
    {0, 84}, {1, 84}, {2, 84} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v2, { 
    {0, 84}, {1, 84}, {2, 84} } }
};

const det_mod_map GeometryIdMappingTool::NRng_OuterTrackerEndCap = {
  { GeometryIdMappingTool::DetSchema::MuColl_v1, { 
    {0, 48}, {1, 48}, {2, 48}, {3, 48}, {4, 48}, {5, 48}, {6, 48}, {7, 48} } },
  { GeometryIdMappingTool::DetSchema::MAIA_v0, { 
    {0, 48}, {1, 48}, {2, 48}, {3, 48}, {4, 48}, {5, 48}, {6, 48}, {7, 48} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v1, { 
    {0, 48}, {1, 48}, {2, 48}, {3, 48}, {4, 48}, {5, 48}, {6, 48}, {7, 48} } },
  { GeometryIdMappingTool::DetSchema::MuSIC_v2, { 
    {0, 48}, {1, 48}, {2, 48}, {3, 48}, {4, 48}, {5, 48}, {6, 48}, {7, 48} } }
};

const std::unordered_map<int32_t, uint32_t> GeometryIdMappingTool::VolumeMap = {
    {VertexEndCapNegative, 13},
    {VertexBarrel, 14},
    {VertexEndCapPositive, 15},
    {InnerTrackerEndCapNegative, 8},
    {InnerTrackerBarrel, 17},
    {InnerTrackerEndCapPositive, 18},
    {OuterInnerTrackerEndCapNegative, 3},
    {OuterInnerTrackerBarrel, 20},
    {OuterInnerTrackerEndCapPositive, 21},
    {OuterTrackerEndCapNegative, 23},
    {OuterTrackerBarrel, 24},
    {OuterTrackerEndCapPositive, 25},
};

GeometryIdMappingTool::GeometryIdMappingTool(const std::string& encoderString,
                                             DetSchema dType) 
	: m_decoder(encoderString), det_type(dType) {}

uint64_t GeometryIdMappingTool::getGeometryID(const edm4hep::SimTrackerHit& hit) {
	// Decode Cell ID
	uint64_t cellID = hit.getCellID();

	// Encode ACTS ID
	return getGeometryID(
    m_decoder.get(cellID, "system"), 
		m_decoder.get(cellID, "layer"), 
		m_decoder.get(cellID, "side"),
		m_decoder.get(cellID, "module"),
		m_decoder.get(cellID, "sensor")
  );
}

uint64_t GeometryIdMappingTool::getGeometryID(const edm4hep::TrackerHitPlane& hit) {
	return GeometryIdMappingTool::getGeometryIDTrack(hit.getCellID());
}

uint64_t GeometryIdMappingTool::getGeometryID(const edm4hep::TrackerHit& hit) {
  return GeometryIdMappingTool::getGeometryIDTrack(hit.getCellID());
}

uint64_t GeometryIdMappingTool::getGeometryIDTrack(uint64_t cellID) {
	// Encode ACTS ID
  return getGeometryID(
    m_decoder.get(cellID, "system"), 
		m_decoder.get(cellID, "layer"), 
		m_decoder.get(cellID, "side"),
		m_decoder.get(cellID, "module"),
		m_decoder.get(cellID, "sensor")
  );
}

uint64_t GeometryIdMappingTool::getGeometryID(
    uint32_t systemID,
    uint32_t layerID, 
    int32_t sideID,
    uint32_t ladderID,
    uint32_t moduleID) {
  // Initialize geometry ID
  uint64_t geometry_id = 0;

  //
  // Volume ID determination.

  // the outermost layer of InnerTracker is "OuterInnerTracker" in ACTS
  if (systemID == InnerTrackerBarrel && layerID == 2) {
    systemID = OuterInnerTrackerBarrel;
  }
  if (systemID == InnerTrackerEndCapPositive && layerID != 0) {
    systemID = OuterInnerTrackerEndCapPositive;
  }

  // endcap is split in +/- sides by ACTS
  int32_t signSystemID = (sideID < 0) ? -systemID : systemID;

  // Map
  uint64_t volume_id = (VolumeMap.find(signSystemID) != VolumeMap.end())
                           ? VolumeMap.at(signSystemID)
                           : signSystemID;
  geometry_id |= volume_id << (14 * 4);

  // Layer ID is counting within sub detector, with pairings depending on the
  // sub detector
  uint64_t layer_id;
  switch (signSystemID) {
    case VertexEndCapNegative:
      switch(det_type){
        case GeometryIdMappingTool::DetSchema::MuColl_v1:
          layer_id = 2 * (7 - layerID) + 2;
          break;
        case GeometryIdMappingTool::DetSchema::MAIA_v0:
          layer_id = 2 * (7 - layerID) + 2;
          break;
        case GeometryIdMappingTool::DetSchema::MuSIC_v1:
          layer_id = (0 - layerID) + 8;
          break;
        case GeometryIdMappingTool::DetSchema::MuSIC_v2:
          layer_id = (0 - layerID) + 8;
          break;
      }
      break;
    case VertexEndCapPositive:
      switch(det_type){
        case GeometryIdMappingTool::DetSchema::MuColl_v1:
          layer_id = 2 * (layerID) + 2;
          break;
        case GeometryIdMappingTool::DetSchema::MAIA_v0:
          layer_id = 2 * (layerID) + 2;
          break;
        case GeometryIdMappingTool::DetSchema::MuSIC_v1:
          layer_id = layerID + 2; 
          break;
        case GeometryIdMappingTool::DetSchema::MuSIC_v2:
          layer_id = layerID + 2; 
          break;
      }
      break;
    case VertexBarrel:
      layer_id = layerID + 2;
      if (det_type == GeometryIdMappingTool::DetSchema::MuColl_v1) layer_id = 2 * (layerID + 1);
      if (det_type == GeometryIdMappingTool::DetSchema::MAIA_v0){
        if(layerID==0) layer_id = 2;
        if(layerID==1) layer_id = 4;
        if(layerID==2) layer_id = 6;
        if(layerID==4) layer_id = 8;
        if(layerID==6) layer_id = 10;
      }
      break;
    case InnerTrackerBarrel:
    case OuterTrackerBarrel: {
      layer_id = 2 * layerID + 2;
    } break;
    case OuterInnerTrackerBarrel: {
      layer_id = 2 * (2 - layerID) + 2;
    } break;

    case InnerTrackerEndCapNegative: {
      layer_id = 2 * (0 - layerID) + 2;
    } break;
    case InnerTrackerEndCapPositive: {
      layer_id = 2 * layerID + 2;
    } break;

      // OuterInner tracker layer counting starts at layer 1
      // as it is layer 1 of the original Inner tracker.
    case OuterInnerTrackerEndCapNegative: {
      layer_id = 2 * (7 - layerID) + 0;
    } break;
    case OuterInnerTrackerEndCapPositive: {
      layer_id = 2 * layerID + 0;
    } break;

    case OuterTrackerEndCapNegative: {
      layer_id = 2 * (3 - layerID) + 2;
    } break;
    case OuterTrackerEndCapPositive: {
      layer_id = 2 * layerID + 2;
    } break;

    default:
      layer_id = layerID;
      break;
  }
  geometry_id |= layer_id << (9 * 4);


  // Module ID counting depends on sub detector and layer
  uint64_t sensitive_id;
  switch (signSystemID) {
    case VertexBarrel:
      if (det_type == GeometryIdMappingTool::DetSchema::MAIA_v0){
        uint32_t my_layer_ID; 
        switch (layerID) {
          case 0:
            my_layer_ID = 0;
            break;
          case 1:
            my_layer_ID = 1;
            break;
          case 2:
            my_layer_ID = 2;
            break;
          case 4:
            my_layer_ID = 3;
            break;
          case 6:
            my_layer_ID = 4;
            break;
        }
        sensitive_id = NLad_VertexBarrel.at(det_type).at(my_layer_ID) * ladderID + moduleID + 1;
        break;
      }
      else{
        sensitive_id = NLad_VertexBarrel.at(det_type).at(layerID) * ladderID + moduleID + 1;
        break;
      }
    case InnerTrackerBarrel:
      sensitive_id =
          NLad_InnerTrackerBarrel.at(det_type).at(layerID) * ladderID + moduleID + 1;
      break;
    case OuterInnerTrackerBarrel:
      sensitive_id =
          NLad_OuterInnerTrackerBarrel.at(det_type).at(layerID) * ladderID + moduleID + 1;
      break;
    case OuterTrackerBarrel:
      sensitive_id =
          NLad_OuterTrackerBarrel.at(det_type).at(layerID) * ladderID + moduleID + 1;
      break;

    case VertexEndCapNegative:
      sensitive_id = 16 - moduleID;
      break;
    case VertexEndCapPositive:
      sensitive_id = moduleID + 1;
      break;

    case InnerTrackerEndCapNegative:
    case InnerTrackerEndCapPositive:
      sensitive_id =
          NRng_InnerTrackerEndCap.at(det_type).at(layerID) * ladderID + moduleID + 1;
      break;

    case OuterInnerTrackerEndCapNegative:
    case OuterInnerTrackerEndCapPositive:
      sensitive_id =
          NRng_OuterInnerTrackerEndCap.at(det_type).at(layerID) * ladderID + moduleID + 1;
      break;

    case OuterTrackerEndCapNegative:
    case OuterTrackerEndCapPositive:
      sensitive_id =
          NRng_OuterTrackerEndCap.at(det_type).at(layerID) * ladderID + moduleID + 1;
      break;

    default:
      sensitive_id = ladderID;
      break;
  }
  geometry_id |= sensitive_id << (2 * 4);

  return geometry_id;
}
