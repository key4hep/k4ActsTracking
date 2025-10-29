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
#include "k4ActsTracking/ACTSAlgBase.hxx"

// ROOT
#include <TGeoManager.h>

// Gaudi
#include <GaudiKernel/MsgStream.h>

// DD4hep
#include <DD4hep/DD4hepUnits.h>
#include <DD4hep/Detector.h>

// ACTS
#include <Acts/Definitions/Units.hpp>
#include <Acts/Geometry/CylinderVolumeBuilder.hpp>
#include <Acts/Geometry/CylinderVolumeHelper.hpp>
#include <Acts/Geometry/ITrackingVolumeBuilder.hpp>
#include <Acts/Geometry/LayerArrayCreator.hpp>
#include <Acts/Geometry/LayerCreator.hpp>
#include <Acts/Geometry/ProtoLayerHelper.hpp>
#include <Acts/Geometry/SurfaceArrayCreator.hpp>
#include <Acts/Geometry/SurfaceBinningMatcher.hpp>
#include <Acts/Geometry/TrackingGeometryBuilder.hpp>
#include <Acts/Geometry/TrackingVolumeArrayCreator.hpp>
#include <Acts/MagneticField/ConstantBField.hpp>
#include <Acts/Plugins/Json/JsonMaterialDecorator.hpp>
#ifndef K4ACTSTRACKING_ACTS_V32
#include <Acts/Utilities/AxisDefinitions.hpp>
#endif

#ifdef K4ACTSTRACKING_ACTS_HAS_TGEO_PLUGIN
#include <Acts/Plugins/TGeo/TGeoDetectorElement.hpp>
#include <Acts/Plugins/TGeo/TGeoLayerBuilder.hpp>
#else
#include <Acts/Plugins/Root/TGeoDetectorElement.hpp>
#include <Acts/Plugins/Root/TGeoLayerBuilder.hpp>
#endif

// ACTSTracking
#include "k4ActsTracking/Helpers.hxx"

using namespace ACTSTracking;

ACTSAlgBase::ACTSAlgBase(const std::string& name, ISvcLocator* svcLoc)
    : MultiTransformer(
          name, svcLoc, {KeyValues("InputTrackerHitCollectionName", {"TrackerHits"})},
          {KeyValues("OutputSeedCollectionName", {"SeedTracks"}), KeyValues("OutputTrackCollectionName", {"Tracks"})}) {
  m_geoSvc = serviceLocator()->service("GeoSvc");  // important to initialize m_geoSvc
}

std::shared_ptr<GeometryIdMappingTool> ACTSAlgBase::geoIDMappingTool() const { return m_geoIDMappingTool; }

const Acts::MagneticFieldContext& ACTSAlgBase::magneticFieldContext() const { return m_magneticFieldContext; }

const Acts::GeometryContext& ACTSAlgBase::geometryContext() const { return m_geometryContext; }

const Acts::CalibrationContext& ACTSAlgBase::calibrationContext() const { return m_calibrationContext; }

std::shared_ptr<Acts::MagneticFieldProvider> ACTSAlgBase::magneticField() const { return m_magneticField; }

std::shared_ptr<const Acts::TrackingGeometry> ACTSAlgBase::trackingGeometry() const { return m_trackingGeometry; }

const Acts::Surface* ACTSAlgBase::findSurface(const edm4hep::TrackerHit hit) const {
  auto moduleGeoId = Acts::GeometryIdentifier(m_geoIDMappingTool->getGeometryID(hit));
  return m_trackingGeometry->findSurface(moduleGeoId);
}

StatusCode ACTSAlgBase::initialize() {
  // Parse parameters
  m_matFile      = findFile(m_matFile);
  m_tgeoFile     = findFile(m_tgeoFile);
  m_tgeodescFile = findFile(m_tgeodescFile);

  // Load geometry
  info() << " -------------------------------------" << endmsg;

  info() << " -- Building magnetic field" << endmsg;
  buildBfield();
  info() << " -- Building tracking detector" << endmsg;
  buildDetector();

  info() << " -------------------------------------" << endmsg;

  // Initialize mapping tool
  std::string initString;
  initString = m_geoSvc->constantAsString(m_encodingStringVariable.value());

  GeometryIdMappingTool::DetSchema dSchema;
  if (m_detectorSchema == "MuSIC_v1") {
    dSchema = GeometryIdMappingTool::DetSchema::MuSIC_v1;
  } else if (m_detectorSchema == "MuSIC_v2") {
    dSchema = GeometryIdMappingTool::DetSchema::MuSIC_v2;
  } else if (m_detectorSchema == "MAIA_v0") {
    dSchema = GeometryIdMappingTool::DetSchema::MAIA_v0;
  } else if (m_detectorSchema == "MuColl_v1") {
    dSchema = GeometryIdMappingTool::DetSchema::MuColl_v1;
  } else {
    dSchema = GeometryIdMappingTool::DetSchema::MuColl_v1;
  }

  m_geoIDMappingTool = std::make_shared<GeometryIdMappingTool>(initString, dSchema);
  return StatusCode::SUCCESS;
}

void ACTSAlgBase::buildDetector() {
  // Logging
  Acts::Logging::Level surfaceLogLevel = Acts::Logging::INFO;
  Acts::Logging::Level layerLogLevel   = Acts::Logging::INFO;
  Acts::Logging::Level volumeLogLevel  = Acts::Logging::INFO;

  // Material description
  std::shared_ptr<const Acts::IMaterialDecorator> matDeco = nullptr;
  if (!m_matFile.empty()) {
    // Set up the converter first
    Acts::MaterialMapJsonConverter::Config jsonGeoConvConfig;
    // Set up the json-based decorator
    matDeco = std::make_shared<const Acts::JsonMaterialDecorator>(jsonGeoConvConfig, m_matFile, Acts::Logging::INFO);
  }

  // Geometry
  TGeoManager* gGeoManagerOld = nullptr;
  if (!m_tgeoFile.empty()) {
    // Save current geometry. This is needed by all the other Processors
    gGeoManagerOld = gGeoManager;
    gGeoManager    = nullptr;  // prevents it from being deleted

    // Load new geometry
    TGeoManager::Import(m_tgeoFile.value().c_str());
  }

  // configure surface array creator
  Acts::SurfaceArrayCreator::Config sacConfig;
  auto                              surfaceArrayCreator = std::make_shared<const Acts::SurfaceArrayCreator>(
      sacConfig, Acts::getDefaultLogger("SurfaceArrayCreator", surfaceLogLevel));

  // configure the proto layer helper
  Acts::ProtoLayerHelper::Config plhConfig;
  auto                           protoLayerHelper = std::make_shared<const Acts::ProtoLayerHelper>(
      plhConfig, Acts::getDefaultLogger("ProtoLayerHelper", layerLogLevel));

  // configure the layer creator that uses the surface array creator
  Acts::LayerCreator::Config lcConfig;
  lcConfig.surfaceArrayCreator = surfaceArrayCreator;
  auto layerCreator =
      std::make_shared<const Acts::LayerCreator>(lcConfig, Acts::getDefaultLogger("LayerCreator", layerLogLevel));

  // configure the layer array creator
  Acts::LayerArrayCreator::Config lacConfig;
  auto                            layerArrayCreator = std::make_shared<const Acts::LayerArrayCreator>(
      lacConfig, Acts::getDefaultLogger("LayerArrayCreator", layerLogLevel));

  // tracking volume array creator
  Acts::TrackingVolumeArrayCreator::Config tvacConfig;
  auto tVolumeArrayCreator = std::make_shared<const Acts::TrackingVolumeArrayCreator>(
      tvacConfig, Acts::getDefaultLogger("TrackingVolumeArrayCreator", volumeLogLevel));

  // configure the cylinder volume helper
  Acts::CylinderVolumeHelper::Config cvhConfig;
  cvhConfig.layerArrayCreator          = layerArrayCreator;
  cvhConfig.trackingVolumeArrayCreator = tVolumeArrayCreator;
  auto cylinderVolumeHelper            = std::make_shared<const Acts::CylinderVolumeHelper>(
      cvhConfig, Acts::getDefaultLogger("CylinderVolumeHelper", volumeLogLevel));

  //-------------------------------------------------------------------------------------
  // list the volume builders
  std::list<std::shared_ptr<const Acts::ITrackingVolumeBuilder>> volumeBuilders;

  /**
   * Disable for now
  // Create a beam pipe if configured to do so
  auto beamPipeParameters =
      vm["geo-tgeo-bp-parameters"].template as<read_range>();
  if (beamPipeParameters.size() > 2) {
    /// configure the beam pipe layer builder
    Acts::PassiveLayerBuilder::Config bplConfig;
    bplConfig.layerIdentification = "BeamPipe";
    bplConfig.centralLayerRadii = std::vector<double>(1, beamPipeParameters[0]);
    bplConfig.centralLayerHalflengthZ =
        std::vector<double>(1, beamPipeParameters[1]);
    bplConfig.centralLayerThickness =
        std::vector<double>(1, beamPipeParameters[2]);
    auto beamPipeBuilder = std::make_shared<const Acts::PassiveLayerBuilder>(
        bplConfig,
        Acts::getDefaultLogger("BeamPipeLayerBuilder", layerLogLevel));
    // create the volume for the beam pipe
    Acts::CylinderVolumeBuilder::Config bpvConfig;
    bpvConfig.trackingVolumeHelper = cylinderVolumeHelper;
    bpvConfig.volumeName = "BeamPipe";
    bpvConfig.layerBuilder = beamPipeBuilder;
    bpvConfig.layerEnvelopeR = {1. * Acts::UnitConstants::mm,
                                1. * Acts::UnitConstants::mm};
    bpvConfig.buildToRadiusZero = true;
    auto beamPipeVolumeBuilder =
        std::make_shared<const Acts::CylinderVolumeBuilder>(
            bpvConfig,
            Acts::getDefaultLogger("BeamPipeVolumeBuilder", volumeLogLevel));
    // add to the list of builders
    volumeBuilders.push_back(beamPipeVolumeBuilder);
  }
  */

  //
  // Detector definition
  std::vector<Acts::TGeoLayerBuilder::Config> layerBuilderConfigs;

  // Check if the geometry has been defined
  if (m_tgeodescFile.empty()) {
    error() << "No TGeo description file provided for subdetectors! Cannot build tracking geometry." << endmsg;
    throw std::runtime_error("No TGeo description file provided for subdetectors! Cannot build tracking geometry.");
  }

  // Open the description file
  nlohmann::json tgeodesc;
  std::ifstream  tgeoDescFile(m_tgeodescFile.value(), std::ifstream::in | std::ifstream::binary);
  if (!tgeoDescFile.is_open()) {
    error() << "Could not open TGeo description file " << m_tgeodescFile.value() << endmsg;
    throw std::runtime_error("Could not open TGeo description file " + m_tgeodescFile.value());
  }
  tgeoDescFile >> tgeodesc;

  // Helper parsing functions
  auto range_from_json = [](const nlohmann::json& jsonpair) -> std::pair<double, double> {
    return {jsonpair["lower"], jsonpair["upper"]};
  };

  // Loop over volumes to define sub-detectors
  for (const auto& volume : tgeodesc["Volumes"]) {
    // Volume information
    Acts::TGeoLayerBuilder::Config layerBuilderConfig;
    layerBuilderConfig.configurationName  = volume["geo-tgeo-volume-name"];
    layerBuilderConfig.unit               = 1 * Acts::UnitConstants::cm;
    layerBuilderConfig.autoSurfaceBinning = true;

    // AutoBinning
    std::vector<std::pair<double, double>> binTolerances{numAxisDirections(), {0., 0.}};
#ifdef K4ACTSTRACKING_ACTS_V32
    binTolerances[Acts::binR]   = range_from_json(volume["geo-tgeo-sfbin-r-tolerance"]);
    binTolerances[Acts::binZ]   = range_from_json(volume["geo-tgeo-sfbin-z-tolerance"]);
    binTolerances[Acts::binPhi] = range_from_json(volume["geo-tgeo-sfbin-phi-tolerance"]);
#else
    binTolerances[static_cast<int>(Acts::AxisDirection::AxisR)] = range_from_json(volume["geo-tgeo-sfbin-r-tolerance"]);
    binTolerances[static_cast<int>(Acts::AxisDirection::AxisZ)] = range_from_json(volume["geo-tgeo-sfbin-z-tolerance"]);
    binTolerances[static_cast<int>(Acts::AxisDirection::AxisPhi)] =
        range_from_json(volume["geo-tgeo-sfbin-phi-tolerance"]);
#endif
    layerBuilderConfig.surfaceBinMatcher = Acts::SurfaceBinningMatcher(binTolerances);

    // Loop over subvolumes (two endcaps and one barrel)
    std::array<std::string, 3> subvolumeNames = {
        "negative", "central",
        "positive"};  // List of possible subvolume names. Order corresponds to layerConfigurations.
    for (std::size_t idx = 0; idx < 3; idx++) {
      const std::string& subvolumeName = subvolumeNames[idx];
      if (!volume["geo-tgeo-volume-layers"][subvolumeName]) {
        // Skip disabled volume
        continue;
      }

      // Create the layer config object and fill it
      Acts::TGeoLayerBuilder::LayerConfig lConfig;
      lConfig.volumeName  = volume["geo-tgeo-subvolume-names"][subvolumeName];
      lConfig.sensorNames = volume["geo-tgeo-sensitive-names"][subvolumeName];
      lConfig.localAxes   = volume["geo-tgeo-sensitive-axes"][subvolumeName];
      lConfig.envelope    = std::pair<double, double>(0.1 * Acts::UnitConstants::mm, 0.1 * Acts::UnitConstants::mm);

#ifdef K4ACTSTRACKING_ACTS_V32
      // Fill the parsing restrictions in r
      lConfig.parseRanges.push_back({Acts::binR, range_from_json(volume["geo-tgeo-layer-r-ranges"][subvolumeName])});

      // Fill the parsing restrictions in z
      lConfig.parseRanges.push_back({Acts::binZ, range_from_json(volume["geo-tgeo-layer-z-ranges"][subvolumeName])});

      // Fill the layer splitting parameters in r
      float rsplit = volume["geo-tgeo-layer-r-split"][subvolumeName];
      if (rsplit > 0) {
        lConfig.splitConfigs.push_back({Acts::binR, rsplit});
      }

      // Fill the layer splitting parameters in z
      float zsplit = volume["geo-tgeo-layer-z-split"][subvolumeName];
      if (zsplit > 0) {
        lConfig.splitConfigs.push_back({Acts::binZ, zsplit});
      }
#else
      // Fill the parsing restrictions in r
      lConfig.parseRanges.push_back(
          {Acts::AxisDirection::AxisR, range_from_json(volume["geo-tgeo-layer-r-ranges"][subvolumeName])});

      // Fill the parsing restrictions in z
      lConfig.parseRanges.push_back(
          {Acts::AxisDirection::AxisZ, range_from_json(volume["geo-tgeo-layer-z-ranges"][subvolumeName])});

      // Fill the layer splitting parameters in r
      float rsplit = volume["geo-tgeo-layer-r-split"][subvolumeName];
      if (rsplit > 0) {
        lConfig.splitConfigs.push_back({Acts::AxisDirection::AxisR, rsplit});
      }

      // Fill the layer splitting parameters in z
      float zsplit = volume["geo-tgeo-layer-z-split"][subvolumeName];
      if (zsplit > 0) {
        lConfig.splitConfigs.push_back({Acts::AxisDirection::AxisZ, zsplit});
      }
#endif

      // Save
      layerBuilderConfig.layerConfigurations[idx].push_back(lConfig);
    }

    // Save
    layerBuilderConfigs.push_back(layerBuilderConfig);
  }

  // remember the layer builders to collect the detector elements
  std::vector<std::shared_ptr<const Acts::TGeoLayerBuilder>> tgLayerBuilders;

  for (auto& lbc : layerBuilderConfigs) {
    std::shared_ptr<const Acts::LayerCreator> layerCreatorLB = nullptr;

    if (lbc.autoSurfaceBinning) {
      // Configure surface array creator (optionally) per layer builder
      // (in order to configure them to work appropriately)
      Acts::SurfaceArrayCreator::Config sacConfigLB;
      sacConfigLB.surfaceMatcher = lbc.surfaceBinMatcher;
      auto surfaceArrayCreatorLB = std::make_shared<const Acts::SurfaceArrayCreator>(
          sacConfigLB, Acts::getDefaultLogger(lbc.configurationName + "SurfaceArrayCreator", surfaceLogLevel));

      // configure the layer creator that uses the surface array creator
      Acts::LayerCreator::Config lcConfigLB;
      lcConfigLB.surfaceArrayCreator = surfaceArrayCreatorLB;
      layerCreatorLB                 = std::make_shared<const Acts::LayerCreator>(
          lcConfigLB, Acts::getDefaultLogger(lbc.configurationName + "LayerCreator", layerLogLevel));
    }

    // Configure the proto layer helper
    Acts::ProtoLayerHelper::Config plhConfigLB;
    auto                           protoLayerHelperLB = std::make_shared<const Acts::ProtoLayerHelper>(
        plhConfigLB, Acts::getDefaultLogger(lbc.configurationName + "ProtoLayerHelper", layerLogLevel));

    //-------------------------------------------------------------------------------------
    lbc.layerCreator     = (layerCreatorLB != nullptr) ? layerCreatorLB : layerCreator;
    lbc.protoLayerHelper = (protoLayerHelperLB != nullptr) ? protoLayerHelperLB : protoLayerHelper;

    auto layerBuilder = std::make_shared<const Acts::TGeoLayerBuilder>(
        lbc, Acts::getDefaultLogger(lbc.configurationName + "LayerBuilder", layerLogLevel));
    // remember the layer builder
    tgLayerBuilders.push_back(layerBuilder);

    // build the pixel volume
    Acts::CylinderVolumeBuilder::Config volumeConfig;
    volumeConfig.trackingVolumeHelper = cylinderVolumeHelper;
    volumeConfig.volumeName           = lbc.configurationName;
    volumeConfig.buildToRadiusZero    = (volumeBuilders.size() == 0);
    volumeConfig.layerEnvelopeR       = {1. * Acts::UnitConstants::mm, 5. * Acts::UnitConstants::mm};
    auto ringLayoutConfiguration      = [&](const std::vector<Acts::TGeoLayerBuilder::LayerConfig>& lConfigs) -> void {
      for (const auto& lcfg : lConfigs) {
        for (const auto& scfg : lcfg.splitConfigs) {
#ifdef K4ACTSTRACKING_ACTS_V32
          if (scfg.first == Acts::binR and scfg.second > 0.) {
#else
          if (scfg.first == Acts::AxisDirection::AxisR and scfg.second > 0.) {
#endif
            volumeConfig.ringTolerance   = std::max(volumeConfig.ringTolerance, scfg.second);
            volumeConfig.checkRingLayout = true;
          }
        }
      }
    };
    ringLayoutConfiguration(lbc.layerConfigurations[0]);
    ringLayoutConfiguration(lbc.layerConfigurations[2]);
    volumeConfig.layerBuilder = layerBuilder;
    auto volumeBuilder        = std::make_shared<const Acts::CylinderVolumeBuilder>(
        volumeConfig, Acts::getDefaultLogger(lbc.configurationName + "VolumeBuilder", volumeLogLevel));
    // add to the list of builders
    volumeBuilders.push_back(volumeBuilder);
  }

  //-------------------------------------------------------------------------------------
  // create the tracking geometry
  Acts::TrackingGeometryBuilder::Config tgConfig;
  // Add the builders
  tgConfig.materialDecorator = matDeco;

  for (auto& vb : volumeBuilders) {
    tgConfig.trackingVolumeBuilders.push_back(
        [=](const auto& gcontext, const auto& inner, const auto&) { return vb->trackingVolume(gcontext, inner); });
  }
  // Add the helper
  tgConfig.trackingVolumeHelper = cylinderVolumeHelper;
  auto cylinderGeometryBuilder  = std::make_shared<const Acts::TrackingGeometryBuilder>(
      tgConfig, Acts::getDefaultLogger("TrackerGeometryBuilder", volumeLogLevel));
  // get the geometry
  m_trackingGeometry = cylinderGeometryBuilder->trackingGeometry(m_geometryContext);
  // collect the detector element store
  for (auto& lBuilder : tgLayerBuilders) {
    auto detElements = lBuilder->detectorElements();
    m_detectorStore.insert(m_detectorStore.begin(), detElements.begin(), detElements.end());
  }

  //
  // Restore old gGeoManager
  if (gGeoManagerOld != nullptr) {
    gGeoManager = gGeoManagerOld;
  }
}

void ACTSAlgBase::buildBfield() {
  // Get the magnetic field
  dd4hep::Detector& lcdd        = dd4hep::Detector::getInstance();
  const double      position[3] = {0, 0, 0};  // position to calculate magnetic field at (the origin in this case)
  double            magneticFieldVector[3] = {0, 0, 0};  // initialise object to hold magnetic field
  lcdd.field().magneticField(position,
                             magneticFieldVector);  // get the magnetic field vector from DD4hep

  // Build ACTS representation of field
  // Note:
  //  magneticFieldVector[2] = 3.57e-13
  //  dd4hep::tesla = 1e-13
  //  Acts::UnitConstants::T = 0.000299792
  m_magneticField = std::make_shared<Acts::ConstantBField>(
      Acts::Vector3(magneticFieldVector[0] / dd4hep::tesla * Acts::UnitConstants::T,
                    magneticFieldVector[1] / dd4hep::tesla * Acts::UnitConstants::T,
                    magneticFieldVector[2] / dd4hep::tesla * Acts::UnitConstants::T));
}
