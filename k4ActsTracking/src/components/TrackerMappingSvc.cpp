#include "TrackerMappingSvc.h"

#include "k4ActsTracking/ActsGaudiLogger.h"

#include <k4FWCore/GaudiChecks.h>

#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/Surfaces/Surface.hpp>

#include <ActsPlugins/DD4hep/DD4hepDetectorElement.hpp>

#include <fmt/format.h>

DECLARE_COMPONENT(TrackerMappingSvc)

TrackerMappingSvc::TrackerMappingSvc(const std::string& name, ISvcLocator* svcLoc)
    : base_class(name, svcLoc) {}

StatusCode TrackerMappingSvc::initialize() {
  m_actsGeoSvc = service<IActsGeoSvc>(m_actsGeoSvcName.value());
  K4_GAUDI_CHECK(m_actsGeoSvc);

  // build BitFieldCoder from layout string
  try {
    m_decoder = std::make_unique<dd4hep::DDSegmentation::BitFieldCoder>(m_idLayout.value());
    // Detect whether x/y exist
    try {
      (void)m_decoder->get(0ULL, "x");
      m_hasX = true;
    } catch (...) {
      m_hasX = false;
    }
    try {
      (void)m_decoder->get(0ULL, "y");
      m_hasY = true;
    } catch (...) {
      m_hasY = false;
    }

    info() << fmt::format("TrackerMappingSvc: BitFieldCoder initialized. MaskXY={}, hasX={}, hasY={}, layout='{}'",
                          m_maskXY.value(), m_hasX, m_hasY, m_idLayout.value())
           << endmsg;

  } catch (const std::exception& e) {
    error() << fmt::format("TrackerMappingSvc: failed to construct BitFieldCoder from layout '{}': {}",
                           m_idLayout.value(), e.what())
            << endmsg;
    return StatusCode::FAILURE;
  }

  return buildMapping();
}

StatusCode TrackerMappingSvc::finalize() {
  std::scoped_lock lock{m_mutex};
  info() << fmt::format("Final mapping size={}, visited={}, withADE={}, dd4hepADE={}, inserted={}",
                        m_cellIDToSurface.size(), m_nSurfacesVisited, m_nWithAssociatedDetElem,
                        m_nDD4hepDetElem, m_nInserted)
         << endmsg;
  m_cellIDToSurface.clear();
  return StatusCode::SUCCESS;
}

std::uint64_t TrackerMappingSvc::normalizeCellID(std::uint64_t cellID) const {
  if (!m_maskXY.value() || !m_decoder) {
    return cellID;
  }
  std::uint64_t key = cellID;

  // mask x/y only if present
  // BitFieldCoder::set modifies the VolumeID in-place
  if (m_hasX) {
    m_decoder->set(key, "x", 0);
  }
  if (m_hasY) {
    m_decoder->set(key, "y", 0);
  }

  return key;
}

const Acts::Surface* TrackerMappingSvc::surface(std::uint64_t cellID) const {
  std::scoped_lock lock{m_mutex};

  const auto key = normalizeCellID(cellID);
  auto it = m_cellIDToSurface.find(key);
  return (it == m_cellIDToSurface.end()) ? nullptr : it->second;
}

bool TrackerMappingSvc::hasSurface(std::uint64_t cellID) const {
  std::scoped_lock lock{m_mutex};

  const auto key = normalizeCellID(cellID);
  return m_cellIDToSurface.find(key) != m_cellIDToSurface.end();
}

std::size_t TrackerMappingSvc::size() const {
  std::scoped_lock lock{m_mutex};
  return m_cellIDToSurface.size();
}

StatusCode TrackerMappingSvc::buildMapping() {
  std::scoped_lock lock{m_mutex};

  m_cellIDToSurface.clear();
  m_nSurfacesVisited        = 0;
  m_nWithAssociatedDetElem  = 0;
  m_nDD4hepDetElem          = 0;
  m_nInserted               = 0;

  auto tg = m_actsGeoSvc->trackingGeometry();
  if (!tg) {
    error() << "Acts tracking geometry is null. Cannot build CellID->Surface mapping." << endmsg;
    return StatusCode::FAILURE;
  }

  info() << fmt::format("Building CellID->Surface mapping from TrackingGeometry via visitSurfaces()")
         << endmsg;

  tg->visitSurfaces([&](const Acts::Surface* s) {
    ++m_nSurfacesVisited;
    if (s == nullptr) {
      return;
    }

    const auto* ade = s->associatedDetectorElement();
    if (ade == nullptr) {
      return;
    }
    ++m_nWithAssociatedDetElem;

    const auto* dd4hepDE = dynamic_cast<const ActsPlugins::DD4hepDetectorElement*>(ade);
    if (dd4hepDE == nullptr) {
      return;
    }
    ++m_nDD4hepDetElem;

    const auto cellID = static_cast<std::uint64_t>(dd4hepDE->sourceElement().volumeID());
    if (cellID == 0) {
      return;
    }

    auto [it, inserted] = m_cellIDToSurface.emplace(cellID, s);
    if (!inserted) {
      warning() << fmt::format("Duplicate cellID={} for surface (existing ptr={}, new ptr={})",
                               cellID, (const void*)it->second, (const void*)s)
                << endmsg;
      return;
    }
    ++m_nInserted;
  });

  info() << fmt::format(
               "Built CellID->Surface mapping: size={}, visited={}, withADE={}, dd4hepADE={}, inserted={}",
               m_cellIDToSurface.size(), m_nSurfacesVisited, m_nWithAssociatedDetElem,
               m_nDD4hepDetElem, m_nInserted)
         << endmsg;

  if (m_cellIDToSurface.empty()) {
    error() << "Mapping table is empty. This usually means surfaces have no DD4hepDetectorElement "
               "associated (wrong geometry builder output or visiting wrong surfaces)."
            << endmsg;
    return StatusCode::FAILURE;
  }

  return StatusCode::SUCCESS;
}

std::uint64_t TrackerMappingSvc::cellIDFromSurface(const Acts::Surface& surface) {
  const auto* ade = surface.associatedDetectorElement();
  if (ade == nullptr) {
    return 0;
  }

  const auto* dd4hepDE = dynamic_cast<const ActsPlugins::DD4hepDetectorElement*>(ade);
  if (dd4hepDE == nullptr) {
    return 0;
  }

  return static_cast<std::uint64_t>(dd4hepDE->sourceElement().volumeID());
}
