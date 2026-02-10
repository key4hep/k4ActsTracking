#include <Acts/Definitions/Algebra.hpp>
#include <Acts/Definitions/Units.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Surfaces/SurfaceBounds.hpp>

#include <ActsPlugins/DD4hep/DD4hepDetectorElement.hpp>
#include "DumpSimTrackerHitCellIDAlg.h"
#include <k4FWCore/GaudiChecks.h>
#include <Acts/Geometry/GeometryIdentifier.hpp>
#include <Acts/Surfaces/Surface.hpp>

#include <fmt/format.h>

DECLARE_COMPONENT(DumpSimTrackerHitCellIDAlg)

DumpSimTrackerHitCellIDAlg::DumpSimTrackerHitCellIDAlg(const std::string& name, ISvcLocator* svcLoc)
    : Algorithm(name, svcLoc) {}

StatusCode DumpSimTrackerHitCellIDAlg::initialize() {
  K4_GAUDI_CHECK(Algorithm::initialize());

  // Rebind the DataHandle key to the property (so option file can override)
  m_inHits = DataHandle<edm4hep::SimTrackerHitCollection>(m_inputColName.value(),
                                                          Gaudi::DataHandle::Reader, this);

  // Configure ServiceHandle name (so option file can override)
  m_mappingSvc = ServiceHandle<ITrackerMappingSvc>(m_mappingSvcName.value(), name());

  K4_GAUDI_CHECK(m_mappingSvc.retrieve());

  info() << fmt::format("Initialized. InputCollection='{}', MappingSvc='{}'",
                        m_inputColName.value(), m_mappingSvcName.value())
         << endmsg;

  return StatusCode::SUCCESS;
}

StatusCode DumpSimTrackerHitCellIDAlg::execute() {
  const auto* hits = m_inHits.get();
  if (hits == nullptr) {
    warning() << "Input collection is missing in TES." << endmsg;
    return StatusCode::SUCCESS;
  }

  info() << fmt::format("Event: SimTrackerHits size={}", hits->size()) << endmsg;

  int nPrint = 0;
  for (const auto& h : *hits) {
    if (nPrint >= m_maxHits.value()) {
      break;
    }

    const std::uint64_t cellID = static_cast<std::uint64_t>(h.getCellID());
    const auto* surf = m_mappingSvc->surface(cellID);

    // Print some hit info for sanity
    const auto p = h.getPosition();   // edm4hep::Vector3f
    const auto m = h.getMomentum();   // edm4hep::Vector3f (at entry)
    const double t = h.getTime();

    if (surf == nullptr) {
      warning() << fmt::format("  hit[{}] cellID={}  pos=({:.3f},{:.3f},{:.3f})  "
                               "mom=({:.3f},{:.3f},{:.3f})  t={:.3f}  ==> NO SURFACE",
                               nPrint, cellID, p.x, p.y, p.z, m.x, m.y, m.z, t)
                << endmsg;
    } else {
      // Acts context (stateless for now)
      Acts::GeometryContext gctx{};

      // Convert hit pos/mom to Acts vectors (assume EDM4hep uses mm / GeV in your chain;
      // geometry was built with mm scale already, so treat numbers as mm here)
      const Acts::Vector3 gpos(p.x, p.y, p.z);

      Acts::Vector3 gdir(m.x, m.y, m.z);
      const double dirNorm = gdir.norm();
      if (dirNorm > 0.) {
        gdir /= dirNorm;
      } else {
        // fallback direction
        gdir = Acts::Vector3(0., 0., 1.);
      }

      // Basic IDs
      const auto gid = surf->geometryId();

      // Surface transform info
      const Acts::Transform3& T = surf->transform(gctx);
      const Acts::Vector3 center = T.translation();

      // Surface normal in global frame
      const Acts::Vector3 n = surf->normal(gctx, gpos, gdir);

      // Signed distance to the plane along normal
      const double signedDist = n.dot(gpos - center);

      auto lres = surf->globalToLocal(gctx, gpos, gdir); //default tolerance
      bool gotLocal = lres.ok();
      Acts::Vector2 lpos(0., 0.);
      if (gotLocal) {
        lpos = *lres;
      }
      
      bool inside = gotLocal ? surf->bounds().inside(lpos) : false;

      // DD4hep DetElement semantic check
      std::string dePath = "<no ADE>";
      std::string deName = "<no ADE>";
      std::uint64_t deVid = 0;

      const auto* ade = surf->associatedDetectorElement();
      if (ade != nullptr) {
        if (const auto* dd4hepDE =
                dynamic_cast<const ActsPlugins::DD4hepDetectorElement*>(ade)) {
          const auto& src = dd4hepDE->sourceElement();
          dePath = src.path();
          deName = src.name();
          deVid  = static_cast<std::uint64_t>(src.volumeID());
        } else {
          dePath = "<ADE not DD4hepDetectorElement>";
          deName = "<ADE not DD4hepDetectorElement>";
        }
      }

      // Print summary
      info() << fmt::format(
                  "  hit[{}] cellID={}  pos=({:.3f},{:.3f},{:.3f}) ==> "
                  "surface={} geoId=0x{:x} | "
                  "center=({:.3f},{:.3f},{:.3f}) n=({:.3f},{:.3f},{:.3f}) "
                  "dist={:.3f}mm | local={} ({:.3f},{:.3f}) inside={} | "
                  "DE name='{}' path='{}' volID={}",
                  nPrint, cellID, p.x, p.y, p.z,
                  (const void*)surf,
                  static_cast<std::uint64_t>(gid.value()),
                  center.x(), center.y(), center.z(),
                  n.x(), n.y(), n.z(),
                  signedDist,
                  gotLocal ? "OK" : "FAIL",
                  lpos.x(), lpos.y(),
                  inside ? "YES" : "NO",
                  deName, dePath, deVid)
             << endmsg;
    }
//    } else {
//      const auto gid = surf->geometryId();
//      info() << fmt::format("  hit[{}] cellID={}  pos=({:.3f},{:.3f},{:.3f})  "
//                            "==> surface={}  geoId=0x{:x}",
//                            nPrint, cellID, p.x, p.y, p.z, (const void*)surf,
//                            static_cast<std::uint64_t>(gid.value()))
//             << endmsg;
//    }// else

    ++nPrint;
  }

  return StatusCode::SUCCESS;
}
