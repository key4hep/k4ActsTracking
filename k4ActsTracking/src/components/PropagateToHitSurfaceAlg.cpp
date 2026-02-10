#include "PropagateToHitSurfaceAlg.h"

#include <k4FWCore/GaudiChecks.h>

#include <Acts/Definitions/Algebra.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/Propagator/EigenStepper.hpp>
#include <Acts/Propagator/Navigator.hpp>
#include <Acts/Propagator/Propagator.hpp>
#include <Acts/EventData/ParticleHypothesis.hpp>
#include <Acts/EventData/GenericBoundTrackParameters.hpp>
#include <Acts/Surfaces/Surface.hpp>

#include <fmt/format.h>
#include <cmath>
#include <fstream>

DECLARE_COMPONENT(PropagateToHitSurfaceAlg)

PropagateToHitSurfaceAlg::PropagateToHitSurfaceAlg(const std::string& name, ISvcLocator* svcLoc)
    : Gaudi::Algorithm(name, svcLoc) {}

StatusCode PropagateToHitSurfaceAlg::initialize() {
  K4_GAUDI_CHECK(Gaudi::Algorithm::initialize());
  K4_GAUDI_CHECK(m_mappingSvc.retrieve());
  K4_GAUDI_CHECK(m_actsGeoSvc.retrieve());

m_inHits = k4FWCore::DataHandle<edm4hep::SimTrackerHitCollection>(
    m_inputColName.value(), Gaudi::DataHandle::Reader, this);


  // Acts logger, same pattern as ActsTestPropagator
  m_actsLogger = makeActsGaudiLogger(this);

  info() << fmt::format(
      "Initialized. Input='{}', MappingSvc='{}', ActsGeoSvc='{}', MaxHits={}, Backstep={} mm, AssumeCharge={}",
      m_inputColName.value(),
      m_mappingSvc.name(),
      m_actsGeoSvc.name(),
      m_maxHits.value(),
      m_backstepMm.value(),
      m_assumeCharge.value())
  << endmsg;

  // Create/overwrite CSV
{
  std::ofstream ofs(m_csvFile.value(), std::ios::out | std::ios::trunc);
  ofs << "event,ihit,cellID,"
         "hit_x,hit_y,hit_z,"
         "start_x,start_y,start_z,"
         "end_x,end_y,end_z,"
         "dist_mm,local_u,local_v,inside,local_ok\n";
}
info() << fmt::format("CSV output: '{}'", m_csvFile.value()) << endmsg;
 // Create/overwrite obj
{
  std::ofstream obj(m_objFile.value(), std::ios::out | std::ios::trunc);
  obj << "# PropagateToHitSurfaceAlg segments\n";
  obj << "# v x y z\n";
  obj << "# l i j\n";
}
info() << fmt::format("OBJ output: '{}'", m_objFile.value()) << endmsg;

  return StatusCode::SUCCESS;
}// init

StatusCode PropagateToHitSurfaceAlg::execute(const EventContext& ctx) const {
  (void)ctx;

  const auto* hits = m_inHits.get();

  if (hits == nullptr) {
    warning() << "Input collection missing." << endmsg;
    return StatusCode::SUCCESS;
  }

  const auto tg = m_actsGeoSvc->trackingGeometry();
  if (!tg) {
    error() << "ActsGeoSvc returned null TrackingGeometry." << endmsg;
    return StatusCode::FAILURE;
  }

  // Contexts (you can later thread real contexts through here if needed)
  Acts::GeometryContext gctx{};
  Acts::MagneticFieldContext bctx{};

  // --- Build propagator like ActsTestPropagator (EigenStepper + Navigator + Logger)
  using Stepper    = Acts::EigenStepper<>;
  using Navigator  = Acts::Navigator;
  using Propagator = Acts::Propagator<Stepper, Navigator>;

  Navigator::Config navCfg{tg};
  navCfg.resolvePassive   = false;
  navCfg.resolveMaterial  = false;
  navCfg.resolveSensitive = true;

  Stepper stepper(m_actsGeoSvc->magneticField());
  Navigator navigator(navCfg, m_actsLogger->cloneWithSuffix(":Nav"));
  Propagator propagator(std::move(stepper), std::move(navigator),
                        m_actsLogger->cloneWithSuffix(":Prop"));

  // Minimal actor list is fine; keep EndOfWorldReached as in ActsTestPropagator
  using EndOfWorld = Acts::EndOfWorldReached;
  using ActorList  = Acts::ActorList<EndOfWorld>;
  using PropagatorOptions = Propagator::template Options<ActorList>;

  PropagatorOptions options{gctx, bctx};

  info() << fmt::format("Event: {} size={}", m_inputColName.value(), hits->size()) << endmsg;
  std::ofstream csv(m_csvFile.value(), std::ios::out | std::ios::app);
  if (!csv) {
    warning() << fmt::format("Cannot open CSV file '{}'", m_csvFile.value()) << endmsg;
  }
  std::ofstream obj(m_objFile.value(), std::ios::out | std::ios::app);
  if (!obj) {
    warning() << fmt::format("Cannot open OBJ file '{}'", m_objFile.value()) << endmsg;
  }
  static std::atomic<long long> globalVtx{1};

  int nPrint = 0;
  for (const auto& h : *hits) {
    if (nPrint >= m_maxHits.value()) break;

    const std::uint64_t cellID = static_cast<std::uint64_t>(h.getCellID());
    const Acts::Surface* surf  = m_mappingSvc->surface(cellID);

    const auto p = h.getPosition();
    const auto m = h.getMomentum();

    Acts::Vector3 hitPos(p.x, p.y, p.z);

    Acts::Vector3 mom(m.x, m.y, m.z);
    const double pAbs = mom.norm();
    if (pAbs <= 0.) {
      warning() << fmt::format("  hit[{}] cellID={} has zero momentum vector.", nPrint, cellID) << endmsg;
      ++nPrint;
      continue;
    }
    Acts::Vector3 dir = mom / pAbs;

    if (surf == nullptr) {
      warning() << fmt::format("  hit[{}] cellID={} ==> NO SURFACE", nPrint, cellID) << endmsg;
      ++nPrint;
      continue;
    }

    // Backstep upstream to avoid starting exactly on the surface
    const Acts::Vector3 startPos =
        hitPos - (m_backstepMm.value() * Acts::UnitConstants::mm) * dir;

    // --- Build start parameters in the same style as ActsTestPropagator
    // Momentum is assumed in GeV from edm4hep SimTrackerHit; we just need qOverP numerically.
    // If you want "pure straight-line validation", set AssumeCharge=0 -> qOverP=0.
    const double qOverP = (m_assumeCharge.value() == 0.0) ? 0.0 : (m_assumeCharge.value() / pAbs);

    Acts::Vector4 startPos4{startPos.x(), startPos.y(), startPos.z(), 0.0};

    const auto startParams = Acts::BoundTrackParameters::createCurvilinear(
        startPos4, dir, qOverP, std::nullopt, Acts::ParticleHypothesis::pion());

    // --- Propagate to target surface
    auto result = propagator.propagate(startParams, *surf, options);
    if (!result.ok()) {
      warning() << fmt::format("  hit[{}] cellID={} propagate FAILED: {}", nPrint, cellID,
                               result.error().message())
                << endmsg;
      ++nPrint;
      continue;
    }

    const auto& propRes = result.value();

    if (!propRes.endParameters) {
      warning() << fmt::format("  hit[{}] cellID={} propagate OK but endParameters is empty", nPrint, cellID)
                << endmsg;
      ++nPrint;
      continue;
    }

    const auto& endParams = *(propRes.endParameters);
    Acts::Vector3 endPos  = endParams.position(gctx);

    const double dist = (endPos - hitPos).norm();

    // Local check
    auto glRes  = surf->globalToLocal(gctx, endPos, dir);
    bool localOk = glRes.ok();
    Acts::Vector2 lpos = localOk ? glRes.value() : Acts::Vector2(0, 0);

    bool inside = false;
    if (localOk) {
      inside = surf->bounds().inside(lpos);
    }

    info() << fmt::format(
                "  hit[{}] cellID={} hit=({:.3f},{:.3f},{:.3f}) start=({:.3f},{:.3f},{:.3f}) "
                "-> end=({:.3f},{:.3f},{:.3f}) | |end-hit|={:.6f} mm | local={} ({:.3f},{:.3f}) inside={}",
                nPrint, cellID,
                hitPos.x(), hitPos.y(), hitPos.z(),
                startPos.x(), startPos.y(), startPos.z(),
                endPos.x(), endPos.y(), endPos.z(),
                dist / Acts::UnitConstants::mm,
                (localOk ? "OK" : "FAIL"),
                lpos.x(), lpos.y(),
                (inside ? "YES" : "NO"))
           << endmsg;

    // csv + obj
    if (csv) {
    csv << 0 << ","
        << nPrint << ","
        << cellID << ","
        << hitPos.x()  << "," << hitPos.y()  << "," << hitPos.z()  << ","
        << startPos.x() << "," << startPos.y() << "," << startPos.z() << ","
        << endPos.x()  << "," << endPos.y()  << "," << endPos.z()  << ","
        << (dist / Acts::UnitConstants::mm) << ","
        << (localOk ? lpos.x() : 0.0) << ","
        << (localOk ? lpos.y() : 0.0) << ","
        << (inside ? 1 : 0) << ","
        << (localOk ? 1 : 0)
        << "\n";
    }
    if (obj) {
      long long v0 = globalVtx.fetch_add(2);
      long long v1 = v0 + 1;

      obj << "v " << startPos.x() << " " << startPos.y() << " " << startPos.z() << "\n";
      obj << "v " << endPos.x()   << " " << endPos.y()   << " " << endPos.z()   << "\n";
      obj << "l " << v0 << " " << v1 << "\n";
    }


    ++nPrint;
  }

  return StatusCode::SUCCESS;
}

