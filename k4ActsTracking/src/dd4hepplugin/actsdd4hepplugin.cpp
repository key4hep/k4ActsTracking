#include <DD4hep/DetElement.h>
#include <DD4hep/Detector.h>
#include <DD4hep/Factories.h>
#include <DD4hep/Printout.h>

#include <DD4hep/IDDescriptor.h>

#include <DDRec/DetectorData.h>
#include <DDRec/SurfaceHelper.h>
#include <DDRec/SurfaceManager.h>

#include <Acts/Plugins/DD4hep/DD4hepDetectorElement.hpp>
#include <Acts/Surfaces/PlaneSurface.hpp>
#include <Acts/Surfaces/TrapezoidBounds.hpp>
#include <Acts/Utilities/Logger.hpp>
#include <Acts/Visualization/GeometryView3D.hpp>
#include <Acts/Visualization/ObjVisualization3D.hpp>

#include <stdexcept>
#include <string>

using dd4hep::DetElement;
using dd4hep::PrintLevel;

namespace {

  /** Plugin for adding the ACTS Extensions to surface, or DetElements, or something
   *
   */

  std::pair<std::shared_ptr<Acts::PlanarBounds>, double> convertShapePlane(const TGeoShape&    shape,
                                                                           const std::string&  axes,
                                                                           Acts::LoggerWrapper logger) {
    ACTS_VERBOSE("Converting TGeoShape to plane with local axes: " << axes);

    std::shared_ptr<Acts::PlanarBounds> bounds;
    double                              thickness;
    if (const auto* trapezoid1 = dynamic_cast<const TGeoTrd1*>(&shape); trapezoid1 != nullptr) {
      ACTS_VERBOSE("Have TGeoTrd1");
    } else if (const auto* trapezoid2 = dynamic_cast<const TGeoTrd2*>(&shape); trapezoid2 != nullptr) {
      ACTS_VERBOSE("Have TGeoTrd2");

      double dx1 = trapezoid2->GetDx1();
      double dx2 = trapezoid2->GetDx2();
      double dy1 = trapezoid2->GetDy1();
      double dy2 = trapezoid2->GetDy2();
      double dz  = trapezoid2->GetDz();
      ACTS_VERBOSE(dx1 << " " << dx2 << " " << dy1 << " " << dy2 << " " << dz);

      if (axes == "zxy") {
        if (std::abs(dy1 - dy2) > Acts::s_epsilon) {
          throw std::runtime_error{"Inconsistent axes"};
        }
        bounds    = std::make_shared<Acts::TrapezoidBounds>(dx1, dx2, dz);
        thickness = dy1;
      } else {
        throw std::runtime_error{"Axes " + axes + " not yet implemented"};
      }

      return {bounds, thickness};
    } else if (const auto* box = dynamic_cast<const TGeoBBox*>(&shape); box != nullptr) {
      // this should always be true
      ACTS_VERBOSE("Have Box");
      double dx = box->GetDX();
      double dy = box->GetDY();
      double dz = box->GetDZ();
      ACTS_VERBOSE(dx << " " << dy << " " << dz);
      if (axes == "xyz") {
        bounds    = std::make_shared<Acts::RectangleBounds>(dx, dy);
        thickness = dz;
      } else if (axes == "yzx") {
        bounds    = std::make_shared<Acts::RectangleBounds>(dy, dz);
        thickness = dx;
      } else {
        throw std::runtime_error{"Axes " + axes + " not yet implemented"};
      }
      return {bounds, thickness};

    } else {
      throw std::runtime_error{"Unable to convert TGeoShape"};
    }
  }

  // std::shared_ptr<Acts::DD4hepDetectorElement>
  std::shared_ptr<Acts::Surface> createDetectorElement(const dd4hep::rec::Surface& srf) {
    auto                _logger = Acts::getDefaultLogger("DDRec2Acts", Acts::Logging::VERBOSE);
    Acts::LoggerWrapper logger{*_logger};

    ACTS_VERBOSE("Converting dd4hep::rec::Surface: checking SurfaceType");

    auto getAxes = [&]() -> std::pair<Acts::Transform3, std::string> {
      ACTS_VERBOSE("Interrogating surface for local coordinate system");

      auto          _origin = srf.globalToLocal(srf.origin());
      Acts::Vector2 origin{_origin.u(), _origin.v()};
      ACTS_VERBOSE(" - origin: " << origin.transpose() << " global " << srf.origin());

      auto          _normal = srf.normal(srf.origin());
      Acts::Vector3 normal{_normal.x(), _normal.y(), _normal.z()};

      // ACTS_VERBOSE(" -> u.v = " << localU.dot(localV));

      const TGeoNode&   placement    = *srf.detElement().placement().ptr();
      const TGeoMatrix& geoTransform = srf.detElement().nominal().worldTransformation();

      Acts::Transform3 transform;
      transform.translation() << geoTransform.GetTranslation()[0], geoTransform.GetTranslation()[1],
          geoTransform.GetTranslation()[2];

      const auto* rot = geoTransform.GetRotationMatrix();

      std::array<Acts::Vector3, 3> columns{Acts::Vector3{rot[0], rot[3], rot[6]}, Acts::Vector3{rot[1], rot[4], rot[7]},
                                           Acts::Vector3{rot[2], rot[5], rot[8]}};

      std::array<char, 3> axes{'x', 'y', 'z'};

      auto rotate = [](auto& a) { std::rotate(std::rbegin(a), std::rbegin(a) + 1, std::rend(a)); };

      ACTS_VERBOSE("trafo: \n" << transform.matrix());
      ACTS_VERBOSE("origin loc: " << (transform * Acts::Vector3::Zero()).transpose());

      bool found = false;

      for (size_t i = 0; i < 3; i++) {
        transform.matrix().block<3, 1>(0, 0) = columns[0];
        transform.matrix().block<3, 1>(0, 1) = columns[1];
        transform.matrix().block<3, 1>(0, 2) = columns[2];

        Acts::Vector3 normalLoc = transform.inverse().linear() * normal;
        ACTS_VERBOSE("Axes: " << axes[0] << axes[1] << axes[2]);
        ACTS_VERBOSE("normal global: " << normal.transpose());
        ACTS_VERBOSE("    ->  local: " << normalLoc.transpose());

        // ACTS_VERBOSE(normalLoc.dot(Acts::Vector3::UnitZ()) - 1);

        if (std::abs(normalLoc.dot(Acts::Vector3::UnitZ()) - 1) < Acts::s_epsilon) {
          ACTS_VERBOSE("Normal accepted");
          found = true;
          break;
        }

        rotate(axes);
        rotate(columns);
      }

      if (!found) {
        ACTS_ERROR("Unable to find permutation of axis for correct surface normal");
        throw std::runtime_error("Unable to find permutation of axis for correct surface normal");
      }

      auto          _globalU = srf.u(srf.origin());
      Acts::Vector3 globalU{_globalU.x(), _globalU.y(), _globalU.z()};
      Acts::Vector3 localU = transform.inverse().linear() * globalU;
      ACTS_VERBOSE(" - u: global " << globalU.transpose() << " local " << localU.transpose());
      auto          _globalV = srf.v(srf.origin());
      Acts::Vector3 globalV{_globalV.x(), _globalV.y(), _globalV.z()};
      Acts::Vector3 localV = transform.inverse().linear() * globalV;
      ACTS_VERBOSE(" - v: global " << globalV.transpose() << " local " << localV.transpose());

      double localRotAngle = -std::acos(localU.normalized().dot(Acts::Vector3::UnitX()));
      ACTS_VERBOSE("Local rot angle: " << (localRotAngle));

      transform = transform * Acts::AngleAxis3{localRotAngle, Acts::Vector3::UnitZ()};

      std::string saxes{axes.data(), 3};
      // ACTS_VERBOSE(saxes);

      return {transform, saxes};

      // sanity check

      // ACTS_VERBOSE((transform.linear() * Acts::Vector3::UnitZ()).transpose());
    };

    if (srf.type().isCylinder()) {
      ACTS_VERBOSE("Surface is Cylinder");
    } else if (srf.type().isPlane()) {
      ACTS_VERBOSE("Is 'Plane' (might be disc)");

      auto [transform, axes] = getAxes();
      auto [bounds, thickness] =
          convertShapePlane(*srf.detElement().placement().ptr()->GetVolume()->GetShape(), axes, logger);

      std::stringstream ss;
      bounds->toStream(ss);
      ACTS_VERBOSE("Created bounds: " << ss.str());
      ACTS_VERBOSE("Thickness: " << thickness);

      auto surface = Acts::Surface::makeShared<Acts::PlaneSurface>(transform, bounds);
      return surface;

    } else {
      ACTS_ERROR("Surface type " << srf.type() << " currently unsupported");
      throw std::runtime_error("Unsupported surface type");
    }

    return nullptr;
    // return std::make_shared<Acts::DD4hepDetectorElement>(srf.detElement(), "Xz");
  }

  static long addActsExtensions(dd4hep::Detector& description, int, char**) {
    const std::string LOG_SOURCE("AddActsExtensions");
    printout(PrintLevel::INFO, LOG_SOURCE, "Running plugin");

    //Getting the surface manager
    dd4hep::rec::SurfaceManager& surfMan = *description.extension<dd4hep::rec::SurfaceManager>();

    std::map<std::tuple<int, int>, std::vector<std::shared_ptr<Acts::Surface>>> layers{};

    dd4hep::rec::SurfaceHelper      ds(description.world());
    dd4hep::rec::SurfaceList const& detSL   = ds.surfaceList();
    int                             counter = 0;
    for (dd4hep::rec::ISurface* surf : detSL) {
      dd4hep::Volume volume     = ((dd4hep::rec::Surface*)surf)->volume();
      auto*          ddsurf     = ((dd4hep::rec::Surface*)surf);
      auto const     volumeName = std::string(volume->GetName());
      if (not ddsurf->detElement().isValid())
        continue;

      dd4hep::BitFieldCoder* cellIDcoder = nullptr;

      // std::cout << ddsurf->type() << std::endl;
      // printout(PrintLevel::INFO, LOG_SOURCE, "%u", ddsurf->type());
      if (ddsurf->type().isSensitive() and not ddsurf->type().isHelper()) {
        // surface is sensitive, assuming it has a readout
        auto detectorName = ddsurf->detElement().type();
        try {
          auto theDetector = dd4hep::SensitiveDetector(description.sensitiveDetector(detectorName));
          cellIDcoder      = theDetector.readout().idSpec().decoder();
          // printout(PrintLevel::DEBUG, LOG_SOURCE, "The encoding string for detector %s is %s", detectorName.c_str(),
          // cellIDcoder->fieldDescription().c_str());
        } catch (...) {
          printout(PrintLevel::INFO, LOG_SOURCE, "Not readout for detector %s", detectorName.c_str());
          continue;
        }
      } else {
        printout(PrintLevel::INFO, LOG_SOURCE, "Not sensitive");
        continue;
      }
      counter++;

      int system = cellIDcoder->get(ddsurf->id(), "system");
      // if (system != 0) {
      // continue;
      // }

      std::string path        = ddsurf->detElement().path();
      auto        layerNumber = cellIDcoder->get(ddsurf->id(), "layer");
      printout(PrintLevel::INFO, LOG_SOURCE, "Found Surface at %s in layer %d at radius %3.2f mm", path.c_str(),
               layerNumber, ddsurf->origin().rho());

      for (const auto& field : cellIDcoder->fields()) {
        // std::cout << "field: " << field.name() << std::endl;
        std::cout << field.name() << ": " << cellIDcoder->get(ddsurf->id(), field.name()) << std::endl;
      }
      std::cout << "---" << std::endl;

      std::shared_ptr<Acts::Surface> surface = createDetectorElement(*ddsurf);

      std::tuple key{
          system, cellIDcoder->get(ddsurf->id(), "layer"),
          // cellIDcoder->get(ddsurf->id(), "barrel"),
      };

      layers[key].push_back(surface);

      // break;

      //fixme: replace this with the ACTS type to be used as an extension
      dd4hep::rec::DoubleParameters* para = nullptr;
      try {  // use existing map, or create a new one
        para                                    = ddsurf->detElement().extension<dd4hep::rec::DoubleParameters>();
        para->doubleParameters["SortingPolicy"] = 123;
      } catch (...) {
        para                                    = new dd4hep::rec::DoubleParameters;
        para->doubleParameters["SortingPolicy"] = 321;
        ddsurf->detElement().addExtension<dd4hep::rec::DoubleParameters>(para);
      }
      // if (counter > 1000) {
      // break;
      // }
    }  // for all surfaces

    Acts::GeometryContext gctx;
    for (const auto& [key, elements] : layers) {
      Acts::ObjVisualization3D vis;
      const auto [system, layer] = key;
      for (const auto& element : elements) {
        Acts::GeometryView3D::drawSurface(vis, *element, gctx);
      }
      std::stringstream ss;
      ss << "obj/" << system << "_" << layer << ""
         << ".obj";
      std::ofstream ofs{ss.str()};
      vis.write(ofs);
    }

    return 42;
  }

}  // namespace

DECLARE_APPLY(k4acts_addActsExtensions, ::addActsExtensions)
