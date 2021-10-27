#include <DD4hep/DetElement.h>
#include <DD4hep/Detector.h>
#include <DD4hep/Factories.h>
#include <DD4hep/Printout.h>

#include <DDRec/DetectorData.h>
#include <DDRec/SurfaceHelper.h>

#include <string>

using dd4hep::DetElement;
using dd4hep::PrintLevel;

namespace {

  /** Plugin for adding the ACTS Extensions to surface, or DetElements, or something
   *
   */

  static long addActsExtensions(dd4hep::Detector& description, int argc, char** argv) {
    const std::string LOG_SOURCE("AddActsExtensions");
    printout(PrintLevel::INFO, LOG_SOURCE, "Running extension");

    dd4hep::rec::SurfaceHelper      ds(description.world());
    dd4hep::rec::SurfaceList const& detSL = ds.surfaceList();

    for (dd4hep::rec::ISurface* surf : detSL) {
      dd4hep::Volume volume     = ((dd4hep::rec::Surface*)surf)->volume();
      auto*          ddsurf     = ((dd4hep::rec::Surface*)surf);
      auto const     volumeName = std::string(volume->GetName());
      if (not ddsurf->detElement().isValid())
        continue;
      std::string path = ddsurf->detElement().path();

      //FIXME: replace this with the ACTS type to be used as an extension
      dd4hep::rec::DoubleParameters* para = nullptr;
      try {  // use existing map, or create a new one
        para                                    = ddsurf->detElement().extension<dd4hep::rec::DoubleParameters>();
        para->doubleParameters["SortingPolicy"] = 123;
      } catch (...) {
        para                                    = new dd4hep::rec::DoubleParameters;
        para->doubleParameters["SortingPolicy"] = 321;
        ddsurf->detElement().addExtension<dd4hep::rec::DoubleParameters>(para);
      }
      printout(PrintLevel::INFO, LOG_SOURCE,
               "Added extension to %s"
               " path %s, type %s, value %3.5f",
               volumeName.c_str(), ddsurf->detElement().path().c_str(), ddsurf->detElement().type().c_str(),
               para->doubleParameters["SortingPolicy"]);
    }  // for all surfaces

    return 0;  // this return value isn't really returned from the plugin service
  }

}  // namespace

DECLARE_APPLY(k4acts_addActsExtensions, ::addActsExtensions)
