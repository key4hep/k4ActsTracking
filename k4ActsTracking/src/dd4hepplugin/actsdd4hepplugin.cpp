#include <DD4hep/DetElement.h>
#include <DD4hep/Detector.h>
#include <DD4hep/Factories.h>
#include <DD4hep/Printout.h>

#include <DD4hep/IDDescriptor.h>

#include <DDRec/DetectorData.h>
#include <DDRec/SurfaceHelper.h>
#include <DDRec/SurfaceManager.h>

#include <string>

using dd4hep::DetElement;
using dd4hep::PrintLevel;

namespace {

  /** Plugin for adding the ACTS Extensions to surface, or DetElements, or something
   *
   */

  static long addActsExtensions(dd4hep::Detector& description, int, char**) {
    const std::string LOG_SOURCE("AddActsExtensions");
    printout(PrintLevel::INFO, LOG_SOURCE, "Running plugin");

    //Getting the surface manager
    dd4hep::rec::SurfaceManager& surfMan = *description.extension<dd4hep::rec::SurfaceManager>();

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
      if (ddsurf->type().isSensitive() and not ddsurf->type().isHelper()) {
        // surface is sensitive, assuming it has a readout
        auto detectorName = ddsurf->detElement().type();
        try {
          auto theDetector = dd4hep::SensitiveDetector(description.sensitiveDetector(detectorName));
          cellIDcoder      = theDetector.readout().idSpec().decoder();
          printout(PrintLevel::DEBUG, LOG_SOURCE, "The encoding string for detector %s is %s", detectorName.c_str(),
                   cellIDcoder->fieldDescription().c_str());
        } catch (...) {
          printout(PrintLevel::INFO, LOG_SOURCE, "Not readout for detector %s", detectorName.c_str());
          continue;
        }
      } else {
        continue;
      }
      counter++;

      std::string path        = ddsurf->detElement().path();
      auto        layerNumber = cellIDcoder->get(ddsurf->id(), "layer");
      printout(PrintLevel::INFO, LOG_SOURCE, "Found Surface at %s in layer %d at radius %3.2f mm", path.c_str(),
               layerNumber, ddsurf->origin().rho());

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
      if (counter > 1000) {
        break;
      }
    }  // for all surfaces

    return 42;
  }

}  // namespace

DECLARE_APPLY(k4acts_addActsExtensions, ::addActsExtensions)
