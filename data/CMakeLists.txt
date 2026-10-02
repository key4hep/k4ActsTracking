#[[
Copyright (c) 2014-2024 Key4hep-Project.

This file is part of Key4hep.
See https://key4hep.github.io/key4hep-doc/ for further info.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
]]

include(ExternalData)

set(ExternalData_URL_TEMPLATES
  "https://key4hep.web.cern.ch/key4hep/k4ActsTracking/MuColl/data/%(hash)"
)
# Copy the files into the build tree instead of symlinking to the object store,
# so that install() installs the files themselves
set(ExternalData_NO_SYMLINKS ON)

set(data_files
  # Tracking geometries
  MuColl_v1.root MuSIC_v1.root MuSIC_v2.root MAIA_v0.root
  # Geometry descriptions
  MuColl_v1.json MuSIC_v1.json MuSIC_v2.json MAIA_v0.json
  # Material maps
  material-maps.json MAIA_v0_material.json MAIA_v0_gen3_material_map.json
)

set(data_args)
foreach(file IN LISTS data_files)
  list(APPEND data_args "DATA{${file}}")
endforeach()

ExternalData_Expand_Arguments(k4ActsTrackingData data_paths ${data_args})
ExternalData_Add_Target(k4ActsTrackingData)

install(FILES ${data_paths}
  DESTINATION ${CMAKE_INSTALL_DATADIR}/k4ActsTracking/data)
