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
#ifndef K4ACTSTRACKING_CELLIDSELECTOR_H
#define K4ACTSTRACKING_CELLIDSELECTOR_H

#include <DD4hep/BitFieldCoder.h>
#include <Parsers/Primitives.h>

#include <string>
#include <vector>

namespace k4ActsTracking {

  class CellIDSelector {
  public:
    struct Selector {
      dd4hep::CellID mask;
      dd4hep::CellID value;
    };

    CellIDSelector(const std::string& encodingString, const std::vector<std::string>& selections);

    bool accept(const dd4hep::CellID cellID) const;

    std::vector<Selector> getSelectionMasks(const std::string& selection) const;

  private:
    dd4hep::BitFieldCoder m_decoder{};
    std::vector<Selector> m_selectors{};
  };
}  // namespace k4ActsTracking

#endif  //  K4ACTSTRACKING_CELLIDSELECTOR_H
