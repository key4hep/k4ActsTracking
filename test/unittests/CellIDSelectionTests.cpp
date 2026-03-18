
#include "k4ActsTracking/CellIDSelector.h"

#include <DD4hep/BitFieldCoder.h>
#include <Parsers/Primitives.h>

#include <catch2/matchers/catch_matchers.hpp>
#include "catch2/catch_test_macros.hpp"
#include "catch2/generators/catch_generators.hpp"
#include "catch2/generators/catch_generators_adapters.hpp"
#include "catch2/generators/catch_generators_random.hpp"
#include "catch2/matchers/catch_matchers_range_equals.hpp"

#include <limits>
#include <ranges>

using namespace k4ActsTracking;

TEST_CASE("CellIDSelector::accept single selection") {
  const std::string encodingString = "system:8,side:-2,layer:5,module:7,sensor:10";
  const auto        encoder        = dd4hep::BitFieldCoder(encodingString);
  const auto        selector       = CellIDSelector{encodingString, {"system:8,layer:3"}};

  dd4hep::CellID cellID{0};
  encoder.set(cellID, "system", 8);
  encoder.set(cellID, "layer", 3);
  encoder.set(cellID, "side", 0);
  encoder.set(cellID, "module", 5);
  encoder.set(cellID, "sensor", 42);

  REQUIRE(selector.accept(cellID));

  encoder.set(cellID, "sensor", 123);
  encoder.set(cellID, "module", 12);
  REQUIRE(selector.accept(cellID));

  encoder.set(cellID, "system", 5);
  REQUIRE_FALSE(selector.accept(cellID));

  encoder.set(cellID, "system", 8);
  encoder.set(cellID, "layer", 16);
  REQUIRE_FALSE(selector.accept(cellID));
}

TEST_CASE("CellIDSelector::accept multiple selections") {
  const std::string encodingString = "system:8,side:-2,layer:5,module:7,sensor:10";
  const auto        encoder        = dd4hep::BitFieldCoder(encodingString);

  const auto selector = CellIDSelector{encodingString, {"system:5,layer:1|4|5", "system:3,layer:2|6|8", "sensor:42"}};

  dd4hep::CellID cellID{0};
  encoder.set(cellID, "side", -1);
  encoder.set(cellID, "module", 5);

  encoder.set(cellID, "system", 8);
  encoder.set(cellID, "layer", 4);
  // Layer would match but system doesn't
  REQUIRE_FALSE(selector.accept(cellID));

  encoder.set(cellID, "sensor", 42);
  encoder.set(cellID, "layer", 3);
  // Nothing matches except sensor:42 from the last selection
  REQUIRE(selector.accept(cellID));

  encoder.set(cellID, "sensor", 123);
  encoder.set(cellID, "system", 5);
  // system:5 would match, but layers are off
  REQUIRE_FALSE(selector.accept(cellID));
  encoder.set(cellID, "layer", 5);
  REQUIRE(selector.accept(cellID));

  encoder.set(cellID, "system", 3);
  REQUIRE_FALSE(selector.accept(cellID));
  encoder.set(cellID, "layer", 8);
  REQUIRE(selector.accept(cellID));
}

TEST_CASE("CellIDSelector::accept empty selection") {
  const auto selector = CellIDSelector{"system:8,side:-2,layer:5,module:7,sensor:10", {}};
  const auto cellID   = GENERATE(take(100, random(0UL, std::numeric_limits<dd4hep::CellID>::max())));
  REQUIRE_FALSE(selector.accept(cellID));
}

TEST_CASE("CellIDSelector::getSelectionMasks") {
  const std::string encodingString = "system:8,side:-2,layer:5,module:7,sensor:10";
  const auto        selector       = CellIDSelector{encodingString, {}};

  // Selectors that select everything should not get a mask
  REQUIRE(selector.getSelectionMasks("system:*").empty());
  REQUIRE(selector.getSelectionMasks("layer:*,system:*").empty());

  constexpr dd4hep::CellID systemMask = (0x0001ULL << 8) - 1;
  constexpr dd4hep::CellID layerMask  = ((0x0001ULL << 5) - 1) << (8 + 2);
  constexpr dd4hep::CellID sensorMask = ((0x0001ULL << 10) - 1) << (8 + 2 + 5 + 7);

  constexpr auto allValues = [](const auto& sel) {
    namespace rv = std::ranges::views;
    return rv::transform(sel, [](const auto& elem) { return elem.value; });
  };

  auto sel = selector.getSelectionMasks("system:8");
  REQUIRE(sel.size() == 1);
  REQUIRE(sel[0].mask == systemMask);
  REQUIRE(sel[0].value == 8);

  sel = selector.getSelectionMasks("layer:2");
  REQUIRE(sel.size() == 1);
  REQUIRE(sel[0].mask == layerMask);
  REQUIRE(sel[0].value == ((2 << (8 + 2))));

  sel = selector.getSelectionMasks("system:3,layer:*");
  REQUIRE(sel.size() == 1);
  REQUIRE(sel[0].mask == systemMask);
  REQUIRE(sel[0].value == 3);

  sel = selector.getSelectionMasks("system:4,layer:3");
  REQUIRE(sel.size() == 1);
  REQUIRE(sel[0].mask == (systemMask | layerMask));
  REQUIRE(sel[0].value == (4 + (3 << (8 + 2))));

  using Catch::Matchers::UnorderedRangeEquals;

  sel = selector.getSelectionMasks("system:3|5");
  REQUIRE(sel.size() == 2);
  for (const auto s : sel) {
    REQUIRE(s.mask == systemMask);
  }
  REQUIRE_THAT(allValues(sel), UnorderedRangeEquals(std::vector{3, 5}));

  sel = selector.getSelectionMasks("system:3|5,layer:1|12|10");
  REQUIRE(sel.size() == 6);
  for (const auto s : sel) {
    REQUIRE(s.mask == (systemMask | layerMask));
  }
  REQUIRE_THAT(allValues(sel), UnorderedRangeEquals(std::vector{
                                   3 + (1 << (8 + 2)),
                                   5 + (1 << (8 + 2)),
                                   3 + (10 << (8 + 2)),
                                   5 + (10 << (8 + 2)),
                                   3 + (12 << (8 + 2)),
                                   5 + (12 << (8 + 2)),
                               }));

  sel = selector.getSelectionMasks("system:*,layer:1|3|5,sensor:3");
  REQUIRE(sel.size() == 3);
  for (const auto s : sel) {
    REQUIRE(s.mask == (layerMask | sensorMask));
  }
  REQUIRE_THAT(allValues(sel), UnorderedRangeEquals(std::vector{
                                   (1 << (8 + 2)) + (3 << (8 + 2 + 5 + 7)),
                                   (3 << (8 + 2)) + (3 << (8 + 2 + 5 + 7)),
                                   (5 << (8 + 2)) + (3 << (8 + 2 + 5 + 7)),
                               }));
}
