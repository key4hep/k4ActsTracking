
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

  constexpr dd4hep::CellID systemMask = (0x0001ULL << 8) - 1;
  constexpr dd4hep::CellID layerMask  = ((0x0001ULL << 5) - 1) << (8 + 2);

  constexpr auto allValues = [](const auto& sel) {
    namespace rv = std::ranges::views;
    return rv::transform(sel, [](const auto& elem) { return elem.value; });
  };

  auto sel = CellIDSelector::getSelectionMasks("system:8", encodingString);
  REQUIRE(sel.size() == 1);
  REQUIRE(sel[0].mask == systemMask);
  REQUIRE(sel[0].value == 8);

  sel = CellIDSelector::getSelectionMasks("layer:2", encodingString);
  REQUIRE(sel.size() == 1);
  REQUIRE(sel[0].mask == layerMask);
  REQUIRE(sel[0].value == ((2 << (8 + 2))));

  sel = CellIDSelector::getSelectionMasks("system:4,layer:3", encodingString);
  REQUIRE(sel.size() == 1);
  REQUIRE(sel[0].mask == (systemMask | layerMask));
  REQUIRE(sel[0].value == (4 + (3 << (8 + 2))));

  using Catch::Matchers::UnorderedRangeEquals;

  sel = CellIDSelector::getSelectionMasks("system:3|5", encodingString);
  REQUIRE(sel.size() == 2);
  for (const auto s : sel) {
    REQUIRE(s.mask == systemMask);
  }
  REQUIRE_THAT(allValues(sel), UnorderedRangeEquals(std::vector{3, 5}));

  sel = CellIDSelector::getSelectionMasks("system:3|5,layer:1|12|10", encodingString);
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
}

TEST_CASE("CellIDSelector failure modes: malformed selection strings") {
  const std::string encodingString = "system:8,side:-2,layer:5,module:7,sensor:10";

  // No colon separator: "field:value" format is required
  REQUIRE_THROWS_AS(CellIDSelector(encodingString, {"system5"}), std::invalid_argument);

  // Multiple colons make the split ambiguous (field name contains ":")
  REQUIRE_THROWS_AS(CellIDSelector(encodingString, {"system:3:5"}), std::invalid_argument);

  // Non-integer value that cannot be parsed
  REQUIRE_THROWS_AS(CellIDSelector(encodingString, {"system:abc"}), std::invalid_argument);

  // Empty value after colon
  REQUIRE_THROWS_AS(CellIDSelector(encodingString, {"system:"}), std::invalid_argument);

  // Empty string as selection is not valid
  REQUIRE_THROWS_AS(CellIDSelector(encodingString, {""}), std::invalid_argument);

  // Pipe-separated list with a non-integer entry
  REQUIRE_THROWS_AS(CellIDSelector(encodingString, {"system:1|abc|3"}), std::invalid_argument);

  // Pipe-separated list with an empty entry
  REQUIRE_THROWS_AS(CellIDSelector(encodingString, {"system:1|"}), std::invalid_argument);

  // Multi-field: valid first field, non-integer value in second
  REQUIRE_THROWS_AS(CellIDSelector(encodingString, {"system:3,layer:abc"}), std::invalid_argument);

  // Multi-field: valid first field, missing colon in second
  REQUIRE_THROWS_AS(CellIDSelector(encodingString, {"system:3,layer3"}), std::invalid_argument);

  // Multi-field: trailing comma produces an empty token for the second field
  REQUIRE_THROWS_AS(CellIDSelector(encodingString, {"system:3,"}), std::invalid_argument);

  // Multi-field: leading comma produces an empty token for the first field
  REQUIRE_THROWS_AS(CellIDSelector(encodingString, {",layer:3"}), std::invalid_argument);
}

TEST_CASE("CellIDSelector failure modes: unknown field name") {
  const std::string encodingString = "system:8,side:-2,layer:5,module:7,sensor:10";

  // Field name not present in the encoding string
  REQUIRE_THROWS(CellIDSelector(encodingString, {"nonexistent:5"}));

  // Unknown field among valid ones in the same selection
  REQUIRE_THROWS(CellIDSelector(encodingString, {"system:3,typo:1"}));
}

TEST_CASE("CellIDSelector failure modes: value overflow throws") {
  // system is 8 bits wide (values 0-255). DD4hep's BitFieldCoder::set is
  // range-checked: specifying 300 throws rather than truncating silently.
  const std::string encodingString = "system:8,side:-2,layer:5,module:7,sensor:10";
  REQUIRE_THROWS(CellIDSelector(encodingString, {"system:300"}));

  // Same for a value in a pipe list that is in range — succeeds.
  REQUIRE_NOTHROW(CellIDSelector(encodingString, {"system:255"}));
  REQUIRE_NOTHROW(CellIDSelector(encodingString, {"system:0|255"}));

  // Out-of-range value inside a pipe list also throws.
  REQUIRE_THROWS(CellIDSelector(encodingString, {"system:1|300"}));
}

TEST_CASE("CellIDSelector failure modes: duplicate field uses last value") {
  // "system:3,system:5" contains system twice. The Cartesian product expansion
  // produces one combination {(system,3),(system,5)} and the second set() call
  // overwrites the first, so the effective selector is system==5 only.
  const std::string encodingString = "system:8,side:-2,layer:5,module:7,sensor:10";
  const auto        encoder        = dd4hep::BitFieldCoder(encodingString);
  const auto        selector       = CellIDSelector{encodingString, {"system:3,system:5"}};

  dd4hep::CellID cellID{0};
  encoder.set(cellID, "system", 5);
  // system:5 matches because the second value wins
  REQUIRE(selector.accept(cellID));

  encoder.set(cellID, "system", 3);
  // system:3 does NOT match even though it appears in the selection string
  REQUIRE_FALSE(selector.accept(cellID));
}
