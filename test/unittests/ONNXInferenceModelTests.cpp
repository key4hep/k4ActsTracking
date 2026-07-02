#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers_vector.hpp"

#include "ONNXInferenceModel.h"

#include <vector>

TEST_CASE("totalSize") {
  REQUIRE(mlutils::totalSize(42) == 1);
  REQUIRE(mlutils::totalSize(std::vector{1.23f, 2.34f}) == 2);
  REQUIRE(mlutils::totalSize(std::vector<int>{}) == 0);
  REQUIRE(mlutils::totalSize(std::vector<std::vector<int>>{{1, 2}, {3, 4, 5}}) == 5);
  REQUIRE(mlutils::totalSize(
              std::vector<std::vector<std::vector<float>>>{{{1.0f, 2.0f}, {3.0f}}, {{4.0f, 5.0f, 6.0f}}}) == 6);
}

TEST_CASE("flatten") {
  SECTION("scalar values") {
    auto output = mlutils::flatten(42);
    REQUIRE_THAT(output, Catch::Matchers::Equals(std::vector<int>{42}));
  }

  SECTION("1D vector") {
    std::vector<float> input{1.5f, 2.5f, 3.5f};
    auto output = mlutils::flatten(input);
    REQUIRE_THAT(output, Catch::Matchers::Equals(std::vector<float>{1.5f, 2.5f, 3.5f}));
  }

  SECTION("empty vector") {
    std::vector<int> input{};
    auto output = mlutils::flatten(input);
    REQUIRE(output.empty());
  }

  SECTION("2D vector") {
    std::vector<std::vector<int>> input{{1, 2}, {3, 4, 5}};
    auto output = mlutils::flatten(input);
    REQUIRE_THAT(output, Catch::Matchers::Equals(std::vector<int>{1, 2, 3, 4, 5}));
  }

  SECTION("3D vector") {
    std::vector<std::vector<std::vector<float>>> input{{{1.0f, 2.0f}, {3.0f}}, {{4.0f, 5.0f, 6.0f}}};
    auto output = mlutils::flatten(input);
    REQUIRE_THAT(output, Catch::Matchers::Equals(std::vector<float>{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f}));
  }
}

TEST_CASE("getDimensions") {
  SECTION("scalar values") {
    auto dims = mlutils::getDimensions(42);
    REQUIRE_THAT(dims, Catch::Matchers::Equals(std::vector<int64_t>{}));
  }

  SECTION("1D vector") {
    std::vector<float> input{1.5f, 2.5f, 3.5f};
    auto dims = mlutils::getDimensions(input);
    REQUIRE_THAT(dims, Catch::Matchers::Equals(std::vector<int64_t>{3}));
  }

  SECTION("empty vector") {
    std::vector<int> input{};
    auto dims = mlutils::getDimensions(input);
    REQUIRE_THAT(dims, Catch::Matchers::Equals(std::vector<int64_t>{0}));
  }

  SECTION("2D vector - rectangular") {
    std::vector<std::vector<int>> input{{1, 2, 3}, {4, 5, 6}};
    auto dims = mlutils::getDimensions(input);
    REQUIRE_THAT(dims, Catch::Matchers::Equals(std::vector<int64_t>{2, 3}));
  }

  SECTION("2D vector - jagged") {
    std::vector<std::vector<int>> input{{1, 2}, {3, 4, 5}};
    auto dims = mlutils::getDimensions(input);
    // NOTE here the "unexpected expected" result, due to the lack of validation
    REQUIRE_THAT(dims, Catch::Matchers::Equals(std::vector<int64_t>{2, 2}));
  }

  SECTION("3D vector") {
    std::vector<std::vector<std::vector<float>>> input{{{1.0f, 2.0f}, {3.0f, 4.0f}}, {{5.0f, 6.0f}, {7.0f, 8.0f}}};
    auto dims = mlutils::getDimensions(input);
    REQUIRE_THAT(dims, Catch::Matchers::Equals(std::vector<int64_t>{2, 2, 2}));
  }
}
