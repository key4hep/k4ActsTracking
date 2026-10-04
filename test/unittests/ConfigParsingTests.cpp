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
#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers_vector.hpp"

#include "ConfigParsing.h"

#include <string>
#include <vector>

TEST_CASE("parseList") {
  SECTION("strings") {
    REQUIRE_THAT(gnntracking::parseList<std::string>("r,phi,z,t"),
                 Catch::Matchers::Equals(std::vector<std::string>{"r", "phi", "z", "t"}));
  }

  SECTION("surrounding whitespace is trimmed") {
    REQUIRE_THAT(gnntracking::parseList<std::string>(" r , phi ,z "),
                 Catch::Matchers::Equals(std::vector<std::string>{"r", "phi", "z"}));
  }

  SECTION("empty elements are skipped") {
    REQUIRE_THAT(gnntracking::parseList<std::string>("r,,phi,"),
                 Catch::Matchers::Equals(std::vector<std::string>{"r", "phi"}));
    REQUIRE(gnntracking::parseList<std::string>("").empty());
    REQUIRE(gnntracking::parseList<float>("  ").empty());
  }

  SECTION("numeric values") {
    REQUIRE_THAT(gnntracking::parseList<float>("1, 2.5,1000"),
                 Catch::Matchers::Equals(std::vector<float>{1.0f, 2.5f, 1000.0f}));
    REQUIRE_THAT(gnntracking::parseList<int>("1,-2,3"), Catch::Matchers::Equals(std::vector<int>{1, -2, 3}));
  }
}

TEST_CASE("parseMultiList") {
  SECTION("one list per entry") {
    const auto parsed = gnntracking::parseMultiList<std::string>({"r,phi", "x,y,z"});
    REQUIRE(parsed.size() == 2);
    REQUIRE_THAT(parsed[0], Catch::Matchers::Equals(std::vector<std::string>{"r", "phi"}));
    REQUIRE_THAT(parsed[1], Catch::Matchers::Equals(std::vector<std::string>{"x", "y", "z"}));
  }

  SECTION("empty entries yield empty lists") {
    const auto parsed = gnntracking::parseMultiList<float>({"1,2", ""});
    REQUIRE(parsed.size() == 2);
    REQUIRE_THAT(parsed[0], Catch::Matchers::Equals(std::vector<float>{1.0f, 2.0f}));
    REQUIRE(parsed[1].empty());
  }

  SECTION("empty input") { REQUIRE(gnntracking::parseMultiList<std::string>({}).empty()); }
}
