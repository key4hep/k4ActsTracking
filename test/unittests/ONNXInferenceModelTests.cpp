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
#include "catch2/matchers/catch_matchers_string.hpp"

#include "ONNXInferenceModel.h"

TEST_CASE("model metadata without a loaded model") {
  // The embedding dimension is read from the model metadata, so the accessors
  // have to stay safe before (or after a failed) loadModel.
  mlutils::ONNXInferenceModel model{"UnitTest"};

  REQUIRE(model.numInputs() == 0);
  REQUIRE(model.numOutputs() == 0);
  REQUIRE(model.inputShape(0).empty());
  REQUIRE(model.outputShape(0).empty());

  // The error names the model, which the onnxruntime message alone does not
  REQUIRE_THROWS_WITH(model.loadModel("this-file-does-not-exist.onnx"),
                      Catch::Matchers::ContainsSubstring("this-file-does-not-exist.onnx"));
  REQUIRE(model.numInputs() == 0);
  REQUIRE(model.numOutputs() == 0);
  REQUIRE(model.inputShape(0).empty());
  REQUIRE(model.outputShape(0).empty());
}
