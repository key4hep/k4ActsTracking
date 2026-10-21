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
#pragma once

#include <Acts/Utilities/Logger.hpp>
#include <ActsPlugins/Gnn/GnnPipeline.hpp>
#include <ActsPlugins/Gnn/Stages.hpp>

#include <cstddef>
#include <string>
#include <vector>

/// Logs the graph after every stage of a GnnPipeline run: the edges, their
/// score once a classifier has produced one, and their edge features if the
/// pipeline computes them.
///
/// GnnPipeline::run() calls the hook once after the graph construction and once
/// after every edge classification stage, so the output shows what each stage
/// made of the graph. The hook counts its calls to know which stage it is
/// looking at, so construct a new one for every run.
class EdgePrintingHook final : public ActsPlugins::GnnHook {
public:
  struct Config {
    /// Names of the pipeline stages in the order the hook is called for them:
    /// the graph construction first, then every edge classification stage.
    /// Calls beyond the end are labelled by their index.
    std::vector<std::string> stageNames{};
    /// How many edges to print per stage. Ignored if printAll is set.
    std::size_t numEdgesShown{5};
    /// Print every edge instead of the first numEdgesShown ones. For a real
    /// event this is a lot of output.
    bool printAll{false};
  };

  /// @param logger has to outlive the hook. Nothing is printed (or copied off
  ///        the device) unless it prints DEBUG.
  EdgePrintingHook(Config cfg, const Acts::Logger& logger);

  void operator()(const ActsPlugins::PipelineTensors& tensors,
                  const ActsPlugins::ExecutionContext& execContext) const override;

private:
  Config m_cfg;
  const Acts::Logger& m_logger;
  /// GnnHook::operator() is const, but which stage comes next is state
  mutable std::size_t m_numCalls{0};

  const Acts::Logger& logger() const { return m_logger; }
};
