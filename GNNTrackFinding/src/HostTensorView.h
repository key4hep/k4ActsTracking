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

#include <ActsPlugins/Gnn/Tensor.hpp>

#include <optional>

namespace gnntracking {

/// Read access on the host to a pipeline tensor that may live on the device.
///
/// A tensor that is already on the CPU is read in place, anything else is
/// copied over once (on the stream of @p execContext) and the copy is kept
/// alive for as long as the view is. Neither copyable nor movable, since data()
/// may point into the copy; construct it in place (e.g. std::optional::emplace)
/// where it is optional.
template <typename T>
class HostTensorView {
public:
  HostTensorView(const ActsPlugins::Tensor<T>& tensor, const ActsPlugins::ExecutionContext& execContext) {
    if (!tensor.device().isCpu()) {
      m_copy.emplace(tensor.clone({ActsPlugins::Device::Cpu(), execContext.stream}));
    }
    m_data = m_copy.has_value() ? m_copy->data() : tensor.data();
  }

  HostTensorView(const HostTensorView&) = delete;
  HostTensorView& operator=(const HostTensorView&) = delete;
  HostTensorView(HostTensorView&&) = delete;
  HostTensorView& operator=(HostTensorView&&) = delete;

  /// The tensor's data, in the tensor's (row-major) layout
  const T* data() const { return m_data; }

private:
  std::optional<ActsPlugins::Tensor<T>> m_copy{};
  const T* m_data{nullptr};
};

} // namespace gnntracking
