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
#ifndef K4ACTSTRACKING_ACTSGAUDILOGGER_H
#define K4ACTSTRACKING_ACTSGAUDILOGGER_H

#include <Acts/Utilities/Logger.hpp>

#include <GaudiKernel/CommonMessaging.h>
#include <GaudiKernel/IMessageSvc.h>
#include <GaudiKernel/MsgStream.h>

#include <memory>
#include <optional>
#include <string>

/// Print policy that forwards Acts logging calls to the Gaudi MsgStream
class ActsGaudiPrintPolicy final : public Acts::Logging::OutputPrintPolicy {
public:
  ActsGaudiPrintPolicy(IMessageSvc* svc, std::shared_ptr<MsgStream> msg, const std::string& name)
      : m_svc(svc), m_msg(msg), m_name(name) {}

  void flush(const Acts::Logging::Level& lvl, const std::string& input) override;

  const std::string& name() const override { return m_name; }

  std::unique_ptr<Acts::Logging::OutputPrintPolicy> clone(const std::string& name) const override;

private:
  IMessageSvc*               m_svc{nullptr};
  std::shared_ptr<MsgStream> m_msg{nullptr};
  std::string                m_name{};
};

/// Filter policy that maps the Gaudi log levels to the Acts log levels
class ActsGaudiFilterPolicy final : public Acts::Logging::OutputFilterPolicy {
public:
  ActsGaudiFilterPolicy(std::shared_ptr<MsgStream> msg) : m_msg(msg) {}

  bool doPrint(const Acts::Logging::Level& lvl) const override;

  Acts::Logging::Level level() const override;

  std::unique_ptr<Acts::Logging::OutputFilterPolicy> clone(Acts::Logging::Level lvl) const override;

private:
  std::shared_ptr<MsgStream> m_msg{nullptr};
};

/// Create an Acts logger wrapping the Gaudi facilities for use in Acts
/// components used within the Gaudi framework
std::unique_ptr<const Acts::Logger> makeActsGaudiLogger(const CommonMessagingBase* parent,
                                                        std::optional<std::string> name = std::nullopt);

/// Create an Acts logger wrapping the Gaudi facilities for use in Acts
/// components used within the Gaudi framework
std::unique_ptr<const Acts::Logger> makeActsGaudiLogger(IMessageSvc* svc, const std::string& name, int level,
                                                        std::optional<std::string> parentName = std::nullopt);

// Overload for accepting string literals
inline std::unique_ptr<const Acts::Logger> makeActsGaudiLogger(const CommonMessagingBase* parent,
                                                               const std::string&         name) {
  return makeActsGaudiLogger(parent, std::optional<std::string>{name});
}

#endif  // K4ACTSTRACKING_ACTSGAUDILOGGER_H
