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

#include "k4ActsTracking/ActsGaudiLogger.h"

#include <GaudiKernel/IMessageSvc.h>
#include <GaudiKernel/INamedInterface.h>

#include <array>
#include <memory>
#include <stdexcept>
#include <type_traits>

namespace {
  constexpr MSG::Level getGaudiLevel(Acts::Logging::Level lvl) {
    // All Acts logging levels are available in Gaudi, we can simply map them
    // one-to-one
    constexpr std::array gaudiLevels = {MSG::VERBOSE, MSG::DEBUG, MSG::INFO, MSG::WARNING, MSG::ERROR, MSG::FATAL};

    return gaudiLevels[static_cast<std::underlying_type_t<Acts::Logging::Level>>(lvl)];
  }

  constexpr Acts::Logging::Level getActsLevel(MSG::Level lvl) {
    // MSG::NIL and MSG::ALWAYS are not available in ACTS, so we have to remap
    // them accordingly.
    constexpr std::array actsLevels = {
        Acts::Logging::Level::FATAL,  // MSG::NIL
        Acts::Logging::Level::VERBOSE, Acts::Logging::Level::DEBUG, Acts::Logging::Level::INFO,
        Acts::Logging::Level::WARNING, Acts::Logging::Level::ERROR, Acts::Logging::Level::FATAL,
        Acts::Logging::Level::VERBOSE,  // MSG::ALWAYS
    };
    return actsLevels[static_cast<std::underlying_type_t<MSG::Level>>(lvl)];
  }

  // Very minor testing here
  static_assert(getGaudiLevel(Acts::Logging::Level::DEBUG) == MSG::DEBUG);
  static_assert(getGaudiLevel(Acts::Logging::Level::INFO) == MSG::INFO);

  static_assert(getActsLevel(MSG::NIL) == Acts::Logging::Level::FATAL);
  static_assert(getActsLevel(MSG::INFO) == Acts::Logging::Level::INFO);
  static_assert(getActsLevel(MSG::DEBUG) == Acts::Logging::Level::DEBUG);
  static_assert(getActsLevel(MSG::ALWAYS) == Acts::Logging::Level::VERBOSE);
}  // namespace

void ActsGaudiPrintPolicy::flush(const Acts::Logging::Level& lvl, const std::string& input) {
  const auto msgLevel = getGaudiLevel(lvl);
  (*m_msg) << msgLevel << input << endmsg;
}

std::unique_ptr<Acts::Logging::OutputPrintPolicy> ActsGaudiPrintPolicy::clone(const std::string& name) const {
  auto msg = std::make_shared<MsgStream>(m_svc, name);
  msg->setLevel(m_msg->level());
  return std::make_unique<ActsGaudiPrintPolicy>(m_svc, msg, name);
}

bool ActsGaudiFilterPolicy::doPrint(const Acts::Logging::Level& level) const {
  return m_msg->level() <= getGaudiLevel(level);
}

Acts::Logging::Level ActsGaudiFilterPolicy::level() const { return getActsLevel(m_msg->level()); }

std::unique_ptr<Acts::Logging::OutputFilterPolicy> ActsGaudiFilterPolicy::clone(Acts::Logging::Level lvl) const {
  auto msg = std::make_shared<MsgStream>(*m_msg.get());
  msg->setLevel(getGaudiLevel(lvl));
  return std::make_unique<ActsGaudiFilterPolicy>(msg);
}

std::unique_ptr<const Acts::Logger> makeActsGaudiLogger(IMessageSvc* svc, const std::string& name, int level,
                                                        std::optional<std::string> parentName) {
  auto msg = std::make_shared<MsgStream>(svc, name);
  msg->setLevel(level);
  auto       filter   = std::make_unique<ActsGaudiFilterPolicy>(msg);
  const auto fullName = [&parentName, &name]() {
    if (parentName) {
      return parentName.value() + "." + name;
    }
    return name;
  }();

  auto print = std::make_unique<ActsGaudiPrintPolicy>(svc, msg, fullName);

  return std::make_unique<const Acts::Logger>(std::move(print), std::move(filter));
}

std::unique_ptr<const Acts::Logger> makeActsGaudiLogger(const CommonMessagingBase* parent,
                                                        std::optional<std::string> name) {
  if (const auto* inamed = dynamic_cast<const INamedInterface*>(parent)) {
    return makeActsGaudiLogger(parent->msgSvc(), inamed->name(), parent->msg().level(), name);
  }
  throw std::invalid_argument("parent needs to be an INamedInterface");
}
