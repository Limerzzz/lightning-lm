//
// Created by codex on 2026/2/25.
//

#ifndef LIGHTNING_INS_CONVERTER_H
#define LIGHTNING_INS_CONVERTER_H

#include "bot_msg/msg/localization_info.hpp"

#include "common/ins.h"

namespace lightning {

InsMeasurement ConvertLocalizationInfo(const bot_msg::msg::LocalizationInfo& msg, const InsConfig& cfg);

}  // namespace lightning

#endif  // LIGHTNING_INS_CONVERTER_H
