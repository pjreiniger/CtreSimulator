
#pragma once

#include <stdint.h>

#include "CtreSimUtils/MockHookUtilities.h"

namespace SnobotSim
{

typedef void (*CTRE_CallbackFunc)(const char* name, uint32_t messageId,
        uint8_t* buffer, int size);

void EXPORT_ SetMotControllerCallback(CTRE_CallbackFunc callback);
void EXPORT_ SetPigeonCallback(CTRE_CallbackFunc callback);
void EXPORT_ SetCanifierCallback(CTRE_CallbackFunc callback);
void EXPORT_ SetBuffTrajPiontStreamCallback(CTRE_CallbackFunc callback);

} // namespace SnobotSim