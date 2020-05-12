
#pragma once

#include <stdint.h>

#include "CtreSimUtils/MockHookUtilities.h"

namespace SnobotSim
{

typedef void (*CTRE_CallbackFunc)(const char* name, uint32_t messageId,
        uint8_t* buffer, int size);

void EXPORT_ SetMotControllerCallback(CTRE_CallbackFunc callback);
void EXPORT_ SetPigeonIMUCallback(CTRE_CallbackFunc callback);
void EXPORT_ SetCANifierCallback(CTRE_CallbackFunc callback);
void EXPORT_ SetCANCoderCallback(CTRE_CallbackFunc callback);
void EXPORT_ SetBuffTrajPointStreamCallback(CTRE_CallbackFunc callback);

} // namespace SnobotSim
