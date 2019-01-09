/*
 * CtreBuffTrajPointStreamWrapper.cpp
 *
 *  Created on: Jan 9, 2019
 *      Author: PJ
 */

#include "CtreSimMocks/CtreBuffTrajPointStreamWrapper.h"

#include <vector>

#include "CtreSimMocks/MockHooks.h"

std::vector<SnobotSim::CTRE_CallbackFunc> gBuffTrajPointStreamCallbacks;

void SnobotSim::SetBuffTrajPiontStreamCallback(
        SnobotSim::CTRE_CallbackFunc callback)
{
    gBuffTrajPointStreamCallbacks.clear();
    gBuffTrajPointStreamCallbacks.push_back(callback);
}


SnobotSim::CtreBuffTrajPointStreamWrapper::CtreBuffTrajPointStreamWrapper()
{
    Send("Create");
}

void SnobotSim::CtreBuffTrajPointStreamWrapper::Send(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    if (!gBuffTrajPointStreamCallbacks.empty())
    {
        gBuffTrajPointStreamCallbacks[0](aName.c_str(), 0, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtreBuffTrajPointStreamWrapper::Receive(const std::string& aName,
        uint8_t* aBuffer,
        int aSize)
{
    if (!gBuffTrajPointStreamCallbacks.empty())
    {
        gBuffTrajPointStreamCallbacks[0](aName.c_str(), 0, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}
