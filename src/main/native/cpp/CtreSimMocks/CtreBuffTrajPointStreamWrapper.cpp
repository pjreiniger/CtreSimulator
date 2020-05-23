

#include "CtreSimMocks/CtreBuffTrajPointStreamWrapper.h"

#include <vector>

#include "CtreSimUtils/MockHooks.h"

#define RECEIVE_HELPER(paramName, size) \
    uint8_t buffer[size]; /* NOLINT */  \
    std::memset(&buffer[0], 0, size);   \
    Receive(paramName, buffer, size);   \
    uint32_t buffer_pos = 0;

std::vector<SnobotSim::CTRE_CallbackFunc> gBuffTrajPointStreamCallbacks;

void SnobotSim::SetBuffTrajPointStreamCallback(
        SnobotSim::CTRE_CallbackFunc callback)
{
    gBuffTrajPointStreamCallbacks.clear();
    gBuffTrajPointStreamCallbacks.push_back(callback);
}

SnobotSim::CtreBuffTrajPointStreamWrapper::CtreBuffTrajPointStreamWrapper(mDeviceId(0),)
        m_simDevice(std::string("CtreBuffTrajPointStreamWrapper " + std::to_string(aDeviceId)).c_str(), aDeviceId)
{


    m_Lookup_outObject = m_simDevice.CreateDouble("Lookup_outObject", false, 0);
    m_Write_arbFeedFwd = m_simDevice.CreateDouble("Write_arbFeedFwd", false, 0);
    m_Write_auxiliaryArbFeedFwd = m_simDevice.CreateDouble("Write_auxiliaryArbFeedFwd", false, 0);
    m_Write_auxiliaryPos = m_simDevice.CreateDouble("Write_auxiliaryPos", false, 0);
    m_Write_auxiliaryVel = m_simDevice.CreateDouble("Write_auxiliaryVel", false, 0);
    m_Write_isLastPoint = m_simDevice.CreateDouble("Write_isLastPoint", false, 0);
    m_Write_position = m_simDevice.CreateDouble("Write_position", false, 0);
    m_Write_profileSlotSelect0 = m_simDevice.CreateDouble("Write_profileSlotSelect0", false, 0);
    m_Write_profileSlotSelect1 = m_simDevice.CreateDouble("Write_profileSlotSelect1", false, 0);
    m_Write_timeDur = m_simDevice.CreateDouble("Write_timeDur", false, 0);
    m_Write_useAuxPID = m_simDevice.CreateDouble("Write_useAuxPID", false, 0);
    m_Write_velocity = m_simDevice.CreateDouble("Write_velocity", false, 0);
    m_Write_zeroPos = m_simDevice.CreateDouble("Write_zeroPos", false, 0);




    Send("Create");
}

void SnobotSim::CtreBuffTrajPointStreamWrapper::Send(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    if (!gBuffTrajPointStreamCallbacks.empty())
    {
        gBuffTrajPointStreamCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
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
        gBuffTrajPointStreamCallbacks[0](aName.c_str(), mDeviceId, aBuffer, aSize);
    }
    else
    {
        LOG_UNSUPPORTED_CAN_FUNC("Callback " << aName << " not registered");
    }
}

void SnobotSim::CtreBuffTrajPointStreamWrapper::Clear()
{

    Send("Clear");
}

void SnobotSim::CtreBuffTrajPointStreamWrapper::Write(double position, double velocity, double arbFeedFwd, double auxiliaryPos, double auxiliaryVel, double auxiliaryArbFeedFwd, uint32_t profileSlotSelect0, uint32_t profileSlotSelect1, bool isLastPoint, bool zeroPos, uint32_t timeDur, bool useAuxPID)
{
    m_Write_zeroPos.Set(zeroPos);
    m_Write_velocity.Set(velocity);
    m_Write_useAuxPID.Set(useAuxPID);
    m_Write_timeDur.Set(timeDur);
    m_Write_profileSlotSelect1.Set(profileSlotSelect1);
    m_Write_profileSlotSelect0.Set(profileSlotSelect0);
    m_Write_position.Set(position);
    m_Write_isLastPoint.Set(isLastPoint);
    m_Write_auxiliaryVel.Set(auxiliaryVel);
    m_Write_auxiliaryPos.Set(auxiliaryPos);
    m_Write_auxiliaryArbFeedFwd.Set(auxiliaryArbFeedFwd);
    m_Write_arbFeedFwd.Set(arbFeedFwd);

    Send("Write", position, velocity, arbFeedFwd, auxiliaryPos, auxiliaryVel, auxiliaryArbFeedFwd, profileSlotSelect0, profileSlotSelect1, isLastPoint, zeroPos, timeDur, useAuxPID);
}

void SnobotSim::CtreBuffTrajPointStreamWrapper::Lookup(void ** outObject)
{
    RECEIVE_HELPER("Lookup", sizeof(*outObject));
    PoplateReceiveResults(buffer, outObject, buffer_pos);

    *outObject = m_Lookup_outObject.Get();
}

