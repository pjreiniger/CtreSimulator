
#include "CtreSimMocks/CtreCANCoderWrapper.h"

#include <vector>

#include "CtreSimUtils/MockHooks.h"

SnobotSim::CtreCANCoderWrapper::CtreCANCoderWrapper(int deviceId)
{
    Send("Create");
}

void SnobotSim::CtreCANCoderWrapper::Send(const std::string& aName,
        uint8_t* aBuffer, int aSize)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
}

void SnobotSim::CtreCANCoderWrapper::Receive(const std::string& aName,
        uint8_t* aBuffer,
        int aSize)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
}
