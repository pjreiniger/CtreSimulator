
#ifndef CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_CTREMOTORCONTROLLERWRAPPER_H_
#define CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_CTREMOTORCONTROLLERWRAPPER_H_

#include <string>

#include "CtreSimMocks/BaseCtreWrapper.h"

namespace SnobotSim
{

class CtreMotorControllerWrapper : public BaseCtreWrapper
{
public:
    using BaseCtreWrapper::Send;

    explicit CtreMotorControllerWrapper(int aDeviceId);
    const int mDeviceId;

    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);
};

}  // namespace SnobotSim

#endif  // CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_CTREMOTORCONTROLLERWRAPPER_H_
