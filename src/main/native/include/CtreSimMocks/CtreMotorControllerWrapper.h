
#ifndef CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_CTREMOTORCONTROLLERWRAPPER_H_
#define CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_CTREMOTORCONTROLLERWRAPPER_H_

#include <string>

#include "CtreSimMocks/BaseCtreWrapper.h"

namespace SnobotSim
{

class CtreMotorControllerWrapper : public BaseCtreWrapper
{
public:
    explicit CtreMotorControllerWrapper(int aDeviceId);

private:
    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Send(const std::string& aName);

    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);

    const int mDeviceId;
};

} // namespace SnobotSim

#endif // CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_CTREMOTORCONTROLLERWRAPPER_H_
