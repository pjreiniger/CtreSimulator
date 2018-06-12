
#ifndef CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_CTREPIGEONIMUWRAPPER_H_
#define CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_CTREPIGEONIMUWRAPPER_H_

#include <string>

#include "CtreSimMocks/BaseCtreWrapper.h"

namespace SnobotSim
{

class CtrePigeonImuWrapper : public BaseCtreWrapper
{
public:
    using BaseCtreWrapper::Send;

    explicit CtrePigeonImuWrapper(int aDeviceId);
    const int mDeviceId;

    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);

};

}  // namespace SnobotSim

#endif  // CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_CTREPIGEONIMUWRAPPER_H_
