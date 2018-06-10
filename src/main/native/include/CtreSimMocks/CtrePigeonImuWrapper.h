
#ifndef CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_CTREPIGEONIMUWRAPPER_H_
#define CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_CTREPIGEONIMUWRAPPER_H_

#include <string>

#include "CtreSimMocks/BaseCtreWrapper.h"

namespace SnobotSim
{

class CtrePigeonImuWrapper : public BaseCtreWrapper
{
public:
    explicit CtrePigeonImuWrapper(int aDeviceId);

private:
    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Send(const std::string& aName);

    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);

    const int mDeviceId;
};

} // namespace SnobotSim

#endif // CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_CTREPIGEONIMUWRAPPER_H_
