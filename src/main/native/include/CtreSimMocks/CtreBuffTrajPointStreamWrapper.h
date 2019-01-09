
#ifndef SRC_MAIN_NATIVE_CPP_CTRESIMMOCKS_CTREBUFFTRAJPOINTSTREAMWRAPPER_H_
#define SRC_MAIN_NATIVE_CPP_CTRESIMMOCKS_CTREBUFFTRAJPOINTSTREAMWRAPPER_H_

#include <string>

#include "CtreSimMocks/BaseCtreWrapper.h"

namespace SnobotSim
{

class CtreBuffTrajPointStreamWrapper : public BaseCtreWrapper
{
public:
    using BaseCtreWrapper::Send;

    CtreBuffTrajPointStreamWrapper();

    void Receive(const std::string& aName, uint8_t* aBuffer, int aSize);
    void Send(const std::string& aName, uint8_t* aBuffer, int aSize);
};

}

#endif /* SRC_MAIN_NATIVE_CPP_CTRESIMMOCKS_CTREBUFFTRAJPOINTSTREAMWRAPPER_H_ */
