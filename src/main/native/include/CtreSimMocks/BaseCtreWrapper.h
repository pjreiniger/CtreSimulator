
#ifndef CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_BASECTREWRAPPER_H_
#define CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_BASECTREWRAPPER_H_

#include <string>

namespace SnobotSim
{

class BaseCtreWrapper
{
protected:
    virtual void Send(const std::string& aName, uint8_t* aBuffer, int aSize) = 0;
    virtual void Send(const std::string& aName) = 0;

    virtual void Receive(const std::string& aName, uint8_t* aBuffer,
            int aSize)
            = 0;

    template <typename T0>
    void Send(const std::string& aName, T0& param0)
    {
        int size = sizeof(T0);

        uint8_t* buffer = new uint8_t[size];
        std::memset(&buffer[0], 0, size);

        uint32_t offset = 0;
        PushValue(buffer, param0, offset);
        Send(aName, buffer, size);

        delete[] buffer;
    }

    template <typename T0, typename T1>
    void Send(const std::string& aName, T0& param0, T1& param1)
    {
        int size = sizeof(T0) + sizeof(T1);

        uint8_t* buffer = new uint8_t[size];
        std::memset(&buffer[0], 0, size);

        uint32_t offset = 0;
        PushValue(buffer, param0, offset);
        PushValue(buffer, param1, offset);
        Send(aName, buffer, size);

        delete[] buffer;
    }

    template <typename T0, typename T1, typename T2>
    void Send(const std::string& aName, T0& param0, T1& param1, T2& param2)
    {
        int size = sizeof(T0) + sizeof(T1) + sizeof(T2);

        uint8_t* buffer = new uint8_t[size];
        std::memset(&buffer[0], 0, size);

        uint32_t offset = 0;
        PushValue(buffer, param0, offset);
        PushValue(buffer, param1, offset);
        PushValue(buffer, param2, offset);
        Send(aName, buffer, size);

        delete[] buffer;
    }

    template <typename T0, typename T1, typename T2, typename T3, typename T4>
    void Send(const std::string& aName, T0& param0, T1& param1, T2& param2,
            T3& param3, T4& param4)
    {
        int size = sizeof(T0) + sizeof(T1) + sizeof(T2) + sizeof(T3) + sizeof(T4);

        uint8_t* buffer = new uint8_t[size];
        std::memset(&buffer[0], 0, size);

        uint32_t offset = 0;
        PushValue(buffer, param0, offset);
        PushValue(buffer, param1, offset);
        PushValue(buffer, param2, offset);
        PushValue(buffer, param3, offset);
        PushValue(buffer, param4, offset);
        Send(aName, buffer, size);

        delete[] buffer;
    }

    template <typename T>
    void PushValue(uint8_t* buffer, T& value, uint32_t& offset)
    {
        std::memcpy(&buffer[offset], &value, sizeof(value));
        offset += sizeof(value);
    }
};

} // namespace SnobotSim

#endif // CTRESIMULATOR_SRC_MAIN_NATIVE_INCLUDE_CTRESIMMOCKS_BASECTREWRAPPER_H_
