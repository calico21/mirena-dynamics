// VectorNav SDK (v1.2.0)
// Copyright (c) 2024 VectorNav Technologies, LLC
// 
// WARNING - This software contains Controlled Technical Data, export of which
// is restricted by the Export Administration Regulations ("EAR") (ECCN 7D994). 
// Disclosure to foreign persons contrary to the EAR is prohibited. Violations
// of these export laws and regulations are subject to severe civil and criminal
// penalties.
// 
// The MIT License (MIT)
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef VN_SERIAL_TCP_WINDOWS_HPP_
#define VN_SERIAL_TCP_WINDOWS_HPP_

#include <stdio.h>
#include <stdlib.h>

#include <cstdint>
#include <functional>

#include "vectornav/Debug.hpp"
#include "vectornav/HAL/Serial_Base.hpp"
#include "vectornav/HAL/Thread_Base.hpp"
#include "vectornav/HAL/Winsock_Headers.hpp"
#include "vectornav/Interface/Errors.hpp"

namespace VN
{

class SerialTcp : public Serial_Base
{
public:
    using Serial_Base::Serial_Base;

    // ***********
    // Port access
    // ***********
    Error open(const PortName& portName, const uint32_t portNumber) noexcept override final;
    void close() noexcept override;
    Error changeBaudRate(const uint32_t baudRate) noexcept override final;
    std::optional<PortName> connectedPortName() const noexcept override final;

    // ***************
    // Callback setup
    // ***************
    void setChangeBaudRate(std::function<Errored(uint32_t)> callback) noexcept;

    // ***************
    // Port read/write
    // ***************
    Error getData() noexcept override final;
    Error send(const char* buffer, const size_t len) noexcept override final;
    int getRawTcpError() { return tcpErr; }

private:
    // ***********
    // Port access
    // ***********
    SOCKET _socket = INVALID_SOCKET;
    int tcpErr;
    std::function<Errored(uint32_t)> _changeBaudRate = nullptr;
    uint16_t _portNumber;

    static Error mapTcpErrors(int err) noexcept
    {
        switch (err)
        {
            case WSAECONNREFUSED:
                return Error::AccessDenied;
            case WSAENETRESET:
            case WSAECONNABORTED:
            case WSAECONNRESET:
            case WSAENOTCONN:
            case WSAETIMEDOUT:
                return Error::SerialPortClosed;
            case WSAEMSGSIZE:
                return Error::SerialWriteFailed;
            case WSAEWOULDBLOCK:
                return Error::SerialReadFailed;
            case EAI_NONAME:
                return Error::InvalidPortName;
            default:
                return Error::UnexpectedTcpError;
        }
    }
};

// ######################
//     Implementation
// ######################
inline Error SerialTcp::open(const PortName& portName, const uint32_t portNumber) noexcept

{
    if (_isOpen) { close(); }
    else { thisThread::sleepFor(10ms); }

    WSADATA wsaData;
    struct addrinfo *result = NULL, hints;

    // Initialize Winsock
    tcpErr = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (tcpErr != 0) { return mapTcpErrors(tcpErr); }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    char port_buf[8];
    std::snprintf(port_buf, sizeof(port_buf), "%u", portNumber);

    // Resolve the server address and port
    int tcpError = getaddrinfo(portName.c_str(), port_buf, &hints, &result);
    if (tcpError != 0)
    {
        WSACleanup();
        return mapTcpErrors(tcpError);
    }

    // Attempt to connect to an address until one succeeds
    for (addrinfo* ptr = result; ptr != NULL; ptr = ptr->ai_next)
    {
        // Create a SOCKET for connecting to server
        _socket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
        if (_socket == INVALID_SOCKET)
        {
            tcpErr = WSAGetLastError();
            WSACleanup();
            return mapTcpErrors(tcpErr);
        }

        int yes = 1;
        if (setsockopt(_socket, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&yes), sizeof(yes)) == SOCKET_ERROR)  // 1 - on, 0 - off
        {
            tcpErr = WSAGetLastError();
            closesocket(_socket);
            _socket = INVALID_SOCKET;
            continue;
        }

        // Connect to server.
        if (connect(_socket, ptr->ai_addr, static_cast<int>(ptr->ai_addrlen)) == SOCKET_ERROR)
        {
            tcpErr = WSAGetLastError();
            closesocket(_socket);
            _socket = INVALID_SOCKET;
            continue;
        }
        break;
    }

    freeaddrinfo(result);

    if (_socket == INVALID_SOCKET)
    {
        WSACleanup();
        return mapTcpErrors(tcpErr);
    }

    _portName = portName;
    _portNumber = static_cast<uint16_t>(portNumber);
    _isOpen = true;
    return Error::None;
}

inline Error SerialTcp::send(const char* buffer, const size_t len) noexcept
{
    if (_isOpen == false) { return Error::SerialPortClosed; }

    if (::send(_socket, buffer, len, 0) == SOCKET_ERROR)
    {
        tcpErr = WSAGetLastError();
        return mapTcpErrors(tcpErr);
    }
    return Error::None;
}

inline Error SerialTcp::getData() noexcept
{
    if (!_isOpen) { return Error::SerialPortClosed; }

    DWORD bytesRemaining = 0;
    if (ioctlsocket(_socket, FIONREAD, &bytesRemaining) != 0) { return Error::SerialReadFailed; }

    size_t linearBytes = _byteBuffer.numLinearBytesToPut();

    while (bytesRemaining > 0 && linearBytes > 0)
    {
        const int bytes_read = ::recv(_socket, reinterpret_cast<char*>(const_cast<uint8_t*>(_byteBuffer.tail())),
                                      static_cast<int>(std::min<DWORD>(bytesRemaining, static_cast<DWORD>(linearBytes))), 0);

        if (bytes_read == 0) { return Error::SerialPortClosed; }

        if (bytes_read < 0)
        {
            tcpErr = WSAGetLastError();
            return mapTcpErrors(tcpErr);
        }

        _byteBuffer.put(static_cast<size_t>(bytes_read));

        bytesRemaining -= static_cast<u_long>(bytes_read);
        linearBytes = _byteBuffer.numLinearBytesToPut();
    }

    if (bytesRemaining > 0) { return Error::PrimaryBufferFull; }

    return Error::None;
}

inline Error SerialTcp::changeBaudRate(const uint32_t baudRate) noexcept
{
    if (!_isOpen) { return Error::SerialPortClosed; }
    if (!_changeBaudRate)
    {
        VN_DEBUG_1("SerialTcp changeBaudRate callback not set" << std::endl;)
        return Error::None;
    }
    if (_changeBaudRate(baudRate))
    {
        _baudRate = baudRate;
        return Error::None;
    }
    else { return Error::ChangeHostBaudRateFailed; }
}

inline void SerialTcp::setChangeBaudRate(std::function<Errored(uint32_t)> callback) noexcept { _changeBaudRate = callback; }

inline std::optional<Serial_Base::PortName> SerialTcp::connectedPortName() const noexcept
{
    String<32> portNumberStr;
    portNumberStr.assign("%u", _portNumber);
    return _isOpen ? std::make_optional(_portName + ":" + portNumberStr) : std::nullopt;
};

inline void SerialTcp::close() noexcept
{
    _isOpen = false;
    closesocket(_socket);
    WSACleanup();
    thisThread::sleepFor(10ms);
}

}  // namespace VN

#endif  // VN_SERIAL_TCP_WINDOWS_HPP_
