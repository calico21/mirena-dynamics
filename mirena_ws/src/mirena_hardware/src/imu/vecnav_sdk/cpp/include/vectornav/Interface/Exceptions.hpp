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

#ifndef VN_EXCEPTIONS_HPP_
#define VN_EXCEPTIONS_HPP_

#if __CLI__
#pragma managed(push, off)
#include "vectornav/Interface/Errors.hpp"
#include "vectornav/TemplateLibrary/String.hpp"
#pragma managed(pop)
#if __TESTS__
#include <msclr/marshal_cppstd.h>
#endif
#else
#include "vectornav/Interface/Errors.hpp"
#include "vectornav/TemplateLibrary/String.hpp"
#endif

namespace VN
{
inline String<40> errorCodeToMessage(ErrorAll error)
{
    String<40> msg = "Error: ";
    auto e = errorCodeToString(error);
    msg.append(e.data(), e.size());
    return msg;
}
}  // namespace VN

#if __CLI__
// clang-format off
using namespace System;

namespace VNSDK
{

public ref class VnException : public Exception
{
public:
    VnException(VN::ErrorAll error) : Exception(gcnew System::String(VN::errorCodeToMessage(error).c_str())) {}
};

#define _VN_DEF_EXCEPTION(className)                      \
public ref class className : public VnException           \
{                                                         \
public:                                                   \
    className(VN::ErrorAll error) : VnException(error) {} \
};

#define _VN_THROW_EXCEPTION(type, error) throw gcnew VNSDK::type(error)

// clang-format on
#else  // if !__CLI__

namespace VN
{

class VnException : public std::runtime_error
{
public:
    VnException(ErrorAll error) : std::runtime_error(errorCodeToMessage(error).data()) {}
};

#define _VN_DEF_EXCEPTION(className)                      \
    class className : public VnException                  \
    {                                                     \
    public:                                               \
        className(ErrorAll error) : VnException(error) {} \
    };

#define _VN_THROW_EXCEPTION(type, error) throw type(error)

#endif  // if __CLI__

_VN_DEF_EXCEPTION(VnSynchronousException)
_VN_DEF_EXCEPTION(VnAsynchronousException)
_VN_DEF_EXCEPTION(VnCommandProcessorException)
_VN_DEF_EXCEPTION(VnSensorException)
_VN_DEF_EXCEPTION(VnSerialException)
_VN_DEF_EXCEPTION(VnPacketSyncException)
_VN_DEF_EXCEPTION(VnFileException)
_VN_DEF_EXCEPTION(VnUnknownException)

#if __FIRMWARE_PROGRAMMER__
_VN_DEF_EXCEPTION(VnBLException)
#endif

#if __CALIBRATION__
_VN_DEF_EXCEPTION(VnCalException)
#endif

inline void throwError(VN::Error error)
{
    if (VN::VnErr_Synchronous::is_value(error)) { _VN_THROW_EXCEPTION(VnSynchronousException, error); }
    else if (VN::VnErr_Asynchronous::is_value(error)) { _VN_THROW_EXCEPTION(VnAsynchronousException, error); }
    else if (VN::VnErr_CommandProccessor::is_value(error)) { _VN_THROW_EXCEPTION(VnCommandProcessorException, error); }
    else if (VN::VnErr_Sensor::is_value(error)) { _VN_THROW_EXCEPTION(VnSensorException, error); }
    else if (VN::VnErr_Serial::is_value(error)) { _VN_THROW_EXCEPTION(VnSerialException, error); }
    else if (VN::VnErr_PacketSync::is_value(error)) { _VN_THROW_EXCEPTION(VnPacketSyncException, error); }
    else if (VN::VnErr_File::is_value(error)) { _VN_THROW_EXCEPTION(VnFileException, error); }
    else { _VN_THROW_EXCEPTION(VnUnknownException, error); }
}

#if __FIRMWARE_PROGRAMMER__
inline void throwError(VN::FirmwareProgrammer::ErrorBL error)
{
    if (VN::VnErr_Bootloader::is_value(error)) { _VN_THROW_EXCEPTION(VnBLException, error); }
    else { _VN_THROW_EXCEPTION(VnUnknownException, error); }
}
#endif

#if __CALIBRATION__
inline void throwError(VN::Calibration::ErrorCal error)
{
    if (VN::VnErr_Calibration::is_value(error)) { _VN_THROW_EXCEPTION(VnCalException, error); }
    else { _VN_THROW_EXCEPTION(VnUnknownException, error); }
}
#endif

inline void throwIfError(VN::ErrorAll error)
{
    if (error == VN::Error::None) { return; }
    if (auto err = error.get_opt<VN::Error>()) { throwError(err.value()); }
#if __FIRMWARE_PROGRAMMER__
    else if (auto err = error.get_opt<VN::FirmwareProgrammer::ErrorBL>()) { throwError(err.value()); }
#endif
#if __CALIBRATION__
    else if (auto err = error.get_opt<VN::Calibration::ErrorCal>()) { throwError(err.value()); }
#endif
    else { _VN_THROW_EXCEPTION(VnUnknownException, error); }
}

#if __TESTS__
inline void testExceptionThrow(std::string type, int code)
{
    if (type == "Error") { throwIfError(VN::Error(code)); }
#if __FIRMWARE_PROGRAMMER__
    else if (type == "ErrorBL") { throwIfError(VN::FirmwareProgrammer::ErrorBL(code)); }
#endif
#if __CALIBRATION__
    else if (type == "ErrorCal") { throwIfError(VN::Calibration::ErrorCal(code)); }
#endif
}

#if __CLI__
// clang-format off
using namespace msclr::interop;

public ref class TestUtilities
{
public:
    static void TestExceptionThrow(String^ type, int code) { testExceptionThrow(marshal_as<std::string>(type), code); }
};
// clang-format on
#endif  // __CLI__
#endif  // __TESTS__
}  // namespace VNSDK or VN

#undef _VN_DEF_EXCEPTION
#undef _VN_THROW_EXCEPTION

#endif  // VN_EXCEPTIONS_HPP_
