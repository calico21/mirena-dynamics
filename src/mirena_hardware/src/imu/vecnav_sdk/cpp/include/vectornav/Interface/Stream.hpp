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

// vectornav/Interface/Stream.hpp
#ifndef VN_INTERFACE_STREAM_HPP_
#define VN_INTERFACE_STREAM_HPP_

#include <ostream>
#include <sstream>
#include <type_traits>

#include "vectornav/HAL/Timer.hpp"
#include "vectornav/Implementation/MeasurementDatatypes.hpp"
#include "vectornav/Interface/Errors.hpp"
#include "vectornav/Interface/Registers.hpp"
#include "vectornav/TemplateLibrary/Matrix.hpp"
#include "vectornav/TemplateLibrary/String.hpp"

namespace VN
{

template <uint16_t N>
inline std::ostream& operator<<(std::ostream& os, const String<N>& s) noexcept
{
    os << s.c_str();
    return os;
}

template <typename T, std::enable_if_t<is_same_any<T, _VN_ERROR_LIST>, bool> = true>
inline std::ostream& operator<<(std::ostream& outStream, const T& error) noexcept
{
    outStream << "Error " << static_cast<uint16_t>(error) << ": " << errorCodeToString(error);
    return outStream;
}

inline std::ostream& operator<<(std::ostream& outStream, const ErrorAll& error) noexcept
{
    std::visit([&outStream](auto&& arg) { outStream << arg; }, error._e);
    return outStream;
}

inline std::ostream& operator<<(std::ostream& os, const time_point& tp) noexcept
{
    os << std::chrono::duration_cast<Milliseconds>(tp.time_since_epoch()).count() << " ms";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const AsyncError& asyncError) noexcept
{
    os << asyncError.message << " (" << asyncError.error << ") at " << asyncError.timestamp;
    return os;
}

template <uint16_t m, uint16_t n, typename T>
inline std::ostream& operator<<(std::ostream& out, const Matrix<m, n, T>& mat) noexcept
{
    out << "[";
    for (uint16_t i = 0; i < m; ++i)
    {
        if (i == 0) { out << "("; }
        else { out << '\n' << "("; }

        for (uint16_t j = 0; j < n; ++j) { out << ' ' << mat(i, j) << ','; }

        out << '\b' << ' ' << ')';
    }
    out << ']' << '\n';
    return out;
}

inline std::ostream& operator<<(std::ostream& outStream, const Registers::System::BaudRate::BaudRates& baudrate) noexcept
{
    return outStream << static_cast<uint32_t>(baudrate);
}

template <typename T, std::enable_if_t<Detail::HasBufferPrint<T>::value, int> = 0>
inline std::ostream& operator<<(std::ostream& os, const T& value) noexcept
{
    char buf[2000]{};
    value.print(buf, sizeof(buf));
    return os << buf;
}

}  // namespace VN

#endif  // VN_INTERFACE_STREAM_HPP_
