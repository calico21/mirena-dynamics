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

#ifndef VN_EXPORTERCSVUTILS_HPP_
#define VN_EXPORTERCSVUTILS_HPP_

#include <stdint.h>

#include <string_view>

#include "vectornav/Implementation/AsciiPacketProtocol.hpp"

namespace VN
{
namespace DataExport
{

inline uint8_t getGnssGroup(uint8_t group) noexcept
{
    switch (group)
    {
        case 3:
            return 1;
        case 6:
            return 2;
        case 12:
            return 3;
        default:
            return 0;
    }
}
inline bool isSatField(uint8_t group, uint8_t field) noexcept { return getGnssGroup(group) > 0 && field == 14; }
inline bool isRawField(uint8_t group, uint8_t field) noexcept { return getGnssGroup(group) > 0 && field == 16; }
inline bool isDynamicField(uint8_t group, uint8_t field) noexcept { return isSatField(group, field) || isRawField(group, field); }

const std::string_view getMeasurementNames(const AsciiPacketProtocol::AsciiMeasurementHeader& msg);

const std::string_view getMeasurementNames(const size_t binaryGroup, const size_t binaryField);

}  // namespace DataExport
}  // namespace VN

#endif  // VN_EXPORTERCSVUTILS_HPP_
