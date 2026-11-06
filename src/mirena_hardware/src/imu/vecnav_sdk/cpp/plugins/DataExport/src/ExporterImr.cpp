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

#include "vectornav/ExporterImr.hpp"

#include "vectornav/Implementation/FaPacketProtocol.hpp"
#include "vectornav/Interface/CompositeData.hpp"
#include "vectornav/TemplateLibrary/ByteBuffer.hpp"

namespace VN
{
namespace DataExport
{

void ExporterImr::exportToFile()
{
    static constexpr uint64_t nsInWeek = static_cast<uint64_t>(7 * 24 * 3600) * 1000000000;

    while (!_queue.isEmpty())
    {
        const auto p = _queue.get();
        if (!p) { return; }
        ByteBuffer byteBuffer(p->buffer, p->capacity, p->capacity);

        const auto cd = FaPacketProtocol::parsePacket(byteBuffer, 0, p->details.faMetadata, Config::PacketDispatchers::cdEnabledMeasTypes);

        if (!cd) return;

        if (cd->time.timeGps.has_value()) { _data.gpsTow = static_cast<double>(cd->time.timeGps.value().nanoseconds() % nsInWeek) / 1000000000; }
        else if (cd->time.timeGpsTow.has_value()) { _data.gpsTow = static_cast<double>(cd->time.timeGpsTow.value().nanoseconds()) / 1000000000; }
        else
        {
            // no timestamps available
            return;
        }

        if (_prevTime != 0) { _avgDt += ((_data.gpsTow - _prevTime) - _avgDt) / (_numSamples++); }
        _prevTime = _data.gpsTow;

        // find imu data
        if (cd->imu.uncompGyro.has_value() && cd->imu.uncompAccel.has_value())
        {
            _data.uncompGyroX = (cd->imu.uncompGyro.value()[0] * RAD2DEG) * scaleFactor;
            _data.uncompGyroY = (cd->imu.uncompGyro.value()[1] * RAD2DEG) * scaleFactor;
            _data.uncompGyroZ = (cd->imu.uncompGyro.value()[2] * RAD2DEG) * scaleFactor;
            _data.uncompAccelX = cd->imu.uncompAccel.value()[0] * scaleFactor;
            _data.uncompAccelY = cd->imu.uncompAccel.value()[1] * scaleFactor;
            _data.uncompAccelZ = cd->imu.uncompAccel.value()[2] * scaleFactor;
        }
        else
        {
            // no inertial data available
            return;
        }

        _file.write(reinterpret_cast<char*>(&_data), sizeof(_data));
    }
}

void ExporterImr::_finalize()
{
    _file.reset();

    _header.dataRateHz = std::round(1.0 / _avgDt);
    _header.gyroScaleFactor = 1.0 / static_cast<double>(scaleFactor);
    _header.accelScaleFactor = 1.0 / static_cast<double>(scaleFactor);
    _header.levelArmValid = 0;
    _header.IYoffset = 0;
    _header.IXoffset = 0;
    _header.IZoffset = 0;

    VN_DEBUG_0("Rate: " + std::to_string(_header.dataRateHz) + " Hz" << std::endl);
    VN_DEBUG_0("Samples: " + std::to_string(_numSamples) << std::endl);

    _file.write(reinterpret_cast<char*>(&_header), sizeof(_header));
}

}  // namespace DataExport
}  // namespace VN
