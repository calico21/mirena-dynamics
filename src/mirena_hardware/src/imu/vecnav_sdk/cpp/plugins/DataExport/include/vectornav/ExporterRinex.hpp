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

#ifndef VN_EXPORTERRINEX_HPP_
#define VN_EXPORTERRINEX_HPP_

#include <array>
#include <ctime>
#include <limits>

#include "vectornav/Config.hpp"
#include "vectornav/Exporter.hpp"
#include "vectornav/HAL/File.hpp"
#include "vectornav/Implementation/BinaryMeasurementDefinitions.hpp"

#if ((GNSS_GROUP_ENABLE & GNSS_GNSS1RAWMEAS_BIT) || (GNSS2_GROUP_ENABLE & GNSS2_GNSS2RAWMEAS_BIT))
#define VN_EXPORTER_RINEX_ENABLED

namespace VN
{
namespace DataExport
{

class ExporterRinex : public Exporter
{
public:
    ExporterRinex(const Filesystem::FilePath& fileName, const uint32_t gnssGroup, PacketQueueMode mode = PacketQueueMode::Force)
        : Exporter(EXPORTER_PACKET_CAPACITY, mode), _fileName(fileName), _gnssGroup(gnssGroup)
    {
        switch (gnssGroup)
        {
            case 1:
            {
                _fileName = _fileName + "-1";
                _gnssGroup = GNSS_BIT;
                break;
            }
            case 2:
            {
                _fileName = _fileName + "-2";
                _gnssGroup = GNSS2_BIT;
                break;
            }
            default:
                VN_ABORT();
        }
        _file.open(_fileName + ".obs");
        if (!_file.is_open()) { VN_ABORT(); }
        for (int i = 0; i < 19; i++) { _file.write("                                                                                \n"); }
    }

    ~ExporterRinex() { _finalize(); }

    void exportToFile() override;

private:
    static constexpr uint16_t EXPORTER_PACKET_CAPACITY = 2048;
    static constexpr uint16_t STRING_BUFFER_CAPACITY = 2048;

    void _finalize();

    static constexpr double _GPS_C = 299792458.0;
    static constexpr std::array<char, 9> _sysId = {'G', 'S', 'E', 'C', 'X', 'J', 'R', 'I', 'L'};
    static constexpr std::array<char, 15> _chanCode = {'P', 'C', 'D', 'Y', 'M', 'N', 'A', 'B', 'I', 'Q', 'S', 'L', 'X', 'W', 'Z'};

    char _buffer[STRING_BUFFER_CAPACITY];

    time_t _firstTimeGps = std::numeric_limits<time_t>::max();
    double _firstTimeFrac = 0.0;
    time_t _lastTimeGps = 0;
    double _lastTimeFrac = 0.0;

    Filesystem::FilePath _fileName;
    OutputFile _file;
    uint32_t _gnssGroup;

    std::array<std::array<uint8_t, _chanCode.size()>, _sysId.size()> _trackedSatelliteInfo{};
};

}  // namespace DataExport
}  // namespace VN

#endif  // VN_EXPORTER_RINEX_ENABLED

#endif  // VN_EXPORTERRINEX_HPP_
