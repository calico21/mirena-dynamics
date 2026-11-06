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

#ifndef VN_EXPORTERASCII_HPP_
#define VN_EXPORTERASCII_HPP_

#include "vectornav/Debug.hpp"
#include "vectornav/Exporter.hpp"
#include "vectornav/HAL/File.hpp"
#include "vectornav/Implementation/Packet.hpp"
#include "vectornav/Interface/GenericCommand.hpp"

namespace VN
{
namespace DataExport
{

class ExporterAscii : public Exporter
{
public:
    struct AsciiGrouping
    {
        uint8_t groupAll : 1;
        uint8_t groupVnMeasurements : 1;
        uint8_t groupVnCommands : 1;
        uint8_t groupNmea : 1;
        uint8_t : 4;

        AsciiGrouping() = default;
        explicit AsciiGrouping(uint8_t other) noexcept { std::memcpy(this, &other, sizeof(AsciiGrouping)); }

        explicit operator uint8_t() const
        {
            uint8_t result;
            std::memcpy(&result, this, sizeof(AsciiGrouping));
            return result;
        }

        AsciiGrouping& operator=(uint8_t other)
        {
            std::memcpy(this, &other, sizeof(AsciiGrouping));
            return *this;
        }
    };

    static_assert(sizeof(ExporterAscii::AsciiGrouping) == 1, "AsciiGrouping must be exactly 1 byte");

    ExporterAscii(const Filesystem::FilePath& outputDir, PacketQueueMode mode = PacketQueueMode::Force, bool enableSystemTimeStamps = false,
                  AsciiGrouping grouping = {})
        : Exporter(EXPORTER_PACKET_CAPACITY, mode), _filePath(outputDir), _grouping(grouping), _enableSystemTimeStamps(enableSystemTimeStamps)
    {
        if (!_filePath.empty() && _filePath.back() != std::filesystem::path::preferred_separator)
        {
            _filePath = _filePath + std::filesystem::path::preferred_separator;
        }
    }

    ~ExporterAscii() = default;

    void exportToFile() override;

private:
    static constexpr uint8_t MAX_NUM_FILES = 10;
    static constexpr uint16_t EXPORTER_PACKET_CAPACITY = 2048;

    struct AsciiInfo
    {
        ShortString header;
        OutputFile file;
    };

    ShortString _resolveAsciiHeader(const Packet* p) const;
    void _flushAllFiles();
    OutputFile& _getFileHandle(const ShortString& header);

    Filesystem::FilePath _filePath;
    AsciiGrouping _grouping{};
    const bool _enableSystemTimeStamps = false;

    Vector<AsciiInfo, MAX_NUM_FILES> _asciiInfo;  // One entry per file created, which is per unique message
};

}  // namespace DataExport
}  // namespace VN

#endif  // VN_EXPORTERASCII_HPP_
