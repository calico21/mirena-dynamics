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

#ifndef VN_EXPORTERCSV_HPP_
#define VN_EXPORTERCSV_HPP_

#include "vectornav/Exporter.hpp"
#include "vectornav/HAL/File.hpp"
#include "vectornav/Implementation/Packet.hpp"

namespace VN
{

class BinaryHeader;

namespace DataExport
{

class ExporterCsv : public Exporter
{
public:
    ExporterCsv(const Filesystem::FilePath& outputDir, PacketQueueMode mode = PacketQueueMode::Force, bool enableSystemTimeStamps = false)
        : Exporter(EXPORTER_PACKET_CAPACITY, mode), _filePath(outputDir), _enableSystemTimeStamps(enableSystemTimeStamps)
    {
        if (!_filePath.empty() && _filePath.back() != std::filesystem::path::preferred_separator)
        {
            _filePath = _filePath + std::filesystem::path::preferred_separator;
        }
    }

    // Overload 1: Enable timestamps only (defaults to current directory, Force mode)
    explicit ExporterCsv(bool enableSystemTimeStamps) : ExporterCsv("", PacketQueueMode::Force, enableSystemTimeStamps) {}

    // Overload 2: Path + Enable timestamps (defaults to Force mode)
    ExporterCsv(const Filesystem::FilePath& outputDir, bool enableSystemTimeStamps) : ExporterCsv(outputDir, PacketQueueMode::Force, enableSystemTimeStamps) {}

    ~ExporterCsv() = default;

    void exportToFile() override;

private:
    static constexpr uint8_t MAX_NUM_FILES = 4;
    static constexpr uint16_t EXPORTER_PACKET_CAPACITY = 2048;
    static constexpr uint16_t STRING_BUFFER_CAPACITY = 4096;

    struct CsvInfo
    {
        PacketDetails details;
        OutputFile file;
    };

    struct DynamicCsvInfo
    {
        uint8_t measGroupNum;
        uint8_t measTypeNum;
        OutputFile file;
    };

    class CsvExtractor : public FaPacketExtractor
    {
    public:
        using FaPacketExtractor::FaPacketExtractor;

        void setFile(OutputFile& file) { _file = &file; }

        template <class T>
        Errored extract(std::optional<T>& value) noexcept
        {
            if (FaPacketExtractor::extract(value)) { return true; }

            int numBytes = measPrint(_buffer, BUFFER_SIZE, *value);
            if (numBytes < 0) { return true; }
            else if (numBytes > BUFFER_SIZE)
            {
                numBytes = BUFFER_SIZE;
                VN_DEBUG_0("CsvExtractor Print Overflow\n");
            }

            _file->write(_buffer, numBytes);

            return false;
        }

        OutputFile* _file = nullptr;
        static constexpr uint16_t BUFFER_SIZE = STRING_BUFFER_CAPACITY;
        char _buffer[BUFFER_SIZE];
    };

    void _initCsv(CsvInfo& csvInfo, const Packet* p);
    void _initDynamicCsv(DynamicCsvInfo& dynamicCsvInfo, uint8_t measGroupNum, uint8_t measTypeNum);
    void _initSatInfo(DynamicCsvInfo& dynamicCsvInfo, uint8_t measGroupNum, uint8_t measTypeNum);
    void _initRawMeas(DynamicCsvInfo& dynamicCsvInfo, uint8_t measGroupNum, uint8_t measTypeNum);
    void _writeTimestamp(OutputFile& file, time_point timestamp);
    OutputFile& _getFileHandle(const Packet* p);
    OutputFile& _getDynamicFileHandle(uint8_t measGroupNum, uint8_t measTypeNum, const BinaryHeader& header);
    void _flushAllFiles();

    Filesystem::FilePath _filePath;
    const bool _enableSystemTimeStamps = false;

    Vector<CsvInfo, MAX_NUM_FILES> _csvInfo;                // One entry per file created, which is per unique message type
    Vector<DynamicCsvInfo, MAX_NUM_FILES> _dynamicCsvInfo;  // One entry per file created, which is per unique message type
};

}  // namespace DataExport
}  // namespace VN

#endif  // VN_EXPORTERCSV_HPP_
