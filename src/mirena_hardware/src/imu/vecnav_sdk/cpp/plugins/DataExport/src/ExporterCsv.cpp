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

#include "vectornav/ExporterCsv.hpp"

// #include <algorithm>
// #include <chrono>

#include "vectornav/ExporterCsvUtils.hpp"
#include "vectornav/Implementation/BinaryHeader.hpp"
#include "vectornav/Implementation/MeasurementDatatypes.hpp"

namespace VN
{
namespace DataExport
{

void ExporterCsv::exportToFile()
{
    while (!_queue.isEmpty())
    {
        const auto p = _queue.get();
        if (!p) { return; }

        if (_csvInfo.size() == _csvInfo.capacity())
        {
            VN_DEBUG_1("Packet dropped.");
            continue;
        }

        OutputFile& csv = _getFileHandle(p.get());
        if (_enableSystemTimeStamps)
        {
            const auto timestamp =
                (p->details.syncByte == PacketDetails::SyncByte::Ascii) ? p->details.asciiMetadata.timestamp : p->details.faMetadata.timestamp;
            _writeTimestamp(csv, timestamp);
        }

        if (p->details.syncByte == PacketDetails::SyncByte::Ascii)
        {
            const size_t begin = p->details.asciiMetadata.delimiterIndices.front() + 1;
            const size_t end = p->details.asciiMetadata.delimiterIndices.back();
            csv.write(reinterpret_cast<const char*>(&p->buffer[begin]), end - begin);
        }
        else
        {
            CsvExtractor extractor(p->buffer, p->details.faMetadata);
            extractor.discard(p->details.faMetadata.header.size() + 1);
            extractor.setFile(csv);
            bool first_meas_of_line = true;

            BinaryHeaderIterator iter(p->details.faMetadata.header);
            CompositeData cd;
            while (iter.next())
            {
                if (isDynamicField(iter.group(), iter.field()))
                {
                    OutputFile& dynamicCsv = _getDynamicFileHandle(iter.group(), iter.field(), p->details.faMetadata.header);
                    extractor.setFile(dynamicCsv);
                    if (_enableSystemTimeStamps) { _writeTimestamp(dynamicCsv, p->details.faMetadata.timestamp); }
                    cd.copyFromBuffer(extractor, iter.group(), iter.field());
                    extractor.setFile(csv);  // Switch back to main file
                }
                else
                {
                    if (!first_meas_of_line) { csv.write(","); }
                    first_meas_of_line = false;
                    cd.copyFromBuffer(extractor, iter.group(), iter.field());
                }
            }
        }
        csv.write("\n");
    }
    _flushAllFiles();
}

void ExporterCsv::_initCsv(CsvInfo& csvInfo, const Packet* p)
{
    if (_enableSystemTimeStamps) { csvInfo.file.write("systemTimeStamp,"); }

    if (csvInfo.details.syncByte == PacketDetails::SyncByte::Ascii)
    {
        const AsciiPacketProtocol::AsciiMeasurementHeader asciiHeader = AsciiPacketProtocol::getMeasHeader(csvInfo.details.asciiMetadata.header);
        csvInfo.file.write(getMeasurementNames(asciiHeader));

        if (std::find(&p->buffer[7], &p->buffer[p->details.asciiMetadata.length], 'S') != &p->buffer[p->details.asciiMetadata.length])
        {
            csvInfo.file.write(",appendStatus");
        }

        if (std::find(&p->buffer[7], &p->buffer[p->details.asciiMetadata.length], 'T') != &p->buffer[p->details.asciiMetadata.length])
        {
            csvInfo.file.write(",appendCount");
        }

        csvInfo.file.write("\n");
    }
    else
    {
        bool firstMeas = true;
        BinaryHeaderIterator iter(p->details.faMetadata.header);
        while (iter.next())
        {
            // Dynamic fields (SAT/RAW) are initialized separately with their own files
            if (isDynamicField(iter.group(), iter.field())) { continue; }

            if (!firstMeas) { csvInfo.file.write(","); }
            firstMeas = false;
            csvInfo.file.write(getMeasurementNames(iter.group(), iter.field()));
        }
        csvInfo.file.write("\n");
    }
}

void ExporterCsv::_initDynamicCsv(DynamicCsvInfo& dynamicCsvInfo, const uint8_t measGroupNum, const uint8_t measTypeNum)
{
    if (_enableSystemTimeStamps) { dynamicCsvInfo.file.write("systemTimeStamp,"); }

    if (isSatField(measGroupNum, measTypeNum)) { _initSatInfo(dynamicCsvInfo, measGroupNum, measTypeNum); }
    else { _initRawMeas(dynamicCsvInfo, measGroupNum, measTypeNum); }
}

void ExporterCsv::_initSatInfo(DynamicCsvInfo& dynamicCsvInfo, const uint8_t measGroupNum, const uint8_t measTypeNum)
{
    auto names = getMeasurementNames(measGroupNum, measTypeNum);
    char outString[256];

    const uint8_t skipLength = sizeof("Gnss1NumSats") - 1;
    uint8_t skip = 0;
    for (uint8_t i = 0; i < GNSS_SAT_INFO_MAX_COUNT; i++)
    {
        if (i == 1) { skip = skipLength; }
        std::memcpy(outString, names.data() + skip, names.size());

        char satNum[] = "00";
        std::snprintf(satNum, sizeof(satNum), "%02d", i);

        for (uint16_t j = 0; j < names.size() - skip; j++)
        {
            if (outString[j] == '?')
            {
                outString[j++] = satNum[0];
                outString[j++] = satNum[1];
            }
        }
        dynamicCsvInfo.file.write(outString, names.size() - skip);
    }
    dynamicCsvInfo.file.write("\n");
}

void ExporterCsv::_initRawMeas(DynamicCsvInfo& dynamicCsvInfo, const uint8_t measGroupNum, const uint8_t measTypeNum)
{
    dynamicCsvInfo.file.write(getMeasurementNames(measGroupNum, measTypeNum));
    dynamicCsvInfo.file.write("\n");
}

void ExporterCsv::_writeTimestamp(OutputFile& file, time_point timestamp)
{
    char buffer[20] = {0};
    const Time time{timestamp};
    const int len = time.print(buffer, 20);
    file.write(buffer, len);
}

OutputFile& ExporterCsv::_getFileHandle(const Packet* p)
{
    Filesystem::FilePath fileName;
    if (p->details.syncByte == PacketDetails::SyncByte::Ascii)
    {
        for (auto& tmp : _csvInfo)
        {
            if (tmp.details.syncByte == PacketDetails::SyncByte::Ascii && p->details.asciiMetadata.header == tmp.details.asciiMetadata.header)
            {
                return tmp.file;
            }
        }
        fileName.assign("%s%s.csv", _filePath.c_str(), p->details.asciiMetadata.header.c_str());
    }
    else
    {
        for (auto& tmp : _csvInfo)
        {
            if (tmp.details.syncByte == PacketDetails::SyncByte::FA && p->details.faMetadata.header == tmp.details.faMetadata.header) { return tmp.file; }
        }
        fileName.assign("%sFA%s.csv", _filePath.c_str(), binaryHeaderToString<64>(p->details.faMetadata.header).c_str());
        std::replace(fileName.begin(), fileName.end(), ',', '_');
    }

    if (_csvInfo.push_back(CsvInfo{p->details, OutputFile(fileName)})) { VN_ABORT(); }
    if (!_csvInfo.back().file.is_open()) { VN_ABORT(); }

    _initCsv(_csvInfo.back(), p);

    return _csvInfo.back().file;
}

OutputFile& ExporterCsv::_getDynamicFileHandle(const uint8_t measGroupNum, const uint8_t measTypeNum, const BinaryHeader& header)
{
    for (auto& tmp : _dynamicCsvInfo)
    {
        if (tmp.measGroupNum == measGroupNum && tmp.measTypeNum == measTypeNum) { return tmp.file; }
    }

    Filesystem::FilePath fileName;
    const uint8_t gnss_num = getGnssGroup(measGroupNum);

    if (isSatField(measGroupNum, measTypeNum))
    {
        fileName.assign("%sFA%s_SatInfo%hu.csv", _filePath.c_str(), binaryHeaderToString<64>(header).c_str(), gnss_num);
    }
    else if (isRawField(measGroupNum, measTypeNum))
    {
        fileName.assign("%sFA%s_RawMeas%hu.csv", _filePath.c_str(), binaryHeaderToString<64>(header).c_str(), gnss_num);
    }
    else { VN_ABORT(); }
    std::replace(fileName.begin(), fileName.end(), ',', '_');

    if (_dynamicCsvInfo.push_back(DynamicCsvInfo{measGroupNum, measTypeNum, OutputFile(fileName)})) { VN_ABORT(); }
    if (!_dynamicCsvInfo.back().file.is_open()) { VN_ABORT(); }

    _initDynamicCsv(_dynamicCsvInfo.back(), measGroupNum, measTypeNum);
    return _dynamicCsvInfo.back().file;
}

void ExporterCsv::_flushAllFiles()
{
    for (auto& csvInfo : _csvInfo) { csvInfo.file.flush(); }
    for (auto& dynamicCsvInfo : _dynamicCsvInfo) { dynamicCsvInfo.file.flush(); }
}

}  // namespace DataExport
}  // namespace VN
