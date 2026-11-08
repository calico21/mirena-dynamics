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

#include "vectornav/ExporterAscii.hpp"

#include "vectornav/HAL/File_Base.hpp"
#include "vectornav/Implementation/MeasurementDatatypes.hpp"
#include "vectornav/TemplateLibrary/String.hpp"

namespace VN
{
namespace DataExport
{

void ExporterAscii::exportToFile()
{
    while (!_queue.isEmpty())
    {
        const auto p = _queue.get();
        if (!p || (p->details.syncByte != PacketDetails::SyncByte::Ascii)) { continue; }

        if (_asciiInfo.size() == _asciiInfo.capacity())
        {
            VN_DEBUG_1("Packet dropped.");
            continue;
        }

        OutputFile& ascii = _getFileHandle(_resolveAsciiHeader(p.get()));

        if (_enableSystemTimeStamps)
        {
            char _buffer[20] = {0};
            Time timestamp{p->details.asciiMetadata.timestamp};
            const int len = timestamp.print(_buffer, 20);
            ascii.write(_buffer, len);
        }

        ascii.write(reinterpret_cast<const char*>(p->buffer), p->details.asciiMetadata.length);
    }
    _flushAllFiles();
}

ShortString ExporterAscii::_resolveAsciiHeader(const Packet* p) const
{
    const auto& meta = p->details.asciiMetadata;
    const auto& header = meta.header;

    if (_grouping.groupAll) { return "AllAscii"; }
    if (_grouping.groupNmea && StringUtils::startsWith(header, "GP")) { return "NMEA"; }
    if (_grouping.groupVnCommands && meta.isCommand) { return "VnCommands"; }
    if (_grouping.groupVnMeasurements && !meta.isCommand && StringUtils::startsWith(header, "VN")) { return "VnMeasurements"; }

    return header;
}

void ExporterAscii::_flushAllFiles()
{
    for (auto& asciiInfo : _asciiInfo) { asciiInfo.file.flush(); }
}

OutputFile& ExporterAscii::_getFileHandle(const ShortString& header)
{
    for (auto& tmp : _asciiInfo)
    {
        if (tmp.header == header) { return tmp.file; }
    }

    const Filesystem::FilePath fileName = _filePath + header + ".txt";

    if (_asciiInfo.push_back(AsciiInfo{header, OutputFile(fileName)})) { VN_ABORT(); }
    if (!_asciiInfo.back().file.is_open()) { VN_ABORT(); }

    return _asciiInfo.back().file;
}

}  // namespace DataExport
}  // namespace VN
