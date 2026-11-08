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

#ifndef VN_EXPORTERSKIPPEDBYTES_HPP_
#define VN_EXPORTERSKIPPEDBYTES_HPP_

#include <filesystem>

#include "vectornav/Exporter.hpp"
#include "vectornav/HAL/File.hpp"

namespace VN
{
namespace DataExport
{

class ExporterSkippedByte : public Exporter
{
public:
    ExporterSkippedByte(const Filesystem::FilePath& outputDir, PacketQueueMode mode = PacketQueueMode::Force)
        : Exporter(EXPORTER_PACKET_CAPACITY, mode), _filePath(outputDir)
    {
        if (!_filePath.empty() && _filePath.back() != std::filesystem::path::preferred_separator)
        {
            _filePath = _filePath + std::filesystem::path::preferred_separator;
        }
        if (_initFile()) { VN_ABORT(); }
    }

    ~ExporterSkippedByte() = default;

    void exportToFile() override;

private:
    static constexpr uint16_t EXPORTER_PACKET_CAPACITY = 2048;

    Errored _initFile();

    Filesystem::FilePath _filePath;
    OutputFile _file;
};

}  // namespace DataExport
}  // namespace VN

#endif  // VN_EXPORTERSKIPPEDBYTES_HPP_
