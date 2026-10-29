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

#ifndef VN_EXPORTERIMR_HPP_
#define VN_EXPORTERIMR_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "vectornav/Config.hpp"
#include "vectornav/Debug.hpp"
#include "vectornav/Exporter.hpp"
#include "vectornav/HAL/File.hpp"

namespace VN
{
namespace DataExport
{

class ExporterImr : public Exporter
{
public:
    ExporterImr(const Filesystem::FilePath& fileName, PacketQueueMode mode = PacketQueueMode::Force)
        : Exporter(EXPORTER_PACKET_CAPACITY, mode), _fileName(fileName), _file(fileName + ".imr")
    {
        if (!_file.is_open()) { VN_ABORT(); }
        _file.write(reinterpret_cast<char*>(&_header), sizeof(_header));
    }

    ~ExporterImr() { _finalize(); }

    void exportToFile() override;

private:
    static constexpr uint16_t EXPORTER_PACKET_CAPACITY = 2048;
    static constexpr int scaleFactor = 1000000;
    static constexpr float RAD2DEG = 180 / 3.1415926535897931;

    void _finalize();

    double _prevTime = 0;
    float _avgDt = 0;
    size_t _numSamples = 1;
    Filesystem::FilePath _fileName;
    OutputFile _file;

// exact fit - no padding
#pragma pack(push, 1)
    struct ImrFileHeader
    {
        uint8_t header[8] = "$IMURAW";
        uint8_t isIntelOrMotorola = 0;
        double versionNumber = 8.9;
        int32_t deltaTheta = 0;
        int32_t deltaVelocity = 0;
        double dataRateHz = 0;
        double gyroScaleFactor = 0;
        double accelScaleFactor = 0;
        int32_t utcOrGpsTime = 2;
        int32_t rcvTimeOrCorrTime = 0;
        double timeTagBias = 0;
        uint8_t imuName[32] = "                               ";
        uint8_t reserved1[4] = {0};
        uint8_t programName[32] = "VectorNav File Export          ";
        uint8_t create[12] = {0};
        uint8_t levelArmValid = 0;
        int32_t IYoffset = 0;
        int32_t IXoffset = 0;
        int32_t IZoffset = 0;
        uint8_t reserved[354] = {0};

    } _header;
#pragma pack(pop)

#pragma pack(push, 1)  // exact fit - no padding
    struct ImrData
    {
        double gpsTow;
        int32_t uncompGyroY;
        int32_t uncompGyroX;
        int32_t uncompGyroZ;
        int32_t uncompAccelY;
        int32_t uncompAccelX;
        int32_t uncompAccelZ;
    } _data;
#pragma pack(pop)
};

}  // namespace DataExport
}  // namespace VN

#endif  // VN_EXPORTERIMR_HPP_
