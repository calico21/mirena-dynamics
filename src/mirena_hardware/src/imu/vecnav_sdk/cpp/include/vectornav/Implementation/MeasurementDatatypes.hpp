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

#ifndef VN_MEASUREMENTDATATYPES_HPP_
#define VN_MEASUREMENTDATATYPES_HPP_

#include <array>
#include <cstddef>  // For size_t
#include <cstdint>
#include <cstdio>
#include <type_traits>

#include "vectornav/Debug.hpp"
#include "vectornav/HAL/Timer.hpp"
#include "vectornav/TemplateLibrary/Matrix.hpp"
#include "vectornav/TemplateLibrary/String.hpp"
#include "vectornav/TemplateLibrary/Vector.hpp"

namespace VN
{

// There is one array element for each measurement group (Common is not included as it's duplicate), which each contain a bit for each measurement type
// The standard data passed around to represent which measurement types are avaialable.
using EnabledMeasurements = std::array<uint32_t, 19>;

// SFINAE trait to detect types with a print(char*, uint16_t) method
namespace Detail
{
template <typename T, typename = void>
struct HasBufferPrint : std::false_type
{
};

template <typename T>
struct HasBufferPrint<T, std::void_t<decltype(std::declval<const T&>().print(std::declval<char*>(), std::declval<uint16_t>()))>> : std::true_type
{
};
}  // namespace Detail

template <typename T, std::enable_if_t<Detail::HasBufferPrint<T>::value, int> = 0>
inline int measPrint(char* buf, uint16_t len, const T& meas)
{
    return meas.print(buf, len);
}
inline int measPrint(char* buf, uint16_t len, const uint8_t& meas) { return std::snprintf(buf, len, "%hu", static_cast<unsigned short>(meas)); }
inline int measPrint(char* buf, uint16_t len, const uint16_t& meas) { return std::snprintf(buf, len, "%hu", static_cast<unsigned short>(meas)); }
inline int measPrint(char* buf, uint16_t len, const uint32_t& meas) { return std::snprintf(buf, len, "%u", static_cast<unsigned int>(meas)); }
inline int measPrint(char* buf, uint16_t len, const float& meas) { return std::snprintf(buf, len, "%.6g", meas); }
inline int measPrint(char* buf, uint16_t len, const double& meas) { return std::snprintf(buf, len, "%.12g", meas); }
inline int measPrint(char* buf, uint16_t len, const bool& meas) { return std::snprintf(buf, len, "%d", static_cast<int>(meas)); }

inline bool allDataIsEnabled(const EnabledMeasurements& measurementsToCheck, const EnabledMeasurements& availableMeasurements) noexcept
{
    size_t numAvailableGroups = availableMeasurements.size();
    bool allAreEnabled = true;
    size_t idx = 0;
    for (const auto binGroup : measurementsToCheck)
    {
        allAreEnabled &=
            ((idx < numAvailableGroups) && ((binGroup & ~availableMeasurements[idx]) == 0));  // don't error if less than num so that it goes into unparsed
        ++idx;
    }
    return allAreEnabled;
}

inline bool anyDataIsEnabled(const EnabledMeasurements& measurementsToCheck, const EnabledMeasurements& availableMeasurements) noexcept
{
    size_t idx = 0;
    for (const auto binGroup : measurementsToCheck)
    {
        if ((binGroup & availableMeasurements[idx]) != 0) { return true; }
        ++idx;
    }
    return false;
}

inline EnabledMeasurements unionOf(const EnabledMeasurements& measurements1, const EnabledMeasurements& measurements2) noexcept
{
    EnabledMeasurements retVal{};
    uint8_t groupIdx = 0;
    for (auto& retValGroup : retVal)
    {
        retValGroup = measurements1[groupIdx] | measurements2[groupIdx];
        groupIdx++;
    }
    return retVal;
}

inline EnabledMeasurements intersectionOf(const EnabledMeasurements& measurements1, const EnabledMeasurements& measurements2) noexcept
{
    EnabledMeasurements retVal{};
    uint8_t groupIdx = 0;
    for (auto& retValGroup : retVal)
    {
        retValGroup = measurements1[groupIdx] & measurements2[groupIdx];
        groupIdx++;
    }
    return retVal;
}

struct InsStatus
{
    uint16_t mode : 2;
    uint16_t gnssFix : 1;
    uint16_t resv1 : 1;
    uint16_t imuErr : 1;
    uint16_t magPresErr : 1;
    uint16_t gnssErr : 1;
    uint16_t resv2 : 1;
    uint16_t gnssCompassFix : 2;
    uint16_t : 6;  // padding

    InsStatus() = default;
    InsStatus(uint16_t other) noexcept { std::memcpy(this, &other, sizeof(InsStatus)); }
    explicit operator uint16_t() const
    {
        uint16_t result;
        std::memcpy(&result, this, sizeof(InsStatus));
        return result;
    }
    InsStatus& operator=(const uint16_t& other)
    {
        std::memcpy(this, &other, sizeof(InsStatus));
        return *this;
    }

    int print(char* buf, uint16_t len) const noexcept { return std::snprintf(buf, len, "%04X", static_cast<uint16_t>(*this)); }
};

static_assert(sizeof(InsStatus) == 2);

inline bool operator==(const InsStatus& lhs, const InsStatus& rhs) { return static_cast<uint16_t>(lhs) == static_cast<uint16_t>(rhs); }
inline bool operator!=(const InsStatus& lhs, const InsStatus& rhs) { return !(lhs == rhs); }

struct TimeStatus
{
    struct
    {
        uint8_t towValid : 1;
        uint8_t dateValid : 1;
        uint8_t utcValid : 1;
        uint8_t : 5;  // padding
    };
    TimeStatus() = default;
    TimeStatus(const uint8_t& other) { std::memcpy(this, &other, sizeof(TimeStatus)); }
    explicit operator uint8_t() const
    {
        uint8_t result;
        std::memcpy(&result, this, sizeof(TimeStatus));
        return result;
    }
    TimeStatus& operator=(const uint8_t& other)
    {
        std::memcpy(this, &other, sizeof(TimeStatus));
        return *this;
    }

    int print(char* buf, uint16_t len) const noexcept { return std::snprintf(buf, len, "%02X", static_cast<uint8_t>(*this)); }
};

static_assert(sizeof(TimeStatus) == 1);

struct AhrsStatus
{
    struct
    {
        uint16_t attitudeQuality : 2;
        uint16_t gyroSaturation : 1;
        uint16_t gyroSaturationRecovery : 1;
        uint16_t magDisturbance : 2;
        uint16_t magSaturation : 1;
        uint16_t accDisturbance : 2;
        uint16_t accSaturation : 1;
        uint16_t resv1 : 1;
        uint16_t knownMagDisturbance : 1;
        uint16_t knownAccDisturbance : 1;
        uint16_t resv2 : 1;
        uint16_t : 2;  // padding
    };
    AhrsStatus() = default;
    AhrsStatus(const uint16_t& other) { std::memcpy(this, &other, sizeof(AhrsStatus)); }
    explicit operator uint16_t() const
    {
        uint16_t result;
        std::memcpy(&result, this, sizeof(AhrsStatus));
        return result;
    }
    AhrsStatus& operator=(const uint16_t& other)
    {
        std::memcpy(this, &other, sizeof(AhrsStatus));
        return *this;
    }

    int print(char* buf, uint16_t len) const noexcept { return std::snprintf(buf, len, "%04X", static_cast<uint16_t>(*this)); }
};

static_assert(sizeof(AhrsStatus) == 2);

struct ImuStatus
{
    struct
    {
        uint16_t gyroStatus : 2;
        uint16_t accelStatus : 2;
        uint16_t magStatus : 2;
        uint16_t presTempStatus : 2;
        uint16_t resv : 2;
        uint16_t : 6;  // padding
    };
    ImuStatus() = default;
    ImuStatus(const uint16_t& other) { std::memcpy(this, &other, sizeof(ImuStatus)); }
    explicit operator uint16_t() const
    {
        uint16_t result;
        std::memcpy(&result, this, sizeof(ImuStatus));
        return result;
    }
    ImuStatus& operator=(const uint16_t& other)
    {
        std::memcpy(this, &other, sizeof(ImuStatus));
        return *this;
    }

    int print(char* buf, uint16_t len) const noexcept { return std::snprintf(buf, len, "%04X", static_cast<uint16_t>(*this)); }
};

static_assert(sizeof(ImuStatus) == 2);

struct SensSat
{
    uint16_t magX : 1;
    uint16_t magY : 1;
    uint16_t magZ : 1;
    uint16_t accelX : 1;
    uint16_t accelY : 1;
    uint16_t accelZ : 1;
    uint16_t gyroX : 1;
    uint16_t gyroY : 1;
    uint16_t gyroZ : 1;
    uint16_t pres : 1;
    uint16_t : 6;  // padding

    SensSat() = default;
    SensSat(const uint16_t& other) noexcept { std::memcpy(this, &other, sizeof(SensSat)); }
    explicit operator uint16_t() const
    {
        uint16_t result;
        std::memcpy(&result, this, sizeof(SensSat));
        return result;
    }
    SensSat& operator=(const uint16_t& other)
    {
        std::memcpy(this, &other, sizeof(SensSat));
        return *this;
    }

    int print(char* buf, uint16_t len) const noexcept { return std::snprintf(buf, len, "%04X", static_cast<uint16_t>(*this)); }
};

static_assert(sizeof(SensSat) == 2);

struct GnssStatus
{
    struct
    {
        uint16_t enabled : 1;
        uint16_t operational : 1;
        uint16_t fix : 1;
        uint16_t antennaSignalError : 1;
        uint16_t usedForNav : 1;
        uint16_t resv1 : 3;
        uint16_t dataSource : 3;
        uint16_t usedForNavCurr : 1;
        uint16_t ppsUsedForTime : 1;
        uint16_t : 3;  // padding
    };
    GnssStatus() = default;
    GnssStatus(const uint16_t& other) { std::memcpy(this, &other, sizeof(GnssStatus)); }
    explicit operator uint16_t() const
    {
        uint16_t result;
        std::memcpy(&result, this, sizeof(GnssStatus));
        return result;
    }
    GnssStatus& operator=(const uint16_t& other)
    {
        std::memcpy(this, &other, sizeof(GnssStatus));
        return *this;
    }

    int print(char* buf, uint16_t len) const noexcept { return std::snprintf(buf, len, "%04X", static_cast<uint16_t>(*this)); }
};

static_assert(sizeof(GnssStatus) == 2);

struct Time
{
public:
    Time() {}
    explicit Time(const uint64_t time) : _time(time) {}
    explicit Time(const double time) : _time(time * 1e9) {}
    explicit Time(const time_point time) : _time(static_cast<uint64_t>(std::chrono::duration_cast<Nanoseconds>(time.time_since_epoch()).count())) {}

    uint64_t nanoseconds() const { return _time; }
    uint64_t microseconds() const { return _time / static_cast<uint64_t>(1e3); }
    uint64_t milliseconds() const { return _time / static_cast<uint64_t>(1e6); }
    double seconds() const { return static_cast<double>(_time) / 1e9; }

    int print(char* buf, uint16_t len) const noexcept;

private:
    uint64_t _time = 0;
};

inline int Time::print(char* buf, uint16_t len) const noexcept { return std::snprintf(buf, len, "%.6f", seconds()); }

struct Ypr
{
    Ypr() {}
    Ypr(const Vec3f& vec) : yaw(vec[0]), pitch(vec[1]), roll(vec[2]) {}
    Ypr(float yaw, float pitch, float roll) : yaw(yaw), pitch(pitch), roll(roll) {}
    float yaw = 0;
    float pitch = 0;
    float roll = 0;

    int print(char* buf, uint16_t len) const noexcept;

    const float& operator()(uint16_t idx) const
    {
        VN_ASSERT(idx < 3);
        switch (idx)
        {
            case 0:
                return yaw;
            case 1:
                return pitch;
            case 2:
                return roll;
            default:
                VN_ABORT();
        }
    }
};

inline int Ypr::print(char* buf, uint16_t len) const noexcept { return std::snprintf(buf, len, "%f,%f,%f", yaw, pitch, roll); }

struct DeltaTheta
{
    DeltaTheta() {}
    DeltaTheta(const float& deltaTime, const Vec3f& deltaTheta) : deltaTime(deltaTime), deltaTheta(deltaTheta) {}
    DeltaTheta(const float& dt, const float& dTx, const float& dTy, const float& dTz) : deltaTime(dt), deltaTheta{dTx, dTy, dTz} {}

    float deltaTime = 0;
    Vec3f deltaTheta = 0;

    int print(char* buf, uint16_t len) const noexcept;

    const float& operator()(uint16_t idx) const
    {
        VN_ASSERT(idx < 4);
        if (idx == 0) { return deltaTime; }
        else { return deltaTheta(idx - 1); }
    }
};

inline int DeltaTheta::print(char* buf, uint16_t len) const noexcept
{
    return std::snprintf(buf, len, "%f,%f,%f,%f", deltaTime, deltaTheta[0], deltaTheta[1], deltaTheta[2]);
}

struct Quat
{
    Quat() {}
    Quat(const Vec3f& vector, const float& scalar) : vector(vector), scalar(scalar) {}
    Quat(const Vec4f& v) : vector{v(0), v(1), v(2)}, scalar(v(3)) {}
    Quat(const float& vx, const float& vy, const float& vz, const float& s) : vector{vx, vy, vz}, scalar(s) {}
    Vec3f vector = 0;
    float scalar = 1;

    int print(char* buf, uint16_t len) const noexcept;

    const float& operator()(uint16_t idx) const
    {
        VN_ASSERT(idx < 4);
        if (idx == 3) { return scalar; }
        else { return vector(idx); }
    }
};

inline int Quat::print(char* buf, uint16_t len) const noexcept { return std::snprintf(buf, len, "%f,%f,%f,%f", vector[0], vector[1], vector[2], scalar); }

struct Lla
{
    Lla() {}
    Lla(const Vec3d& lla) : lat(lla[0]), lon(lla[1]), alt(lla[2]) {}
    Lla(double lat, double lon, double alt) : lat(lat), lon(lon), alt(alt) {}
    double lat = 0;
    double lon = 0;
    double alt = 0;

    int print(char* buf, uint16_t len) const noexcept;

    const double& operator()(uint16_t idx) const
    {
        VN_ASSERT(idx < 3);
        switch (idx)
        {
            case 0:
                return lat;
            case 1:
                return lon;
            case 2:
                return alt;
            default:
                VN_ABORT();
        }
    }
};

inline int Lla::print(char* buf, uint16_t len) const noexcept { return std::snprintf(buf, len, "%f,%f,%f", lat, lon, alt); }

struct TimeUtc
{
    int8_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint16_t fracSec = 0;

    TimeUtc() = default;

    TimeUtc(int8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint16_t fracsec)
        : year(year), month(month), day(day), hour(hour), minute(minute), second(second), fracSec(fracsec) {};

    int print(char* buf, uint16_t len) const noexcept;
};

inline int TimeUtc::print(char* buf, uint16_t len) const noexcept
{
    return std::snprintf(buf, len, "%d,%u,%u,%u,%u,%u,%u", year, month, day, hour, minute, second, fracSec);
}

struct GnssTimeInfo
{
    uint8_t gnssTimeStatus = 0;
    int8_t leapSeconds = 0;

    GnssTimeInfo() = default;

    GnssTimeInfo(uint8_t gnssTimeStatus, int8_t leapSeconds) : gnssTimeStatus(gnssTimeStatus), leapSeconds(leapSeconds) {};

    int print(char* buf, uint16_t len) const noexcept;
};

inline int GnssTimeInfo::print(char* buf, uint16_t len) const noexcept { return std::snprintf(buf, len, "%u,%d", gnssTimeStatus, leapSeconds); }

struct GnssDop
{
    float gDop = 0.0;
    float pDop = 0.0;
    float tDop = 0.0;
    float vDop = 0.0;
    float hDop = 0.0;
    float nDop = 0.0;
    float eDop = 0.0;

    GnssDop() = default;

    GnssDop(std::array<float, 7> arrIn) : gDop(arrIn[0]), pDop(arrIn[1]), tDop(arrIn[2]), vDop(arrIn[3]), hDop(arrIn[4]), nDop(arrIn[5]), eDop(arrIn[6]) {};

    int print(char* buf, uint16_t len) const noexcept;
};

inline int GnssDop::print(char* buf, uint16_t len) const noexcept
{
    return std::snprintf(buf, len, "%f,%f,%f,%f,%f,%f,%f", gDop, pDop, tDop, vDop, hDop, nDop, eDop);
}

constexpr uint8_t GNSS_SAT_INFO_MAX_COUNT = 35;

struct GnssSatInfo
{
    uint8_t numSats = 0;
    uint8_t resv = 0;
    std::array<uint8_t, GNSS_SAT_INFO_MAX_COUNT> sys = {0};
    std::array<uint8_t, GNSS_SAT_INFO_MAX_COUNT> svId = {0};
    std::array<uint8_t, GNSS_SAT_INFO_MAX_COUNT> flags = {0};
    std::array<uint8_t, GNSS_SAT_INFO_MAX_COUNT> cno = {0};
    std::array<uint8_t, GNSS_SAT_INFO_MAX_COUNT> qi = {0};
    std::array<int8_t, GNSS_SAT_INFO_MAX_COUNT> el = {0};
    std::array<int16_t, GNSS_SAT_INFO_MAX_COUNT> az = {0};

    GnssSatInfo() = default;

    GnssSatInfo(uint8_t numSats, uint8_t resv, const std::array<uint8_t, GNSS_SAT_INFO_MAX_COUNT>& sys,
                const std::array<uint8_t, GNSS_SAT_INFO_MAX_COUNT>& svId, const std::array<uint8_t, GNSS_SAT_INFO_MAX_COUNT>& flags,
                const std::array<uint8_t, GNSS_SAT_INFO_MAX_COUNT>& cno, const std::array<uint8_t, GNSS_SAT_INFO_MAX_COUNT>& qi,
                const std::array<int8_t, GNSS_SAT_INFO_MAX_COUNT>& el, const std::array<int16_t, GNSS_SAT_INFO_MAX_COUNT>& az)
        : numSats(numSats), resv(resv), sys(sys), svId(svId), flags(flags), cno(cno), qi(qi), el(el), az(az) {};

    int print(char* buf, uint16_t len) const noexcept;
};

inline int GnssSatInfo::print(char* buf, uint16_t len) const noexcept
{
    int nprint = 0;

    int ret = std::snprintf(buf, len, "%u", numSats);
    if (ret < 0 || ret >= len) { return ret; }
    nprint += ret;

    for (size_t i = 0; i < GNSS_SAT_INFO_MAX_COUNT; ++i)
    {
        ret = std::snprintf(buf + nprint, len - nprint, ",%u,%u,%u,%u,%u,%d,%d", sys[i], svId[i], flags[i], cno[i], qi[i], el[i], az[i]);
        if (ret < 0) { return ret; }
        else if (ret + nprint >= len) { return ret + nprint; }
        nprint += ret;
    }
    buf[nprint++] = '\n';
    return nprint;
}

constexpr uint8_t GNSS_RAW_MEAS_MAX_COUNT = 55;

struct GnssRawMeas
{
    double tow = 0.0;
    uint16_t week = 0;
    uint8_t numMeas = 0;
    uint8_t resv = 0;
    std::array<uint8_t, GNSS_RAW_MEAS_MAX_COUNT> sys = {0};
    std::array<uint8_t, GNSS_RAW_MEAS_MAX_COUNT> svId = {0};
    std::array<uint8_t, GNSS_RAW_MEAS_MAX_COUNT> band = {0};
    std::array<uint8_t, GNSS_RAW_MEAS_MAX_COUNT> chan = {0};
    std::array<int8_t, GNSS_RAW_MEAS_MAX_COUNT> freqNum = {0};
    std::array<uint8_t, GNSS_RAW_MEAS_MAX_COUNT> cno = {0};
    std::array<uint16_t, GNSS_RAW_MEAS_MAX_COUNT> flags = {0};
    std::array<double, GNSS_RAW_MEAS_MAX_COUNT> pr = {0.0};
    std::array<double, GNSS_RAW_MEAS_MAX_COUNT> cp = {0.0};
    std::array<float, GNSS_RAW_MEAS_MAX_COUNT> dp = {0.0};

    GnssRawMeas() = default;

    GnssRawMeas(double tow, uint16_t week, uint8_t numMeas, uint8_t resv, const std::array<uint8_t, GNSS_RAW_MEAS_MAX_COUNT>& sys,
                const std::array<uint8_t, GNSS_RAW_MEAS_MAX_COUNT>& svId, const std::array<uint8_t, GNSS_RAW_MEAS_MAX_COUNT>& band,
                const std::array<uint8_t, GNSS_RAW_MEAS_MAX_COUNT>& chan, const std::array<int8_t, GNSS_RAW_MEAS_MAX_COUNT>& freqNum,
                const std::array<uint8_t, GNSS_RAW_MEAS_MAX_COUNT>& cno, const std::array<uint16_t, GNSS_RAW_MEAS_MAX_COUNT>& flags,
                const std::array<double, GNSS_RAW_MEAS_MAX_COUNT>& pr, const std::array<double, GNSS_RAW_MEAS_MAX_COUNT>& cp,
                const std::array<float, GNSS_RAW_MEAS_MAX_COUNT>& dp)
        : tow(tow),
          week(week),
          numMeas(numMeas),
          resv(resv),
          sys(sys),
          svId(svId),
          band(band),
          chan(chan),
          freqNum(freqNum),
          cno(cno),
          flags(flags),
          pr(pr),
          cp(cp),
          dp(dp) {};

    int print(char* buf, uint16_t len) const noexcept;
};

inline int GnssRawMeas::print(char* buf, uint16_t len) const noexcept
{
    int nprint = 0;
    int ret = 0;

    for (size_t i = 0; i < numMeas; ++i)
    {
        ret = std::snprintf(buf + nprint, len - nprint, "%12.8f,%u,%u,%u,%u,%u,%u,%d,%u,%u,%12.8f,%12.8f,%f\n", tow, week, numMeas, sys[i], svId[i], band[i],
                            chan[i], freqNum[i], cno[i], flags[i], pr[i], cp[i], dp[i]);

        if (ret < 0) { return ret; }
        else if (ret + nprint >= len) { return ret + nprint; }
        nprint += ret;
    }

    return nprint;
}

constexpr uint8_t binaryGroupMaxSize = 3;
constexpr uint8_t binaryTypeMaxSize = 10;
constexpr uint8_t binaryHeaderMaxLength = binaryGroupMaxSize + 2 * binaryTypeMaxSize;
}  // namespace VN

#endif  // VN_MEASUREMENTDATATYPES_HPP_
