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

#ifndef VN_COMMANDS_HPP_
#define VN_COMMANDS_HPP_

#include <cstdint>
#include <cstdio>

#include "vectornav/Implementation/CoreUtils.hpp"
#include "vectornav/Interface/GenericCommand.hpp"

namespace VN
{

class WriteSettings : public GenericCommand
{
public:
    WriteSettings() : GenericCommand{"WNV"} {};
};

class RestoreFactorySettings : public GenericCommand
{
public:
    RestoreFactorySettings() : GenericCommand{"RFS"} {};
};

class Reset : public GenericCommand
{
public:
    Reset() : GenericCommand{"RST"} {};
};

class FirmwareUpdate : public GenericCommand
{
public:
    FirmwareUpdate() : GenericCommand("FWU") {};
};

class KnownMagneticDisturbance : public GenericCommand
{
public:
    enum class State : uint8_t
    {
        NotPresent = 0,
        Present = 1
    } state;

    KnownMagneticDisturbance(const State state) : GenericCommand{"KMD"}, state(state) {};

    AsciiMessage getCommandString() noexcept override
    {
        _commandString.assign("KMD,%d", static_cast<uint8_t>(state));
        return _commandString;
    };
};

class KnownAccelerationDisturbance : public GenericCommand
{
public:
    enum class State : uint8_t
    {
        NotPresent = 0,
        Present = 1
    } state;

    KnownAccelerationDisturbance(const State state) : GenericCommand{"KAD"}, state(state) {};

    AsciiMessage getCommandString() noexcept override
    {
        _commandString.assign("KAD,%d", static_cast<uint8_t>(state));
        return _commandString;
    };
};

class SetInitialHeading : public GenericCommand
{
public:
    SetInitialHeading(const float heading) : GenericCommand{"SIH"} { params.push_back(heading); };

    SetInitialHeading(const Ypr ypr) : GenericCommand{"SIH"}
    {
        params.push_back(ypr.yaw);
        params.push_back(ypr.pitch);
        params.push_back(ypr.roll);
    };

    SetInitialHeading(const Quat quat) : GenericCommand{"SIH"}
    {
        params.push_back(quat.vector[0]);
        params.push_back(quat.vector[1]);
        params.push_back(quat.vector[2]);
        params.push_back(quat.scalar);
    };

    Vector<float, 4> params;
    AsciiMessage getCommandString() noexcept override
    {
        _commandString.assign("SIH");

        for (const auto& param : params)
        {
            // Print however many params exist
            _commandString.append(",%+08.3f", param);
        }

        return _commandString;
    };
};

class AsyncOutputEnable : public GenericCommand
{
public:
    enum class State : uint8_t
    {
        Disable = 0,
        Enable = 1
    } state;

    AsyncOutputEnable(const State state) : GenericCommand{"ASY"}, state(state) {};

    AsciiMessage getCommandString() noexcept override
    {
        _commandString.assign("ASY,%d", static_cast<uint8_t>(state));
        return _commandString;
    };
};

class SetFilterBias : public GenericCommand
{
public:
    SetFilterBias() : GenericCommand("SFB") {};
};

class PollBinaryOutputMessage : public GenericCommand
{
public:
    PollBinaryOutputMessage(const uint8_t binMsgNum) : GenericCommand{"BOM"}, binMsgNum(binMsgNum) {};
    uint8_t binMsgNum;
    AsciiMessage getCommandString() noexcept override
    {
        _commandString.assign("BOM,%02d", binMsgNum);
        return _commandString;
    };
};

class SetBootLoader : public GenericCommand
{
public:
    enum class Processor : uint8_t
    {
        Nav = 0,
        Gnss = 1,
        Imu = 2,
        Poll = static_cast<uint8_t>('?')
    } processorId;

    SetBootLoader(const Processor processorId) : GenericCommand{"SBL"}, processorId(processorId) {};

    AsciiMessage getCommandString() noexcept override
    {
        if (processorId == Processor::Poll) { _commandString.assign("SBL,?"); }
        else { _commandString.assign("SBL,%d", static_cast<uint8_t>(processorId)); }
        return _commandString;
    }

    Errored isMatchingResponse(const AsciiMessage& responseToCheck, time_point timestamp) noexcept override
    {
        LockGuard lock(_mutex);
        _awaitingResponse = false;
        _responseMatched = false;
        AsciiMessage stringToMatch{};
        if (processorId == Processor::Poll) { stringToMatch.assign("$VNSBL,"); }
        else { stringToMatch.assign("$VN%s", _commandString.c_str()); }
        if (StringUtils::startsWith(responseToCheck, stringToMatch))
        {
            _responseMatched = true;

            // Necessary because polling is an option, where we would like to populate the processorId with the response.
            auto u8_maybe = StringUtils::fromString<uint8_t>(responseToCheck.data() + 7, responseToCheck.data() + 8);
            if (!u8_maybe.has_value()) { return false; }             // This was invalid somehow.
            processorId = static_cast<Processor>(u8_maybe.value());  // SBL only has 1 digit processor ids
        }
        else
        {
            if (isMatchingError(responseToCheck)) { _responseMatched = true; }
            else { VN_DEBUG_1("response NOT matched.\n Expected response:\t" + stringToMatch + "\nReceived response:\t" + responseToCheck); }
        }
        if (_responseMatched)
        {
            _commandString = responseToCheck;
            _responseTime = timestamp;
        }
        return _responseMatched;
    }
};

/// @brief This is the base class used by all register definitions. Its derived classes are directly used by the user.
class Register
{
public:
    Register(uint8_t id) : _id(id) {};

    virtual ~Register() {};

    /// @brief Generates a Read Register command from this object.
    virtual GenericCommand toReadCommand()
    {
        AsciiMessage responseMatch;

        responseMatch.assign("RRG,%02d", _id);

        return GenericCommand(responseMatch, (_id > 99) ? 7 : 6);
    }
    /// @brief Populates the contents of this register from a command.
    /// @return An error occurred.
    Errored fromCommand(GenericCommand& commandIn) { return fromString(commandIn.getResponse()); }

    /// @brief Populates the contents of this register from a string.
    /// @return An error occurred.
    virtual Errored fromString(const AsciiMessage& sensorResponse) = 0;

    /// @brief Formats the string payload necessary for a read or write register command.
    AsciiMessage toString()
    {
        AsciiMessage ret;
        ret.assign("%02u", _id);
        return ret;
    }

    /// @brief Gets the register id.
    uint8_t id() const { return _id; }

protected:
    uint8_t _id;
};

class MeasurementRegister : public Register
{
public:
    using Register::Register;

protected:
    using Register::_id;
};

/// @brief Base class inherited by all configuration registers.
class ConfigurationRegister : public Register
{
public:
    using Register::Register;

    virtual AsciiMessage toString() const = 0;

    /// @brief Generates a Write Register command from this object.  Can fail if not all fields are set.
    std::optional<GenericCommand> toWriteCommand()
    {
        AsciiMessage responseMatch;
        String cmdStr = toString();
        if (cmdStr.empty() && _id != 0) { return std::nullopt; }
        responseMatch.assign("WRG,%02d,%s", _id, cmdStr.c_str());
        return std::make_optional<GenericCommand>(GenericCommand{responseMatch, static_cast<uint8_t>((_id > 99) ? 7 : 6)});
    }

protected:
    using Register::_id;
};

/// @brief Custom configuration register class.
class CustomConfigurationRegister : public ConfigurationRegister
{
public:
    CustomConfigurationRegister(uint8_t registerId) : ConfigurationRegister(registerId) {};

    /// @brief Formats the string payload necessary for a read or write register command.
    AsciiMessage toString() const override
    {
        AsciiMessage result = "";

        if (_params.empty()) { return result; }

        bool first = true;
        for (const auto& p : _params)
        {
            if (!p.has_value()) { return result; }

            if (!first) { result.append(","); }
            first = false;

            result.append("%s", p.value().data());
        }
        return result;
    }

    /// @brief Populates the contents of this register from a string.
    /// @return An error occurred.
    Errored fromString(const AsciiMessage& sensorResponse) override
    {
        const auto tokens = findIndexOfFieldSeparators(sensorResponse);

        int index = 0;
        const char* start;
        const char* end;

        while (!_params.empty()) { _params.pop_back(); }

        for (uint8_t i = 0; i < tokens.size() - 1; ++i)
        {
            start = tokens[index++] + 1;
            end = tokens[index];
            _params.push_back(AsciiParameter(start, end));
        }
        return false;
    };

    /// @brief Set the parameter of a custom configuration register. Senosr::readRegister must be called first to populate _params
    /// @return An error occurred.
    Errored setParam(const uint8_t ind, const AsciiParameter& value)
    {
        if (_params.empty() || !_params[ind].has_value())
        {
            VN_DEBUG_1("Error: Register " << std::to_string(id()) << " has not been read yet\n");
            return true;
        }
        if (ind >= _params.size())
        {
            VN_DEBUG_1("Error: Register " << std::to_string(id()) << "only has " << std::to_string(_params.size()) << " fields\n");
            return true;
        }
        _params[ind] = value;
        return false;
    }

    /// @brief Set all the parameters of a custom configuration registers. Senosr::readRegister must be called first to populate _params
    /// @return An error occurred.
    Errored setParams(const Vector<AsciiParameter, Config::PacketFinders::asciiMaxFieldCount>& vec)
    {
        if (vec.size() != _params.size())
        {
            VN_DEBUG_1("Error: Register " << std::to_string(id()) << "only has " << std::to_string(_params.size()) << " fields\n");
            return true;
        }
        for (uint8_t i = 0; i < _params.size(); ++i)
        {
            if (setParam(i, vec[i])) { return true; }
        }
        return false;
    }

    /// @brief Get number of params from custom register.
    uint8_t numParams() { return _params.size(); }

protected:
    using Register::_id;
    Vector<std::optional<AsciiParameter>, Config::PacketFinders::asciiMaxFieldCount> _params;
};

}  // namespace VN

#endif  // VN_COMMANDS_HPP_
