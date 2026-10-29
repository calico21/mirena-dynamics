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

#ifndef VN_REGISTERSCAN_HPP_
#define VN_REGISTERSCAN_HPP_

#include <utility>

#include "vectornav/ConfigReader.hpp"
#include "vectornav/ConfigWriter.hpp"
#include "vectornav/Interface/Registers.hpp"

namespace VN
{

namespace RegisterScan
{

constexpr uint16_t NUM_REG = 256;

Error _setConfigurationRegister(Sensor& sensor, const AsciiMessage& msg);

template <typename T>
/**
 * @brief Applies multiple configuration settings to a VectorNav unit from a configuration reader.
 * @tparam T The specific type of configuration reader.
 * @param sensor The sensor object to configure.
 * @param configReader The configuration reader providing the settings.
 * @return Error code indicating success or failure of the operation.
 * @details Iteratively reads configuration messages from the configuration reader and applies each one
 *          to the sensor until no more configurations are available. This method only applies the
 *          configuration settings in the configuration reader such that any settings previously
 *          configured on the unit will persist.
 */
Error setConfigurationRegisters(Sensor& sensor, ConfigReader<T>& configReader)
{
    AsciiMessage msg;
    Error err{Error::None};
    bool readFlag = false;
    while (configReader.getNextConfig(msg) == Error::None)
    {
        readFlag = true;
        Error err = _setConfigurationRegister(sensor, msg);
        if (err != Error::None) { return err; };
    }
    if (readFlag == false) { err = Error::FileReadFailed; }
    return err;
}

template <typename T>
/**
 * @brief Loads a complete configuration onto a VectorNav sensor from a configuration reader.
 * @tparam T The specific type of configuration reader.
 * @param sensor The sensor object to configure.
 * @param configReader The configuration reader providing the settings.
 * @return Error code indicating success or failure of the operation.
 * @details Restores factory settings first, then applies all configurations from the configuration
 *          reader, and finally saves the settings and resets the device. If performing a restore
 *          factory settings is not desired, the RegisterScan::setConfigurationRegisters()
 *          method should instead be used to load the user-defined settings onto the unit.
 */
Error loadConfiguration(Sensor& sensor, ConfigReader<T>& configReader)
{
    Error error = sensor.restoreFactorySettings();
    if (error != Error::None) { return error; }
    Error err = setConfigurationRegisters(sensor, configReader);
    if (err != Error::None) return err;

    error = sensor.writeSettings();
    if (error != Error::None) { return error; }
    error = sensor.reset();
    if (error != Error::None) { return error; }

    return error;
}

/**
 * @struct SaveConfigurationFilter
 * @brief Defines which registers to include or exclude when saving a sensor configuration.
 * @details Contains a filter type (Include or Exclude) and a list of register IDs to filter.
 */
struct SaveConfigurationFilter
{
    /**
     * @enum Type
     * @brief Determines whether the list specifies registers to include or exclude.
     */
    enum class Type
    {
        Include,
        Exclude
    } type;
    Vector<uint8_t, NUM_REG> list;
};

Error _getRegisterList(SaveConfigurationFilter filter, Vector<uint8_t, NUM_REG>& reg_to_poll);

template <typename T>
Error _saveConfigurationRegister(Sensor& sensor, ConfigWriter<T>& configWriter, uint8_t regId)
{
    VN_DEBUG_0("Polling register " << std::to_string(regId) << "...\n");
    AsciiMessage msg;
    GenericCommand cmd;
    Error err{Error::None};

    if (!((regId == 5) || (regId == 6) || (regId == 7) || (regId == 99)))
    {
        if (regId == 250) { return Error::None; }  // invalid register

        AsciiMessage msg;
        msg.assign("RRG,%d", regId);
        cmd = GenericCommand(msg);
        err = sensor.sendCommand(&cmd, Sensor::SendCommandBlockMode::BlockWithRetry);
        if (err == Error::InvalidRegister || err == Error::UnauthorizedAccess || err == Error::NotEnoughParameters) { return Error::None; }
        else if (err != Error::None) { return err; }

        AsciiMessage tmp = cmd.getResponse();
        const auto start = tmp.find(',');
        const auto end = tmp.find('*');

        if (tmp.find(',', start + 1) == AsciiMessage::npos) { return Error::None; }  // catches responses with no arguments

        tmp.assign("WRG%.*s", int(end - start), &tmp[start]);
        auto wrg = GenericCommand(tmp);

        err = sensor.sendCommand(&wrg, Sensor::SendCommandBlockMode::BlockWithRetry);
        if (err == Error::InvalidRegister || err == Error::UnauthorizedAccess)
        {
            // Read-only reg. (Some registers incorrectly report InvalidRegister, i.e. VN-100 WRG 101)
            err = Error::None;
            return err;
        }
        else if (err != Error::None) { return err; }
        err = configWriter.writeConfig(cmd.getResponse());
        if (err != Error::None) { return err; }
    }
    else  // (regId == 5) || (regId == 6) || (regId == 7) || (regId == 99))
    {
        msg.assign("RRG,%02d,1", regId);
        cmd = GenericCommand(msg);
        err = sensor.sendCommand(&cmd, Sensor::SendCommandBlockMode::BlockWithRetry);
        if (regId == 99 && err == Error::None)
        {
            // Some firmwares do not correctly report TooManyParameters, so we have to manually check whether it echoed correctly to see if this firmware
            // supports the optional parameter. A standard "RRG,99" call has 9 commas, so if there are only 9 commas the ",1" was not postpended, and the
            // optional parameter was ignored.
            const auto rsp = cmd.getResponse();
            const uint16_t commaCount = std::count(rsp.begin(), rsp.end(), ',');
            if (commaCount == 9) { err = Error::TooManyParameters; }
        }
        if (err == Error::None)
        {
            err = configWriter.writeConfig(cmd.getResponse());
            if (err != Error::None) { return err; }
        }
        else if (err == Error::InvalidRegister)  // Unsupported reg
        {
            err = Error::None;
            return err;
        }
        else if (err == Error::TooManyParameters)
        {
            msg.assign("RRG,%02d", regId);
            cmd = GenericCommand(msg);
            err = sensor.sendCommand(&cmd, Sensor::SendCommandBlockMode::BlockWithRetry);
            if (err == Error::None) { err = configWriter.writeConfig(cmd.getResponse()); }
            return err;
        }
        else { return err; }
        for (uint16_t i = 2;; ++i)
        {
            msg.assign("RRG,%02d,%d", regId, int(i));
            cmd = GenericCommand(msg);
            err = sensor.sendCommand(&cmd, Sensor::SendCommandBlockMode::BlockWithRetry);
            if (err == Error::None)
            {
                err = configWriter.writeConfig(cmd.getResponse());
                if (err != Error::None) { return err; }
            }
            else if (err == Error::InvalidParameter)
            {
                err = Error::None;
                return err;
            }
            else { return err; }
        }
    }

    return err;
}

inline std::optional<uint8_t> _getRegId(const AsciiMessage& msg)
{
    const uint16_t start = msg.find(',', 0);
    const uint16_t end = msg.find(',', start + 1);
    if (start == AsciiMessage::npos || start >= msg.length() || end == AsciiMessage::npos || end >= msg.length()) { return std::nullopt; }
    return StringUtils::fromString<uint8_t>(&msg[start + 1], &msg[end]);
}

inline bool _isMsgEqual(const AsciiMessage& msg1, const AsciiMessage& msg2)
{
    // Only compare until the asterisk (crc might be different formats, and line endings change when reading from file
    // due to getLine)
    constexpr uint16_t msgHeaderLen = 6;  // initial "$VNRRG," part of message that can be ignored

    const auto msg1AsteriskPos = msg1.find('*', msgHeaderLen);
    if (msg1AsteriskPos == AsciiMessage::npos || msg1AsteriskPos >= msg1.length()) { return false; }  // failed to find asterisk

    const auto msg2AsteriskPos = msg2.find('*', msgHeaderLen);
    if (msg2AsteriskPos == AsciiMessage::npos || msg2AsteriskPos >= msg2.length()) { return false; }  // failed to find asterisk

    if (msg1AsteriskPos == msg2AsteriskPos && msg1.substr(msgHeaderLen, msg1AsteriskPos) == msg2.substr(msgHeaderLen, msg2AsteriskPos)) { return true; }
    else { return false; }
}

/**
 * @brief Gets the default list of configuration registers for VectorNav devices.
 * @return Vector<uint8_t, NUM_REG> A vector containing IDs of all known configuration registers.
 */
constexpr Vector<uint8_t, NUM_REG> getDefaultConfigRegisters()
{
    Vector<uint8_t, NUM_REG> retVal;
    for (auto i = 0; i < 255; ++i)
    {
        if (Registers::getConfigRegisterName(i)) { retVal.push_back(i); }
    }
    return retVal;
}

/**
 * @brief Gets the default list of measurement registers for VectorNav devices.
 * @return Vector<uint8_t, NUM_REG> A vector containing IDs of all known measurement registers.
 */
constexpr Vector<uint8_t, NUM_REG> getDefaultMeasRegisters()
{
    Vector<uint8_t, NUM_REG> retVal;
    for (auto i = 0; i < 255; ++i)
    {
        if (Registers::getMeasurementRegisterName(i)) { retVal.push_back(i); }
    }
    return retVal;
}

template <typename T>
/**
 * @brief Saves the current configuration of a VectorNav sensor using the provided configuration writer.
 * @tparam T The specific type of configuration writer.
 * @param sensor The sensor object to read from.
 * @param configWriter The configuration writer that handles the actual storage of configurations.
 * @param filter Optional filter specifying which registers to include or exclude.
 * @return Error code indicating success or failure of the operation.
 * @details Temporarily disables asynchronous output, reads each specified register,
 *          and passes its configuration to the provided writer. The writer determines how and where
 *          the configuration is actually stored.
 */
Error saveConfiguration(Sensor& sensor, ConfigWriter<T>& configWriter,
                        SaveConfigurationFilter filter = SaveConfigurationFilter{SaveConfigurationFilter::Type::Include, getDefaultConfigRegisters()})
{
    Error error{Error::None};
    Vector<uint8_t, NUM_REG> reg_to_poll;

    error = _getRegisterList(filter, reg_to_poll);

    error = sensor.asyncOutputEnable(AsyncOutputEnable::State::Disable);
    if (error != Error::None) { return error; }

    for (uint8_t regId : reg_to_poll)
    {
        error = _saveConfigurationRegister(sensor, configWriter, regId);
        if (error != Error::None) { return error; }
    }

    configWriter.close();
    error = sensor.asyncOutputEnable(AsyncOutputEnable::State::Enable);
    if (error != Error::None) { return error; }

    return error;
}

template <typename T>
/**
 * @brief Saves only the non-default configuration settings of a VectorNav sensor.
 * @tparam T The specific type of configuration writer.
 * @param sensor The sensor object to read from.
 * @param configWriter The configuration writer that handles the actual storage of configurations.
 * @return Error code indicating success or failure of the operation.
 * @details Saves the current configuration, restores factory defaults, then compares
 *          the two configurations to identify and save only the settings that differ
 *          from factory defaults. If performing a Restore Factory Settings command is
 *          not desired, the RegisterScan::saveConfiguration() method should instead be
 *          used to save the current configuration of a VectorNav unit.
 */
Error saveNonDefaultConfiguration(Sensor& sensor, ConfigWriter<T>& configWriter)
{
    const Filesystem::FilePath tmpFileName = "UserSettings.tmp.txt";
    AsciiConfigWriter tmpWriter(tmpFileName);
    VN_DEBUG_0("Collecting user settings...\n");
    Error error = saveConfiguration(sensor, tmpWriter);
    if (error != Error::None) { return error; }
    error = sensor.restoreFactorySettings();
    if (error != Error::None) { return error; }

    AsciiConfigReader tmpReader(tmpFileName);
    GenericConfigWriter writeNonDefault{[&tmpReader, &sensor, &configWriter](const AsciiMessage& defaultMsg)
                                        {
                                            AsciiMessage userMsg;
                                            Error err = tmpReader.next(userMsg);
                                            if (err != Error::None) { return err; }
                                            const bool msgEqual = _isMsgEqual(userMsg, defaultMsg);
                                            if (!msgEqual)
                                            {
                                                err = _setConfigurationRegister(sensor, userMsg);
                                                if (err != Error::None) { return err; }
                                                err = configWriter.writeConfig(userMsg);
                                                return err;
                                            }
                                            else { return err; }
                                        },
                                        [&configWriter]() { configWriter.close(); }};

    VN_DEBUG_0("Collecting default settings and writing diffs...\n");
    error = saveConfiguration(sensor, writeNonDefault);
    if (error != Error::None) { return error; }

    VN_DEBUG_0("Issuing Write Settings and Reset...\n");
    error = sensor.writeSettings();
    if (error != Error::None) { return error; }
    error = sensor.reset();
    if (error != Error::None) { return error; }
    return error;
}

template <typename T, typename S>
/**
 * @brief   Reads the registers listed in the configReader, and only writes to configWriter those that are different than
 *          what is on the sensor.  Returns Error::Unknown if there are no differences
 * @tparam T The specific type of configuration writer.
 * @tparam S The specific type of configuration reader.
 * @param sensor The sensor object to read from.
 * @param configWriter The configuration writer that handles the actual storage of configurations.
 * @param configReader The configuration reader providing the settings.
 * @return Error code indicating success or failure of the operation.
 */
Error saveNonMatchingConfiguration(Sensor& sensor, ConfigWriter<T>& configWriter, ConfigReader<S>& configReader)
{
    // Disable async output
    Error latestError = sensor.asyncOutputEnable(AsyncOutputEnable::State::Disable);
    if (latestError != Error::None) { return latestError; }

    // save non-matching registers to a file
    AsciiMessage userMsg;
    bool readFlag = false;
    bool writeFlag = false;
    while (configReader.getNextConfig(userMsg) == Error::None)
    {
        readFlag = true;

        // parse userMsg to determine the register ID
        std::optional<uint8_t> regId = _getRegId(userMsg);
        if (!regId.has_value()) { return Error::ReceivedUnexpectedMessage; }
        const uint8_t regIdInt = regId.value();

        // Get the sensor register msg
        AsciiMessage readRegistergMsg;
        const size_t commaCount = std::count(userMsg.begin(), userMsg.end(), ',');
        const auto lastCommaPos = userMsg.rfind(',');
        const auto userAsteriskPos = userMsg.find('*');
        ShortString lastParam = userMsg.substr(lastCommaPos + 1, userAsteriskPos - lastCommaPos - 1);
        if ((regIdInt == 5) || (regIdInt == 6) || (regIdInt == 7))
        {
            if (commaCount == 3) { readRegistergMsg.assign("RRG,%02d,%s", regIdInt, lastParam.c_str()); }
            else { readRegistergMsg.assign("RRG,%d", regIdInt); }
        }
        else if (regIdInt == 99)
        {
            if (commaCount == 10) { readRegistergMsg.assign("RRG,%02d,%s", regIdInt, lastParam.c_str()); }
            else { readRegistergMsg.assign("RRG,%d", regIdInt); }
        }
        else  // !((regIdInt == 5) || (regIdInt == 6) || (regIdInt == 7) || (regIdInt == 99))
        {
            readRegistergMsg.assign("RRG,%d", regIdInt);
        }

        GenericCommand readRegisterCmd = GenericCommand(readRegistergMsg);
        latestError = sensor.sendCommand(&readRegisterCmd, Sensor::SendCommandBlockMode::BlockWithRetry);
        if (latestError != Error::None) { return latestError; }
        AsciiMessage sensorMsg = readRegisterCmd.getResponse();

        // Parse and check message validity
        if (!_isMsgEqual(userMsg, sensorMsg))
        {
            writeFlag = true;
            latestError = configWriter.writeConfig(sensorMsg);
            if (latestError != Error::None) { return latestError; }
        }
    }

    configWriter.close();
    if (!readFlag) { return Error::FileReadFailed; }

    // reenable async output
    latestError = sensor.asyncOutputEnable(AsyncOutputEnable::State::Enable);
    if (latestError != Error::None) { return latestError; }
    if (!writeFlag) { return Error::Unknown; }

    return latestError;
}

}  // namespace RegisterScan
}  // namespace VN

#endif  // VN_REGISTERSCAN_HPP_
