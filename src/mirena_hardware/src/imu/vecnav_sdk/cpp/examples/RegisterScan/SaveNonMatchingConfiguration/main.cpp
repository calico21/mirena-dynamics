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

#include <filesystem>
#include <string>

#include "vectornav/HAL/File.hpp"
#include "vectornav/Interface/Registers.hpp"
#include "vectornav/Interface/Sensor.hpp"
#include "vectornav/Interface/Stream.hpp"
#include "vectornav/RegisterScan.hpp"

using namespace VN;

std::string usage = "[port] [inputFilePath or \"generic\"] [outputFilePath or \"generic\"]";

enum class ReadWriteType
{
    Xml,
    Ascii,
    Generic
};

int main(int argc, char* argv[])
{
    /* This save configuration example walks through the C++ usage of the SDK to save the configuration on
    the VectorNav sensor that differs from the configuration specified. It save the configuration as an XML file, ASCII file,
    or generic user-defined lambda expression

    This example will achieve the following:

    1. Handle input arguments
    2. Instantiate a Sensor object and use it to connect to the VectorNav unit
    3. Configure initial settings onto the sensor
    4. Save all non-matching values from the VectorNav unit as specified
    5. Disconnect from the VectorNav unit
    */

    // 1. Handle input arguments
    if (argc > 4) { std::cerr << "Usage: SaveNonMatchingConfiguration " << usage << std::endl; };
    const std::string portName = (argc > 1) ? argv[1] : "COM1";  // Pass in port name as a positional argument, or edit here
    std::filesystem::path inputFilePath = (argc > 2) ? argv[2] : (std::filesystem::path(__FILE__).parent_path() / "configSettingsIn.xml");
    std::filesystem::path outputFilePath = (argc > 3) ? argv[3] : (std::filesystem::path(__FILE__).parent_path() / "configSettingsOut.xml");
    // std::filesystem::path filePath = "generic";

    ReadWriteType rwType{ReadWriteType::Generic};
    if (inputFilePath.string() == std::string("generic"))
    {
        if (outputFilePath.string() != std::string("generic"))
        {
            std::cerr << "Error: Output file type does not match input file type." << std::endl;
            return 1;
        }
        rwType = ReadWriteType::Generic;
    }
    else
    {
        if (!std::filesystem::exists(inputFilePath))
        {
            std::cerr << "Error: File does not exist: " << inputFilePath.string() << std::endl;
            return 1;
        }
        std::filesystem::path inputExtension = inputFilePath.extension();
        std::filesystem::path outputExtension = outputFilePath.extension();
        if (inputExtension.empty() || outputExtension.empty())
        {
            std::cerr << "Error: One of the file path extensions was not specified." << std::endl;
            return 1;
        }
        else if (inputExtension == ".xml")
        {
            if (outputExtension != ".xml")
            {
                std::cerr << "Error: Output file type does not match input file type." << std::endl;
                return 1;
            }
            rwType = ReadWriteType::Xml;
        }
        else
        {
            if (outputExtension != ".txt")
            {
                std::cerr << "Error: Output file type does not match input file type." << std::endl;
                return 1;
            }
            rwType = ReadWriteType::Ascii;
        }
    }

    // 2. Instantiate a Sensor object and use it to connect to the VectorNav unit
    VN::Sensor sensor;
    Error latestError = sensor.autoConnect(portName);
    if (latestError != Error::None)
    {
        std::cerr << "Error " << latestError << " encountered when connecting to " + portName << std::endl;
        return static_cast<int>(latestError);
    }
    std::cout << "Connected to " << portName << " at " << sensor.connectedBaudRate().value() << std::endl;

    // 3. Configure initial settings onto the sensor
    latestError = sensor.restoreFactorySettings();
    if (latestError != Error::None)
    {
        std::cerr << latestError << " encountered when restoring factory settings.";
        return static_cast<int>(latestError);
    }

    Registers::System::AsyncOutputType asyncDataOutputType;
    asyncDataOutputType.ador = Registers::System::AsyncOutputType::Ador::YPR;
    asyncDataOutputType.serialPort = Registers::System::AsyncOutputType::SerialPort::Serial1;
    latestError = sensor.writeRegister(&asyncDataOutputType);
    if (latestError != Error::None)
    {
        std::cerr << latestError << " encountered when configuring register " << std::to_string(asyncDataOutputType.id());
        return static_cast<int>(latestError);
    }

    Registers::IMU::DeltaThetaVelConfig deltaThetaVelConfig;
    deltaThetaVelConfig.integrationFrame = Registers::IMU::DeltaThetaVelConfig::IntegrationFrame::NED;
    deltaThetaVelConfig.gyroCompensation = Registers::IMU::DeltaThetaVelConfig::GyroCompensation::Bias;
    deltaThetaVelConfig.accelCompensation = Registers::IMU::DeltaThetaVelConfig::AccelCompensation::Gravity;
    deltaThetaVelConfig.earthRateCompensation = Registers::IMU::DeltaThetaVelConfig::EarthRateCompensation::None;
    deltaThetaVelConfig.resv = 0;
    latestError = sensor.writeRegister(&deltaThetaVelConfig);
    if (latestError != Error::None)
    {
        std::cerr << latestError << " encountered when configuring register " << std::to_string(deltaThetaVelConfig.id());
        return static_cast<int>(latestError);
    }
    std::cout << "Configured sensor initial settings." << std::endl;

    // 4. Save all non-matching values from the VectorNav unit as specified
    VN::Error saveConfigError;
    switch (rwType)
    {
        case (ReadWriteType::Xml):
        {
            RegisterScan::XmlConfigReader configReader(inputFilePath.string());
            RegisterScan::XmlConfigWriter configWriter(sensor, outputFilePath.string());
            saveConfigError = RegisterScan::saveNonMatchingConfiguration(sensor, configWriter, configReader);
        }
        break;
        case (ReadWriteType::Ascii):
        {
            RegisterScan::AsciiConfigReader configReader(inputFilePath.string());
            RegisterScan::AsciiConfigWriter configWriter(outputFilePath.string());
            saveConfigError = RegisterScan::saveNonMatchingConfiguration(sensor, configWriter, configReader);
        }
        break;
        case (ReadWriteType::Generic):
        {
            uint16_t messageCount{0};
            std::vector<AsciiMessage> messages{"$VNRRG,5,921600,2*30E1", "$VNRRG,06,2,1*5135", "$VNRRG,07,40,1*78B2", "$VNRRG,82,1,1,1,0,0*2D73"};
            RegisterScan::GenericConfigReader configReader(
                [&messages, &messageCount](AsciiMessage& msg)
                {
                    if (messageCount < messages.size())
                    {
                        msg = messages[messageCount];
                        messageCount++;
                        return Error::None;
                    }
                    else { return Error::FileReadFailed; }
                });
            RegisterScan::GenericConfigWriter configWriter(
                [](const AsciiMessage& msg)
                {
                    std::cout << msg.c_str();
                    return Error::None;
                });
            saveConfigError = RegisterScan::saveNonMatchingConfiguration(sensor, configWriter, configReader);
        }
        break;
        default:
            VN_ABORT();
    }

    if (saveConfigError != Error::None)
    {
        std::cerr << "Error " << saveConfigError << " occured when saving configuration." << std::endl;
        return 1;
    }

    // 5. Disconnect from the VectorNav unit
    sensor.disconnect();
    std::cout << "SaveNonMatchingConfiguration example complete." << std::endl;
}
