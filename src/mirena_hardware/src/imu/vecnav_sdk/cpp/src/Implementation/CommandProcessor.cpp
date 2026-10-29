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

#include "vectornav/Implementation/CommandProcessor.hpp"

#include <cstdint>
#include <cstring>
#include <optional>

#include "vectornav/Debug.hpp"
#include "vectornav/HAL/Timer.hpp"
#include "vectornav/Implementation/CoreUtils.hpp"
#include "vectornav/Interface/GenericCommand.hpp"
#include "vectornav/TemplateLibrary/String.hpp"

namespace VN
{

Error CommandProcessor::registerCommand(GenericCommand* pCommand, const Microseconds timeoutThreshold) noexcept
{  // Should only be called on main thread
    LockGuard guard{_mutex};

    if (pCommand->isAwaitingResponse()) { return Error::CommandResent; }

    auto t = now();
    while (_clearStaleCommand(t));

    if (_cmdQueue.isFull()) { return Error::CommandQueueFull; }

    pCommand->prepareToSend();
    _cmdQueue.put({pCommand, timeoutThreshold});
    pCommand->setQueuePtr(_cmdQueue.peekBack());

    return Error::None;
}

Errored CommandProcessor::matchResponse(const AsciiMessage& response, const AsciiPacketProtocol::Metadata& metadata) noexcept
{  // Should be called on high-priority thread
    VN_DEBUG_1("RX: " + response + "\t queue size: " + std::to_string(queueSize()));
    if (StringUtils::startsWith(response, AsciiMessage("$VNERR,")) && !GenericCommand::isMatchingError(response))
    {
        // Is an async error or is ill-formed
        AsciiMessage errorNumString = StringUtils::extractBetween(response, ',', '*');
        const auto errorNum = StringUtils::fromStringHex<uint8_t>(errorNumString);
        if (errorNum.has_value()) { _asyncErrorQueuePush(AsyncError(static_cast<Error>(errorNum.value()), response, now())); }
        else { _asyncErrorQueuePush(AsyncError(Error::ReceivedUnexpectedMessage, response, now())); }
        return true;
    }
    else
    {
        LockGuard guard{_mutex};
        bool responseHasBeenMatched = false;
        while (!responseHasBeenMatched)
        {
            if (_clearStaleCommand(metadata.timestamp)) { continue; }
            auto frontCommand = _cmdQueue.get();
            if (!frontCommand.has_value()) { break; }
            frontCommand.value().cmd->setQueuePtr(nullptr);
            responseHasBeenMatched = (*frontCommand).cmd->isMatchingResponse(response, metadata.timestamp);
        }
        if (!responseHasBeenMatched)
        {
            _asyncErrorQueuePush(AsyncError(Error::ReceivedUnexpectedMessage, response, now()));
            return true;
        }
    }

    return false;
}

int CommandProcessor::queueSize() const noexcept
{
    LockGuard guard{_mutex};
    return _cmdQueue.size();
}

void CommandProcessor::popCommand(GenericCommand* cmd) noexcept
{
    LockGuard guard{_mutex};
    cmd->invalidateOnQueue();
    cmd->setQueuePtr(nullptr);
}

bool CommandProcessor::_clearStaleCommand(time_point timeRef) noexcept
{
    bool ret = false;
    const auto item = _cmdQueue.peek();
    if (!item.has_value()) { ret = false; }  // queue is empty
    else if (!item.value().cmd)
    {  // command has already been destructed, clear from queue
        _cmdQueue.get();
        ret = true;
    }
    else if ((timeRef - item.value().cmd->getSentTime()) > item.value().timeoutThreshold)
    {  // command has timed out, clear from queue
        _cmdQueue.get().value().cmd->setStale();
        ret = true;
    }
    else { ret = false; }

    return ret;
}

std::optional<CommandProcessor::QueueItem> CommandProcessor::getFrontCommand() noexcept
{
    LockGuard guard{_mutex};
    auto qItem = _cmdQueue.get();
    if (!qItem.has_value() || !qItem.value().cmd) { return std::nullopt; }
    else
    {
        qItem.value().cmd->setQueuePtr(nullptr);
        return qItem;
    }
}

}  // namespace VN
