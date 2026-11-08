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

#ifndef VN_STRING_HPP_
#define VN_STRING_HPP_

#include <stdint.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cstdarg>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

#include "vectornav/Debug.hpp"
#include "vectornav/TemplateLibrary/Vector.hpp"

namespace VN
{

template <uint16_t Capacity>
class String : public Vector<char, (size_t)Capacity>
{
    static_assert(Capacity < uint16_t(-1));

public:
    String() = default;

    String(const char* inputString, uint16_t length = Capacity + 1)
    {
        const uint16_t copyLength = std::min(Capacity, length);
#ifdef __GNUC__
#pragma GCC diagnostic push
#ifndef __clang__
#pragma GCC diagnostic ignored "-Wstringop-truncation"
#endif
#endif
        strncpy(&this->_array[0], inputString, copyLength);
        this->_size = strnlen(&this->_array[0], Capacity);
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
    }

    String(const char* begin, const char* end) : String(begin, std::distance(begin, end)) {}

    String(const std::string& inputString) : String(inputString.c_str(), inputString.size() + 1) {}

    String(const std::string_view inputString) : String(inputString.data(), inputString.size() + 1) {}

    String(const char& inputChar)
    {
        this->_array[0] = inputChar;
        this->_size = 1;
    }

    template <uint16_t CapacityR>
    String(const String<CapacityR>& other)
    {
        const uint16_t copyLength = std::min(Capacity, other.length());
        memcpy(&this->_array[0], &other.front(), copyLength);  // Using memcpy because gcc was pushing a truncation warning
        this->_array[copyLength] = '\0';                       // Explicitly set null termination
        this->_size = copyLength;
    }

    const char* c_str() const noexcept { return this->data(); }
    uint16_t length() const noexcept { return static_cast<uint16_t>(this->size()); }

    void set_len() noexcept { this->_size = strnlen(this->data(), Capacity); }
    void set_len(uint16_t len) noexcept { this->_size = len; }

    String<Capacity>& assign(const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        int ret = vsnprintf(&this->_array[0], Capacity + 1, format, args);
        va_end(args);
        if (ret >= 0)
        {
            if (ret <= Capacity) { this->_size = ret; }
            else { this->_size = Capacity; }
        }
        else { this->clear(); }
        return *this;
    }

    String<Capacity>& append(const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        int ret = vsnprintf(&this->_array[this->_size], (Capacity + 1) - this->_size, format, args);
        va_end(args);
        if (ret >= 0)
        {
            size_t newSize = this->_size + static_cast<size_t>(ret);
            if (newSize <= Capacity) { this->_size = newSize; }
            else { this->_size = Capacity; }
        }
        else { this->_array[this->_size] = '\0'; }
        return *this;
    }

    const String<Capacity> substr(const uint16_t index, const uint16_t len) const noexcept
    {
        if (index > Capacity) { return String<Capacity>(); }
        else { return String<Capacity>(&this->_array[index], len); }
    }

    const String<Capacity> substr(const uint16_t index) const noexcept
    {
        if (index > Capacity) { return String<Capacity>(); }
        else { return String<Capacity>(&this->_array[index]); }
    }

    uint16_t find(const String<Capacity>& substr, uint16_t startIndex = 0) const noexcept
    {
        uint16_t substrLength = substr.length();
        if (substrLength > length()) { return String<Capacity>::npos; }
        if (startIndex > (length() - substrLength)) { return String<Capacity>::npos; }

        for (uint16_t i = startIndex; i <= (length() - substrLength); i++)
        {
            uint16_t j = 0;
            for (j = 0; j < substrLength; j++)
            {
                if (this->_array[i + j] != substr[j]) { break; }
            }
            if (j == substrLength) { return i; }
        }
        return String<Capacity>::npos;
    }

    uint16_t find(const char& substr, uint16_t startIndex = 0) const noexcept
    {
        if (startIndex > length()) { return String<Capacity>::npos; }

        for (uint16_t j = startIndex; j < length(); j++)
        {
            if (this->_array[j] == substr) return j;
        }
        return String<Capacity>::npos;
    }

    uint16_t rfind(const char* substr, const uint16_t startIndex, const uint16_t substrLength) const noexcept
    {
        if (substrLength > length()) { return String<Capacity>::npos; }
        for (uint16_t i = (std::min(static_cast<uint16_t>(length() - substrLength), startIndex)); i != std::numeric_limits<uint16_t>::max(); i--)
        {
            uint16_t j = 0;
            for (j = 0; j < substrLength; j++)
            {
                if (this->_array[i + j] != substr[j]) { break; }
            }
            if (j == substrLength) { return (i == std::numeric_limits<uint16_t>::max()) ? 0 : i; }
            if (i == 0) { return String<Capacity>::npos; }
        }
        return String<Capacity>::npos;
    }

    uint16_t rfind(const String<Capacity>& substr, uint16_t startIndex = Capacity) const noexcept { return rfind(substr.c_str(), startIndex, substr.length()); }

    std::string to_string() const noexcept { return std::string(this->data(), length()); }

    operator std::string() const noexcept { return to_string(); }
    operator std::string_view() const noexcept { return std::string_view(this->data(), length()); }

    inline static const uint16_t npos = std::numeric_limits<uint16_t>::max();
};

// ******************
// Addition operators
// ******************

// string + const char
template <uint16_t Capacity>
String<Capacity> operator+(String<Capacity> lhs, const char rhs) noexcept
{
    lhs.push_back(rhs);
    return lhs;
}

// Inversion of above
template <uint16_t Capacity>
String<Capacity> operator+(const char lhs, const String<Capacity>& rhs) noexcept
{
    return String<Capacity>(lhs) + rhs;
}

// string + array
template <uint16_t CapacityL, uint16_t CapacityR>
String<CapacityL> operator+(String<CapacityL> lhs, const char (&rhs)[CapacityR]) noexcept
{
    const uint16_t rhsLen = strnlen(rhs, CapacityR);
    const uint16_t rem = CapacityL - lhs.length();
    const uint16_t num = (rem >= rhsLen) ? rhsLen : rem;
    std::copy_n(rhs, num, lhs.end());
    lhs.set_len(lhs.length() + num);
    return lhs;
}

// Inversion of above
template <uint16_t CapacityL, uint16_t CapacityR>
String<CapacityR> operator+(const char (&lhs)[CapacityL], const String<CapacityR>& rhs) noexcept
{
    return String<CapacityR>(lhs) + rhs;
}

// string + std::string
template <uint16_t Capacity>
String<Capacity> operator+(String<Capacity> lhs, const std::string& rhs) noexcept
{
    return lhs + String<Capacity>(rhs);
}

// Inversion of above
template <uint16_t Capacity>
String<Capacity> operator+(const std::string& lhs, const String<Capacity>& rhs) noexcept
{
    return String<Capacity>(lhs) + rhs;
}

// string + string
template <uint16_t CapacityL, uint16_t CapacityR>
String<CapacityL> operator+(String<CapacityL> lhs, const String<CapacityR>& rhs) noexcept
{
    const uint16_t rhsLen = rhs.length();
    const uint16_t rem = CapacityL - lhs.length();
    const uint16_t num = (rem >= rhsLen) ? rhsLen : rem;
    std::copy_n(rhs.begin(), num, lhs.end());
    lhs.set_len(lhs.length() + num);
    return lhs;
}

// ******************
// Equality operators
// ******************

template <uint16_t CapacityL, uint16_t CapacityR>
bool operator!=(const String<CapacityL>& lhs, const char (&rhs)[CapacityR]) noexcept
{
    return !std::equal(lhs.begin(), lhs.end(), rhs, rhs + strnlen(rhs, CapacityR));
}

template <uint16_t CapacityL, uint16_t CapacityR>
inline bool operator!=(const String<CapacityL>& lhs, const String<CapacityR>& rhs) noexcept
{
    return !std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
}

template <uint16_t CapacityL, uint16_t CapacityR>
inline bool operator==(const String<CapacityL>& lhs, const String<CapacityR>& rhs) noexcept
{
    return std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
}

template <uint16_t CapacityL, uint16_t CapacityR>
bool operator==(const char (&lhs)[CapacityR], const String<CapacityL>& rhs) noexcept
{
    return std::equal(rhs.begin(), rhs.end(), lhs, lhs + strnlen(lhs, CapacityR));
}

template <uint16_t CapacityL, uint16_t CapacityR>
bool operator==(const String<CapacityL>& lhs, const char (&rhs)[CapacityR]) noexcept
{
    return std::equal(lhs.begin(), lhs.end(), rhs, rhs + strnlen(rhs, CapacityR));
}

template <uint16_t Capacity>
bool operator==(const std::string& lhs, const String<Capacity>& rhs) noexcept
{
    return std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
}

template <uint16_t Capacity>
bool operator==(const String<Capacity>& lhs, const std::string& rhs) noexcept
{
    return std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
}
/** \endcond */

namespace StringUtils
{
// ###################
// Numeric Conversions
// ###################

template <class T>
std::optional<T> fromString(const char* begin, const char* end)
{
    static_assert(std::is_arithmetic<T>::value, "Template parameter T must be a numeric type");
    T numReturn;
    if (*begin == '+') { begin++; }
    auto [ptr, ec] = std::from_chars(begin, end, numReturn);
    if (ptr != end || ec != std::errc{}) { return std::nullopt; }
    return std::make_optional(numReturn);
}

#if (defined(__clang__) && __clang_major < 16) || defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ < 11)
template <>
inline std::optional<double> fromString(const char* begin, const char* end)
{
    char* endPtr;
    errno = 0;
    const double numReturn = strtod(begin, &endPtr);
    if (endPtr != end || errno == ERANGE) { return std::nullopt; }
    return std::make_optional(numReturn);
}

template <>
inline std::optional<float> fromString(const char* begin, const char* end)
{
    char* endPtr;
    errno = 0;
    const float numReturn = strtof(begin, &endPtr);
    if (endPtr != end || errno == ERANGE) { return std::nullopt; }
    return std::make_optional(numReturn);
}
#endif

template <class T>
std::optional<T> fromStringHex(const char* begin, const char* end)
{
    static_assert(std::is_unsigned<T>::value, "Template parameter must be an unsigned integer type");

    T numReturn;
    if ((*(begin + 1) == 'x') || (*(begin + 1) == 'X')) { begin += 2; }
    auto [ptr, ec] = std::from_chars(begin, end, numReturn, 16);
    if (ptr != end || ec != std::errc{}) { return std::nullopt; }
    return std::make_optional(numReturn);
}

template <class T>
std::optional<T> fromStringHex(const std::string_view str)
{
    return StringUtils::fromStringHex<T>(str.data(), str.data() + str.size());
}

template <class T, uint16_t Capacity>
std::optional<T> fromString(const String<Capacity>& str, uint16_t offset = 0)
{
    return fromString<T>(&str.front() + offset, &str.back() + 1);
}

template <class T, uint16_t Capacity>
std::optional<T> fromStringHex(const String<Capacity>& str, uint16_t offset = 0)
{
    return fromStringHex<T>(&str.front() + offset, &str.back() + 1);
}

// ############
// Manipulation
// ############

constexpr std::uint16_t stringSplitMaximumSize = 42;

template <uint16_t Capacity>
bool startsWith(const String<Capacity>& strIn, const String<Capacity>& pattern) noexcept
{
    return strIn.rfind(pattern, 0) == 0;
}

template <uint16_t Capacity>
bool startsWith(const String<Capacity>& strIn, const char* pattern) noexcept
{
    return strIn.rfind(pattern, 0, std::strlen(pattern)) == 0;
}

inline std::string_view extractAfter(const std::string_view strIn, const char delimiter) noexcept
{
    const uint16_t idx = strIn.find(delimiter);
    if (idx >= strIn.size()) { return std::string_view(""); }
    return strIn.substr(idx + 1);
}

inline std::string_view extractAfter(const std::string_view strIn, const std::string_view delimiter) noexcept
{
    const uint16_t idx = strIn.find(delimiter);
    if (idx >= strIn.size()) { return std::string_view(""); }
    return strIn.substr(idx + delimiter.size());
}

inline std::string_view extractBetween(const std::string_view strIn, const std::string_view start, const std::string_view end) noexcept
{
    uint16_t startIndex = strIn.find(start);
    if (startIndex >= strIn.size()) { return ""; }
    startIndex += start.length();
    const uint16_t endIndex = strIn.find(end, startIndex);
    if (endIndex >= strIn.size()) { return ""; }
    return strIn.substr(startIndex, endIndex - startIndex);
}

inline std::string_view extractBetween(const std::string_view strIn, const char start, const char end) noexcept
{
    uint16_t startIndex = strIn.find(start) + 1;
    if (startIndex >= strIn.size()) { return std::string_view(""); }
    uint16_t endIndex = strIn.find(end, startIndex);
    if (endIndex >= strIn.size()) { return std::string_view(""); }
    return strIn.substr(startIndex, endIndex - startIndex - 1);
}

template <uint16_t Capacity>
Vector<String<Capacity>, stringSplitMaximumSize> split(const String<Capacity>& strIn, const char delimiter) noexcept
{
    Vector<String<Capacity>, stringSplitMaximumSize> splitStr;
    String<Capacity> buildingStr;
    for (const auto currChar : strIn)
    {
        if (currChar == delimiter)
        {
            splitStr.push_back(buildingStr);
            buildingStr.clear();
        }
        else { buildingStr = buildingStr + currChar; }
    }
    splitStr.push_back(buildingStr);
    return splitStr;
}

template <size_t Capacity>
Vector<String<Capacity>, stringSplitMaximumSize> split(const char* strIn, const char delimiter) noexcept
{
    Vector<String<Capacity>, stringSplitMaximumSize> splitStr;
    String<Capacity> buildingStr;
    for (size_t i = 0; strIn[i] != '\0'; ++i)
    {
        char currChar = strIn[i];
        if (currChar == delimiter)
        {
            splitStr.push_back(buildingStr);
            buildingStr.clear();
        }
        else { buildingStr = buildingStr + currChar; }
    }
    splitStr.push_back(buildingStr);
    return splitStr;
}
}  // namespace StringUtils

using LongString = String<256>;
using ShortString = String<32>;

}  // namespace VN

/** \cond */
template <uint16_t Capacity>
struct std::hash<VN::String<Capacity>>
{
    uint16_t operator()(const VN::String<Capacity>& str) const noexcept
    {
        uint16_t hash = 0;
        for (uint16_t i = 0; i < str.length(); ++i) { hash ^= static_cast<uint16_t>(str[i]) + 0x9e3779b9 + (hash << 6) + (hash >> 2); }
        return hash;
    }
};
/** \endcond */
#endif  // VN_STRING_HPP_
