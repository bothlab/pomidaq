/*
 * Copyright (C) 2019-2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the license, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "utils.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <format>
#include <system_error>

namespace Miniscope::Utils
{

/*
 * String helpers
 */

std::string fmtDouble(double value)
{
    return std::format("{:g}", value);
}

std::string asciiToLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

std::string stringTrimmed(const std::string &s)
{
    const auto isSpace = [](unsigned char c) {
        return std::isspace(c) != 0;
    };
    auto start = s.begin();
    while (start != s.end() && isSpace(*start))
        ++start;
    auto end = s.end();
    while (end != start && isSpace(*(end - 1)))
        --end;
    return std::string(start, end);
}

size_t fileSuffixLength(const std::string &fname)
{
    const auto dotPos = fname.rfind('.');
    if (dotPos == std::string::npos)
        return fname.length();
    return fname.length() - dotPos - 1;
}

bool parseUInt(std::string_view s, int base, unsigned int &result)
{
    if (s.empty())
        return false;
    const auto res = std::from_chars(s.data(), s.data() + s.size(), result, base);
    return res.ec == std::errc() && res.ptr == s.data() + s.size();
}

/*
 * JSON helpers
 */

int jsonInt(const json &v, int defaultValue)
{
    if (v.is_number_integer())
        return v.get<int>();
    if (v.is_number_float()) {
        const auto d = v.get<double>();
        if (d == std::floor(d))
            return static_cast<int>(d);
    }
    return defaultValue;
}

double jsonDouble(const json &v, double defaultValue)
{
    if (v.is_number())
        return v.get<double>();
    return defaultValue;
}

bool jsonBool(const json &v, bool defaultValue)
{
    if (v.is_boolean())
        return v.get<bool>();
    return defaultValue;
}

std::string jsonString(const json &v, const std::string &defaultValue)
{
    if (v.is_string())
        return v.get<std::string>();
    return defaultValue;
}

const json &jsonMember(const json &obj, const char *key)
{
    static const json nullValue;
    const auto it = obj.find(key);
    if (it == obj.end())
        return nullValue;
    return *it;
}

json jsonArray(const json &v)
{
    if (v.is_array())
        return v;
    return json::array();
}

json jsonObject(const json &v)
{
    if (v.is_object())
        return v;
    return json::object();
}

} // namespace Miniscope::Utils
