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

// Internal helper functions for use only within libminiscope.
// Not part of the public API.

#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <nlohmann/json.hpp>

namespace Miniscope::Utils
{

using json = nlohmann::json;

/*
 * String helpers
 */

/**
 * @brief Format double into a string ('g' format, precision 6)
 */
std::string fmtDouble(double value);

/**
 * @brief Convert ASCII characters of a string to lowercase.
 */
std::string asciiToLower(std::string s);

/**
 * @brief Remove leading and trailing whitespace from a string.
 */
std::string stringTrimmed(const std::string &s);

/**
 * @brief Length of the file suffix (characters after the last dot) of a filename.
 *
 * If the filename contains no dot, the length of the whole name is returned.
 */
size_t fileSuffixLength(const std::string &fname);

/**
 * @brief Parse an unsigned integer with the given base, like QString::toUInt() would.
 *
 * The whole string must be consumed for the parse to succeed.
 * @return True on success, with the parsed value stored in @p result.
 */
bool parseUInt(std::string_view s, int base, unsigned int &result);

/**
 * @brief Return the value of a key in a map, or a default if the key does not exist.
 */
template<typename Map>
typename Map::mapped_type mapValueOr(
    const Map &map,
    const typename Map::key_type &key,
    const typename Map::mapped_type &defaultValue = typename Map::mapped_type())
{
    const auto it = map.find(key);
    if (it == map.end())
        return defaultValue;
    return it->second;
}

/*
 * JSON helpers.
 */

/**
 * @brief Get an integer from a JSON value.
 *
 * Floating-point values are only accepted if they are integral.
 */
int jsonInt(const json &v, int defaultValue = 0);

double jsonDouble(const json &v, double defaultValue = 0);

bool jsonBool(const json &v, bool defaultValue = false);

std::string jsonString(const json &v, const std::string &defaultValue = std::string());

/**
 * @brief Get a member of a JSON object, or a null value if it does not exist.
 */
const json &jsonMember(const json &obj, const char *key);

/**
 * @brief Return the value if it is an array, or an empty array otherwise.
 */
json jsonArray(const json &v);

/**
 * @brief Return the value if it is an object, or an empty object otherwise.
 */
json jsonObject(const json &v);

} // namespace Miniscope::Utils
