/*
 * Copyright (C) 2025-2026 Matthias Klumpp <matthias@tenstral.net>
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

// Internal logging helpers for use only within libminiscope.
// Not part of the public API.

#pragma once

#include <atomic>
#include <format>
#include <string>

#include "logging.h"

namespace Miniscope
{

/**
 * @brief A named log category with a per-instance severity threshold.
 */
struct LogCategory {
    const char *name;
    std::atomic<int> threshold; /// Minimum LogSeverity (as int) to dispatch
    LogCategory *next = nullptr;

    explicit LogCategory(const char *name, LogSeverity defaultSeverity = LogSeverity::Info) noexcept;
};

/// Head of the global category linked list
extern std::atomic<LogCategory *> g_categoryListHead;

/// Set to true whenever a non-empty handler is installed
extern std::atomic<bool> g_handlerActive;

inline bool shouldLog(const LogCategory &c, LogSeverity s) noexcept
{
    return g_handlerActive.load(std::memory_order_acquire) && int(s) >= c.threshold.load(std::memory_order_relaxed);
}

void dispatchLog(const LogCategory &cat, LogSeverity sev, const std::string &message);

} // namespace Miniscope

/**
 * Define a file-local log category.
 *
 * The category self-registers in the global list at static-init time.
 */
#define MS_DEFINE_LOG_CATEGORY(varname, catname) static ::Miniscope::LogCategory varname(catname)

/**
 * Forward-declare a log category for use from a header.
 */
#define MS_DECLARE_LOG_CATEGORY(varname) extern ::Miniscope::LogCategory varname

#define MS_LOG(cat, sev, ...)                                                 \
    do {                                                                      \
        if (::Miniscope::shouldLog((cat), (sev)))                             \
            ::Miniscope::dispatchLog((cat), (sev), std::format(__VA_ARGS__)); \
    } while (0)

#define MS_LOG_DEBUG(cat, ...)    MS_LOG((cat), ::Miniscope::LogSeverity::Debug, __VA_ARGS__)
#define MS_LOG_INFO(cat, ...)     MS_LOG((cat), ::Miniscope::LogSeverity::Info, __VA_ARGS__)
#define MS_LOG_WARNING(cat, ...)  MS_LOG((cat), ::Miniscope::LogSeverity::Warning, __VA_ARGS__)
#define MS_LOG_ERROR(cat, ...)    MS_LOG((cat), ::Miniscope::LogSeverity::Error, __VA_ARGS__)
#define MS_LOG_CRITICAL(cat, ...) MS_LOG((cat), ::Miniscope::LogSeverity::Critical, __VA_ARGS__)
