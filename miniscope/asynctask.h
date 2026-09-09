/*
 * Copyright (C) 2020-2026 Matthias Klumpp <matthias@tenstral.net>
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

#pragma once

#include <functional>
#include <memory>
#include <string>

#ifdef _WIN32
#define MS_LIB_EXPORT __declspec(dllexport)
#else
#define MS_LIB_EXPORT __attribute__((visibility("default")))
#endif

namespace Miniscope
{

class TaskProgress;

/**
 * @brief Handle for a long-running operation executed in a background thread.
 *
 * The task is started immediately when the handle is created. Destroying the
 * handle waits for the task to complete.
 */
class MS_LIB_EXPORT AsyncTask
{
public:
    ~AsyncTask();
    AsyncTask(AsyncTask &&other) noexcept;
    AsyncTask &operator=(AsyncTask &&other) noexcept;
    AsyncTask(const AsyncTask &) = delete;
    AsyncTask &operator=(const AsyncTask &) = delete;

    /**
     * @brief Returns true as soon as the task has completed (successfully or not).
     */
    bool isFinished() const;

    /**
     * @brief Current progress of the task, in percent (0-100).
     */
    int progressValue() const;

    /**
     * @brief Optional human-readable description of the current task step.
     */
    std::string progressText() const;

    /**
     * @brief Block until the task has finished and return its result.
     *
     * If the task failed, the exception it raised (usually a std::runtime_error)
     * is rethrown here. May be called multiple times.
     */
    bool waitForFinished();

private:
    class Private;
    std::unique_ptr<Private> d;

    AsyncTask();
    friend class TaskProgress;
    friend AsyncTask launchAsyncTask(std::function<bool(TaskProgress &)> body);
};

} // namespace Miniscope
