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

#include <expected>
#include <functional>
#include <string>

#include "asynctask.h"

namespace Miniscope
{

/**
 * @brief Progress reporting interface.
 */
class TaskProgress
{
public:
    explicit TaskProgress(AsyncTask::Private *d);

    void setValue(int value);
    void setValueAndText(int value, const std::string &text);

private:
    AsyncTask::Private *m_d;
};

/**
 * @brief Run the given function in a new thread and return a handle to it.
 *
 * Any exception thrown by the function is captured and rethrown from AsyncTask::waitForFinished().
 */
AsyncTask launchAsyncTask(std::function<std::expected<void, std::string>(TaskProgress &)> body);

} // namespace Miniscope
