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

#include "asynctask.h"
#include "asynctask-private.h"

#include <atomic>
#include <exception>
#include <future>
#include <mutex>
#include <thread>

namespace Miniscope
{

class AsyncTask::Private
{
public:
    std::atomic_int progress{0};
    std::atomic_bool finished{false};

    std::mutex textMutex;
    std::string progressText;

    std::promise<std::expected<void, std::string>> promise;
    std::shared_future<std::expected<void, std::string>> future;
    std::thread thread;
};

AsyncTask::AsyncTask()
    : d(new AsyncTask::Private())
{
    d->future = d->promise.get_future().share();
}

AsyncTask::~AsyncTask()
{
    if (d && d->thread.joinable())
        d->thread.join();
}

AsyncTask::AsyncTask(AsyncTask &&other) noexcept = default;

AsyncTask &AsyncTask::operator=(AsyncTask &&other) noexcept
{
    if (this != &other) {
        // wait for any task we currently own before it gets destroyed,
        // destroying a joinable std::thread would terminate the process
        if (d && d->thread.joinable())
            d->thread.join();
        d = std::move(other.d);
    }
    return *this;
}

bool AsyncTask::isFinished() const
{
    return d->finished;
}

int AsyncTask::progressValue() const
{
    return d->progress;
}

std::string AsyncTask::progressText() const
{
    std::lock_guard<std::mutex> lock(d->textMutex);
    return d->progressText;
}

std::expected<void, std::string> AsyncTask::waitForFinished()
{
    if (d->thread.joinable())
        d->thread.join();
    return d->future.get();
}

TaskProgress::TaskProgress(AsyncTask::Private *d)
    : m_d(d)
{
}

void TaskProgress::setValue(int value)
{
    m_d->progress = value;
}

void TaskProgress::setValueAndText(int value, const std::string &text)
{
    {
        std::lock_guard<std::mutex> lock(m_d->textMutex);
        m_d->progressText = text;
    }
    m_d->progress = value;
}

AsyncTask launchAsyncTask(std::function<std::expected<void, std::string>(TaskProgress &)> body)
{
    AsyncTask task;
    auto *d = task.d.get();

    d->thread = std::thread([d, body = std::move(body)]() {
        std::expected<void, std::string> result;
        try {
            TaskProgress progress(d);
            result = body(progress);
        } catch (const std::exception &e) {
            result = std::unexpected(e.what());
        } catch (...) {
            result = std::unexpected("Unknown error");
        }
        d->promise.set_value(std::move(result));
        d->finished = true;
    });

    return task;
}

} // namespace Miniscope
