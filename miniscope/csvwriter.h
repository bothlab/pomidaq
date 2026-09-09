/*
 * Copyright (C) 2019-2024 Matthias Klumpp <matthias@tenstral.net>
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

#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/**
 * @brief Write rows of data to a CSV file from a dedicated thread.
 */
class CSVWriter
{
public:
    using ErrorCallback = std::function<void(const std::string &errorMessage)>;

    explicit CSVWriter(const std::string &filename);
    ~CSVWriter();

    CSVWriter(const CSVWriter &) = delete;
    CSVWriter &operator=(const CSVWriter &) = delete;

    /**
     * @brief Set a callback that is invoked (from the writer thread) if writing fails.
     */
    void setErrorCallback(ErrorCallback callback);

    void start();
    void addRow(const std::vector<std::string> &rowData);
    void addRow(const std::chrono::milliseconds &timestamp, const std::vector<float> &rowData);
    void stop();

private:
    void run();

    std::string m_filename;
    ErrorCallback m_errorCallback;
    std::deque<std::vector<std::string>> m_dataQueue;
    std::mutex m_mutex;
    std::condition_variable m_dataAvailable;
    std::thread m_thread;
    bool m_stopThread;
};
