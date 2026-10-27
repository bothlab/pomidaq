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

#include "csvwriter.h"

#include <format>
#include <fstream>

#include "loginternal.h"

MS_DEFINE_LOG_CATEGORY(logCSVWriter, "csvwriter");

CSVWriter::CSVWriter(const std::string &filename)
    : m_filename(filename),
      m_stopThread(false)
{
}

CSVWriter::~CSVWriter()
{
    stop();
}

void CSVWriter::setErrorCallback(ErrorCallback callback)
{
    m_errorCallback = std::move(callback);
}

void CSVWriter::start()
{
    if (m_thread.joinable())
        return;
    m_stopThread = false;
    m_thread = std::thread(&CSVWriter::run, this);
}

void CSVWriter::addRow(const std::vector<std::string> &rowData)
{
    std::lock_guard<std::mutex> locker(m_mutex);
    m_dataQueue.push_back(rowData);
    m_dataAvailable.notify_one();
}

void CSVWriter::addRow(const std::chrono::milliseconds &timestamp, const std::vector<float> &rowData)
{
    std::vector<std::string> rowStr;
    rowStr.push_back(std::to_string(timestamp.count()));
    for (const auto &n : rowData)
        rowStr.push_back(std::format("{:g}", n));
    addRow(rowStr);
}

void CSVWriter::stop()
{
    bool waitForThread = m_stopThread == false;
    {
        std::lock_guard<std::mutex> locker(m_mutex);
        m_stopThread = true;
        m_dataAvailable.notify_one();
    }
    if (waitForThread && m_thread.joinable())
        m_thread.join();
}

void CSVWriter::run()
{
    const auto reportError = [this](const std::string &errorMsg) {
        MS_LOG_WARNING(logCSVWriter, "{}", errorMsg);
        if (m_errorCallback)
            m_errorCallback(errorMsg);
    };

    std::ofstream file(m_filename, std::ios::out | std::ios::app);
    if (!file.is_open()) {
        reportError("Unable to open file " + m_filename);
        return;
    }

    MS_LOG_DEBUG(logCSVWriter, "Writing CSV file: {}", m_filename);

    while (true) {
        std::vector<std::string> rowData;
        {
            std::unique_lock<std::mutex> locker(m_mutex);
            m_dataAvailable.wait(locker, [this] {
                return m_stopThread || !m_dataQueue.empty();
            });

            // write all remaining rows before we stop
            if (m_dataQueue.empty())
                break;

            rowData = std::move(m_dataQueue.front());
            m_dataQueue.pop_front();
        }

        std::string line;
        for (size_t i = 0; i < rowData.size(); ++i) {
            if (i > 0)
                line += ";";
            line += rowData[i];
        }
        file << line << "\n";
        if (!file) {
            reportError("Unable to write to file " + m_filename);
            return;
        }
    }

    // data is buffered, so we may only notice write errors when flushing it on close
    file.close();
    if (file.fail()) {
        reportError("Unable to write to file " + m_filename);
        return;
    }

    MS_LOG_DEBUG(logCSVWriter, "Writer thread stopped.");
}
