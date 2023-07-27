/*
 * Copyright (C) 2019-2022 Matthias Klumpp <matthias@tenstral.net>
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

#include <QDebug>
#include <QFile>
#include <QMutexLocker>

Q_LOGGING_CATEGORY(logCSVWriter, "csvwriter")

CSVWriter::CSVWriter(const QString &filename, QObject *parent)
    : QThread{parent},
      m_filename(filename),
      m_stopThread(false)
{
}

void CSVWriter::addRow(const QStringList &rowData)
{
    QMutexLocker locker(&m_mutex);
    m_dataQueue.enqueue(rowData);
    m_dataAvailable.wakeOne();
}

void CSVWriter::addRow(const std::chrono::milliseconds &timestamp, const std::vector<float> &rowData)
{
    QStringList rowStr;
    rowStr << QString::number(timestamp.count());
    for (const auto &n : rowData)
        rowStr << QString::number(n);
    addRow(rowStr);
}

void CSVWriter::stop()
{
    bool waitForThread = m_stopThread == false;
    {
        QMutexLocker locker(&m_mutex);
        m_stopThread = true;
        m_dataAvailable.wakeOne();
    }
    if (waitForThread)
        wait();
}

void CSVWriter::run()
{
    QFile file(m_filename);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append)) {
        QString errorMsg = "Unable to open file " + m_filename;
        qCWarning(logCSVWriter).noquote() << errorMsg;
        emit error(errorMsg);
        emit finished();
        return;
    }

    qCDebug(logCSVWriter).noquote() << "Writing CSV file:" << m_filename;

    QTextStream out(&file);
    while (true) {
        QStringList rowData;
        {
            QMutexLocker locker(&m_mutex);
            if (m_stopThread)
                break;

            if (m_dataQueue.isEmpty()) {
                m_dataAvailable.wait(&m_mutex);
                continue;
            }

            rowData = m_dataQueue.dequeue();
        }

        out << rowData.join(";") << "\n";
    }

    file.close();

    qCDebug(logCSVWriter).noquote() << "Writer thread stopped.";
}
