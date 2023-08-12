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

#pragma once

#include <QThread>
#include <QObject>
#include <QMutex>
#include <QWaitCondition>
#include <QQueue>
#include <QLoggingCategory>

Q_DECLARE_LOGGING_CATEGORY(logCSVWriter)

class CSVWriter : public QThread
{
    Q_OBJECT
public:
    explicit CSVWriter(const QString &filename, QObject *parent = nullptr);

    void addRow(const QStringList &rowData);
    void addRow(const std::chrono::milliseconds &timestamp, const std::vector<float> &rowData);
    void stop();

signals:
    void error(const QString &errorMessage);
    void finished();

protected:
    void run() override;

private:
    QString m_filename;
    QQueue<QStringList> m_dataQueue;
    QMutex m_mutex;
    QWaitCondition m_dataAvailable;
    bool m_stopThread;
};
