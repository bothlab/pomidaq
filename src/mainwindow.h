/*
 * Copyright (C) 2019-2020 Matthias Klumpp <matthias@tenstral.net>
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

#include <QMainWindow>
#include <QQueue>
#include <QVBoxLayout>

class ImageViewWidget;
class MSControlWidget;
class QLabel;
namespace MScope {
class Miniscope;
}

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void on_deviceTypeComboBox_currentIndexChanged(const QString &arg1);
    void on_btnDevConnect_clicked();
    void on_btnRecord_toggled(bool checked);
    void on_losslessCheckBox_toggled(bool checked);
    void on_containerComboBox_currentIndexChanged(const QString &arg1);
    void on_codecComboBox_currentIndexChanged(const QString &arg1);
    void on_cbExtRecTrigger_toggled(bool checked);
    void on_sbDisplayMax_valueChanged(int arg1);
    void on_sbDisplayMin_valueChanged(int arg1);
    void on_sliceIntervalSpinBox_valueChanged(int arg1);

    void on_actionAbout_triggered();
    void on_actionAboutVideoFormats_triggered();
    void on_actionQuit_triggered();
    void on_actionSetDataLocation_triggered();
    void on_btnOpenSaveDir_clicked();

    void on_bgDivCheckBox_toggled(bool checked);
    void on_bgSubstCheckBox_toggled(bool checked);
    void on_accAlphaSpinBox_valueChanged(double arg1);

    void on_actionShowMiniscopeLog_toggled(bool arg1);

    void on_actionUseDarkTheme_toggled(bool arg1);
    void on_actionSetTimestampStyle_triggered();

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    Ui::MainWindow *ui;
    QLabel *m_statusBarLabel;

    MScope::Miniscope *m_mscope;
    QList<MSControlWidget*> m_controls;
    QVBoxLayout *m_controlsLayout;
    ImageViewWidget *m_scopeView;

    bool m_useUnixTimestamps;
    bool m_running;

    int m_messageCount;
    QQueue<QString> m_newMessages;

    QString dataDir;

    void addLogMessage(const QString &msg);
    void setStatusText(const QString& msg);
    void setDataExportDir(const QString& dir);
    void setUseUnixTimestamps(bool useUnixTimestamp);
};
