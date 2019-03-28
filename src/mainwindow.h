/*
 * Copyright (C) 2019 Matthias Klumpp <matthias@tenstral.net>
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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QQueue>

class MiniScope;
class VideoViewWidget;
class QLabel;

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
    void on_btnStartStop_clicked();
    void on_btnRecord_toggled(bool checked);
    void on_sbExposure_valueChanged(int arg1);
    void on_sbExcitation_valueChanged(int value);
    void on_sbGain_valueChanged(int arg1);
    void on_losslessCheckBox_toggled(bool checked);
    void on_containerComboBox_currentIndexChanged(const QString &arg1);
    void on_codecComboBox_currentIndexChanged(const QString &arg1);
    void on_cbExtRecTrigger_toggled(bool checked);
    void on_sbDisplayMax_valueChanged(int arg1);
    void on_sbDisplayMin_valueChanged(int arg1);

protected:
    void closeEvent (QCloseEvent *event) override;

private:
    Ui::MainWindow *ui;
    QLabel *m_statusBarLabel;

    MiniScope *m_mscope;
    VideoViewWidget *m_scopeView;
    bool m_running;

    int m_messageCount;
    QQueue<QString> m_newMessages;

    void addLogMessage(const QString &msg);
    void setStatusText(const QString& msg);
};

#endif // MAINWINDOW_H
