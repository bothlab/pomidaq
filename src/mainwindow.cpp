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

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QScrollBar>
#include <miniscope.h>
#include "videoviewwidget.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_scopeView = new VideoViewWidget(this);
    ui->videoDisplayWidget->layout()->addWidget(m_scopeView);

    m_mscope = new MiniScope();
    m_mscope->setOnMessage([&](const std::string &msg) {
        m_newMessages.enqueue(QString::fromStdString(msg));
    });

    // display default values
    ui->sbExposure->setValue(m_mscope->exposure());
    ui->sbExcitation->setValue(m_mscope->excitation());
    ui->sbGain->setValue(m_mscope->gain());

    ui->btnStartStop->setFocus();
    ui->containerScopeControls->setEnabled(false);
    ui->btnRecord->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete m_mscope;
}

void MainWindow::on_sbExcitation_valueChanged(int value)
{
    m_mscope->setExcitation(value);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    Q_UNUSED(event);
    m_mscope->disconnect();
}

void MainWindow::addLogMessage(const QString &msg)
{
    m_messageCount += 1;
    if (m_messageCount > 200) {
        m_messageCount = 1;
        ui->logTextList->clear();
    }

    ui->logTextList->appendPlainText(msg);
    ui->logTextList->verticalScrollBar()->setValue(ui->logTextList->verticalScrollBar()->maximum());
}

void MainWindow::on_sbExposure_valueChanged(int arg1)
{
    m_mscope->setExposure(arg1);
}

void MainWindow::on_btnStartStop_clicked()
{
    if (m_mscope->running()) {
        m_mscope->disconnect();
        ui->btnStartStop->setText("Connect");
        ui->btnStartStop->setChecked(false);

        ui->containerScopeControls->setEnabled(false);
        ui->btnRecord->setEnabled(false);
        return;
    }
    m_newMessages.clear();

    m_mscope->setScopeCamId(ui->sbCamId->value());
    m_mscope->connect();

    ui->btnStartStop->setText("Stop");
    ui->btnStartStop->setChecked(true);
    ui->containerScopeControls->setEnabled(true);
    ui->btnRecord->setEnabled(true);
    //QApplication::processEvents();

    while (m_mscope->running()) {
        auto frame = m_mscope->currentFrame();
        if (!frame.empty())
            m_scopeView->showImage(frame);
        if (!m_newMessages.isEmpty())
            addLogMessage(m_newMessages.dequeue());

        QApplication::processEvents();
    }
}

void MainWindow::on_sbGain_valueChanged(int arg1)
{
    m_mscope->setGain(arg1);
}

void MainWindow::on_btnRecord_toggled(bool checked)
{
    Q_UNUSED(checked);
}
