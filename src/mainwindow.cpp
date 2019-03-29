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
#include <QMessageBox>
#include <QLabel>
#include <miniscope.h>
#include "videoviewwidget.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Create status bar
    m_statusBarLabel = new QLabel("OK", this);
    statusBar()->addWidget(m_statusBarLabel, 1);
    statusBar()->setSizeGripEnabled(false);  // fixed window size

    // Video view
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

    // ensure codecs and container UI is aligned with the mscope settings
    ui->codecComboBox->setCurrentIndex(0);
    this->on_codecComboBox_currentIndexChanged(ui->codecComboBox->currentText());
    ui->containerComboBox->setCurrentIndex(0);
    this->on_containerComboBox_currentIndexChanged(ui->containerComboBox->currentText());
    ui->losslessCheckBox->setChecked(true);
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
    setStatusText(msg);
}

void MainWindow::setStatusText(const QString& msg)
{
    m_statusBarLabel->setText(msg);
    QApplication::processEvents();
}

void MainWindow::on_sbExposure_valueChanged(int arg1)
{
    m_mscope->setExposure(arg1);
}

void MainWindow::on_btnStartStop_clicked()
{
    if (m_mscope->running()) {
        ui->btnStartStop->setEnabled(false);
        QApplication::processEvents();
        m_mscope->disconnect();
        ui->btnStartStop->setEnabled(true);
        return;
    }
    m_newMessages.clear();

    ui->btnStartStop->setEnabled(false);
    m_mscope->setScopeCamId(ui->sbCamId->value());
    if (!m_mscope->connect()) {
        QMessageBox::critical(this,
                              "Error",
                              QString("Unable to connect to camera '%1'.").arg(ui->sbCamId->value()));
        setStatusText("Connection error.");
        ui->btnStartStop->setEnabled(true);
        return;
    }

    // run and display images
    m_mscope->run();

    ui->btnStartStop->setText("Stop");
    ui->btnStartStop->setChecked(true);
    ui->containerScopeControls->setEnabled(true);
    ui->btnRecord->setEnabled(true);
    ui->btnStartStop->setEnabled(true);

    while (m_mscope->running()) {
        auto frame = m_mscope->currentFrame();
        if (!frame.empty()) {
            m_scopeView->showImage(frame);

            ui->labelCurrentFPS->setText(QString::number(m_mscope->currentFPS()));
            ui->labelDroppedFrames->setText(QString::number(m_mscope->droppedFramesCount()));

            ui->labelScopeMin->setText(QString::number(m_mscope->minFluor()).rightJustified(3, '0'));
            ui->labelScopeMax->setText(QString::number(m_mscope->maxFluor()).rightJustified(3, '0'));
        }

        if (!m_newMessages.isEmpty())
            addLogMessage(m_newMessages.dequeue());

        QApplication::processEvents();
    }

    // if we are here, we stopped running
    // get last messages
    while (!m_newMessages.isEmpty())
        addLogMessage(m_newMessages.dequeue());

    // reset UI elements
    ui->btnStartStop->setText("Connect");
    ui->btnStartStop->setChecked(false);

    ui->containerScopeControls->setEnabled(false);
    ui->btnRecord->setEnabled(false);
    ui->btnStartStop->setEnabled(true);
    ui->labelCurrentFPS->setText(QStringLiteral("???"));
}

void MainWindow::on_sbGain_valueChanged(int arg1)
{
    m_mscope->setGain(arg1);
}

void MainWindow::on_btnRecord_toggled(bool checked)
{
    // don't do anything if miniscope isn't running
    if (!m_mscope->running())
        return;

    if (checked) {
        if (m_mscope->startRecording("/tmp/rectest"))
            ui->gbRecording->setEnabled(false);
        else
            ui->btnRecord->setChecked(false);
    } else {
        m_mscope->stopRecording();
        ui->gbRecording->setEnabled(true);
    }
}

void MainWindow::on_codecComboBox_currentIndexChanged(const QString &arg1)
{
    // reset state of lossless infobox
    ui->losslessCheckBox->setEnabled(true);
    ui->losslessLabel->setEnabled(true);
    ui->losslessCheckBox->setChecked(m_mscope->recordLossless());
    ui->containerComboBox->setEnabled(true);

    if (arg1 == "VP9")
        m_mscope->setVideoCodec(VideoCodec::VP9);
    else if (arg1 == "AV1")
        m_mscope->setVideoCodec(VideoCodec::AV1);
    else if (arg1 == "FFV1") {
        m_mscope->setVideoCodec(VideoCodec::FFV1);

        // FFV1 is always lossless
        ui->losslessCheckBox->setEnabled(false);
        ui->losslessLabel->setEnabled(false);
        ui->losslessCheckBox->setChecked(true);

        // FFV1 only works with MKV containers
        ui->containerComboBox->setCurrentIndex(0);
        ui->containerComboBox->setEnabled(false);
    } else if (arg1 == "MPEG-4") {
        m_mscope->setVideoCodec(VideoCodec::MPEG4);

        // MPEG-4 can't do lossless encoding
        ui->losslessCheckBox->setEnabled(false);
        ui->losslessLabel->setEnabled(false);
        ui->losslessCheckBox->setChecked(false);
    } else if (arg1 == "None") {
        m_mscope->setVideoCodec(VideoCodec::Raw);

        // Raw is always lossless
        ui->losslessCheckBox->setEnabled(false);
        ui->losslessLabel->setEnabled(false);
        ui->losslessCheckBox->setChecked(true);

        // Raw RGB only works with AVI containers
        ui->containerComboBox->setCurrentIndex(3);
        ui->containerComboBox->setEnabled(false);
    } else
        qCritical() << "Unknown video codec option selected:" << arg1;
}

void MainWindow::on_containerComboBox_currentIndexChanged(const QString &arg1)
{
    if (arg1 == "MKV")
        m_mscope->setVideoContainer(VideoContainer::Matroska);
    else if (arg1 == "MP4")
        m_mscope->setVideoContainer(VideoContainer::MP4);
    else if (arg1 == "AVI")
        m_mscope->setVideoContainer(VideoContainer::AVI);
    else
        qCritical() << "Unknown video container option selected:" << arg1;
}

void MainWindow::on_losslessCheckBox_toggled(bool checked)
{
    m_mscope->setRecordLossless(checked);
}

void MainWindow::on_cbExtRecTrigger_toggled(bool checked)
{
    ui->btnRecord->setChecked(!checked);
    ui->btnRecord->setEnabled(!checked);
    m_mscope->setExternalRecordTrigger(checked);
}

void MainWindow::on_sbDisplayMin_valueChanged(int arg1)
{
    m_mscope->setMinFluorDisplay(arg1);
}


void MainWindow::on_sbDisplayMax_valueChanged(int arg1)
{
    m_mscope->setMaxFluorDisplay(arg1);
}

void MainWindow::on_fpsSpinBox_valueChanged(int arg1)
{
    m_mscope->setFps(static_cast<uint>(arg1));
}
