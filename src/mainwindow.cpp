/*
 * Copyright (C) 2019-2021 Matthias Klumpp <matthias@tenstral.net>
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

#include "config.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QScrollBar>
#include <QMessageBox>
#include <QLabel>
#include <QStandardPaths>
#include <QFileDialog>
#include <QDateTime>
#include <QSettings>
#include <QInputDialog>
#include <miniscope.h>

#include "imageviewwidget.h"
#include "mscontrolwidget.h"

#ifdef Q_OS_LINUX
#include <KSharedConfig>
#include <KColorScheme>
#endif

using namespace MScope;

static bool darkColorSchemeAvailable()
{
#ifdef Q_OS_LINUX
    const auto fname = QStringLiteral("/usr/share/color-schemes/BreezeDark.colors");
    QFile file(fname);
    if (!file.exists()) {
        qDebug().noquote() << "Could not find dark color scheme file";
        return false;
    }
    return true;
#else
    return false;
#endif
}

#ifdef Q_OS_LINUX
static void changeColorScheme(const QString &filename, bool darkColors = false)
{
    QFile file(filename);
    if (!file.exists()) {
        qWarning().noquote() << "Could not find color scheme file" << filename << "Colors will not be changed.";
        return;
    }

    auto config = KSharedConfig::openConfig(filename);

    QPalette palette = qApp->palette();
    QPalette::ColorGroup states[3] = { QPalette::Active, QPalette::Inactive, QPalette::Disabled };
    KColorScheme schemeTooltip(QPalette::Active, KColorScheme::Tooltip, config);

    for (int i = 0; i < 3 ; ++i) {
        QPalette::ColorGroup state = states[i];
        KColorScheme schemeView(state,      KColorScheme::View,      config);
        KColorScheme schemeWindow(state,    KColorScheme::Window,    config);
        KColorScheme schemeButton(state,    KColorScheme::Button,    config);
        KColorScheme schemeSelection(state, KColorScheme::Selection, config);

        palette.setBrush(state, QPalette::WindowText,      schemeWindow.foreground());
        palette.setBrush(state, QPalette::Window,          schemeWindow.background());
        palette.setBrush(state, QPalette::Base,            schemeView.background());
        palette.setBrush(state, QPalette::Text,            schemeView.foreground());
        palette.setBrush(state, QPalette::Button,          schemeButton.background());
        palette.setBrush(state, QPalette::ButtonText,      schemeButton.foreground());
        palette.setBrush(state, QPalette::Highlight,       schemeSelection.background());
        palette.setBrush(state, QPalette::HighlightedText, schemeSelection.foreground());
        palette.setBrush(state, QPalette::ToolTipBase,     schemeTooltip.background());
        palette.setBrush(state, QPalette::ToolTipText,     schemeTooltip.foreground());

        palette.setColor(state, QPalette::Light,           schemeWindow.shade(KColorScheme::LightShade));
        palette.setColor(state, QPalette::Midlight,        schemeWindow.shade(KColorScheme::MidlightShade));
        palette.setColor(state, QPalette::Mid,             schemeWindow.shade(KColorScheme::MidShade));
        palette.setColor(state, QPalette::Dark,            schemeWindow.shade(KColorScheme::DarkShade));
        palette.setColor(state, QPalette::Shadow,          schemeWindow.shade(KColorScheme::ShadowShade));

        palette.setBrush(state, QPalette::AlternateBase,   schemeView.background(KColorScheme::AlternateBackground));
        palette.setBrush(state, QPalette::Link,            schemeView.foreground(KColorScheme::LinkText));
        palette.setBrush(state, QPalette::LinkVisited,     schemeView.foreground(KColorScheme::VisitedText));
    }

    qApp->setProperty("KDE_COLOR_SCHEME_PATH", filename);
    qApp->setPalette(palette);

    QIcon::setThemeName(darkColors ? QStringLiteral("breeze-dark") : QStringLiteral("breeze"));
}

static void changeColorsDarkmode(bool enabled)
{
    if (enabled)
        changeColorScheme(QStringLiteral("/usr/share/color-schemes/BreezeDark.colors"), true);
    else
        changeColorScheme(QStringLiteral("/usr/share/color-schemes/Breeze.colors"), false);
}
#endif

static MainWindow *g_mainWin = nullptr;

void messageOutputHandler(QtMsgType type, const QMessageLogContext &ctx, const QString &msg)
{
    QByteArray localMsg = msg.toLocal8Bit();
    switch (type) {
    case QtDebugMsg:
        fprintf(stdout, "%s: %s\n", ctx.category, localMsg.constData());
        break;
    case QtInfoMsg:
        fprintf(stdout, "%s: %s\n", ctx.category, localMsg.constData());
        if (g_mainWin) g_mainWin->queueLogMessage(msg);
        break;
    case QtWarningMsg:
        fprintf(stderr, "W: %s: %s\n", ctx.category, localMsg.constData());
        if (g_mainWin) g_mainWin->queueLogMessage(QStringLiteral("W: %1").arg(msg));
        break;
    case QtCriticalMsg:
        fprintf(stderr, "E: %s: %s\n", ctx.category, localMsg.constData());
        if (g_mainWin) g_mainWin->queueLogMessage(QStringLiteral("E: %1").arg(msg));
        break;
    case QtFatalMsg:
        fprintf(stderr, "FATAL: %s: %s\n", ctx.category, localMsg.constData());
        if (g_mainWin) g_mainWin->queueLogMessage(QStringLiteral("FATAL: %1").arg(msg));
        break;
    }
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // set icons with fallbacks
    ui->btnOpenSaveDir->setIcon(QIcon::fromTheme(QStringLiteral("folder-open"),
                                                 QIcon(":/icons/folder-open.svg")));
    ui->actionSetDataLocation->setIcon(QIcon::fromTheme(QStringLiteral("folder-open"),
                                                        QIcon(":/icons/folder-open.svg")));
    ui->btnDispLimitsReset->setIcon(QIcon::fromTheme(QStringLiteral("edit-reset"),
                                                     QIcon(":/icons/edit-reset.svg")));

    // Create status bar
    m_statusBarLabel = new QLabel("OK", this);
    statusBar()->addWidget(m_statusBarLabel, 1);

    // don't display log by default
    ui->logTextList->setVisible(false);

    // Video view
    m_scopeView = new ImageViewWidget(this);
    ui->videoDisplayWidget->layout()->addWidget(m_scopeView);

    m_mscope = new Miniscope();
    m_mscope->setOnStatusMessage([&](const QString &msg, void*) {
        setStatusText(msg);
    });

    // Miniscope controls
    m_controlsLayout = new QVBoxLayout(ui->gbDeviceCtls);
    m_controlsLayout->setMargin(2);
    m_controlsLayout->setSpacing(4);
    ui->gbDeviceCtls->setLayout(m_controlsLayout);
    m_controlsLayout->addStretch();

    // display default values
    ui->accAlphaSpinBox->setValue(m_mscope->bgAccumulateAlpha());

    ui->btnDevConnect->setFocus();
    ui->containerScopeControls->setEnabled(false);
    ui->groupBoxDisplay->setEnabled(false);
    ui->btnRecord->setEnabled(false);

    // ensure codecs and container UI is aligned with the MiniScope settings
    ui->codecComboBox->setCurrentIndex(0);
    this->on_codecComboBox_currentIndexChanged(ui->codecComboBox->currentText());
    ui->containerComboBox->setCurrentIndex(0);
    this->on_containerComboBox_currentIndexChanged(ui->containerComboBox->currentText());
    ui->losslessCheckBox->setChecked(true);
    on_sliceIntervalSpinBox_valueChanged(ui->sliceIntervalSpinBox->value());

    // set export directory, default to /tmp
    setDataExportDir(QStandardPaths::writableLocation(QStandardPaths::StandardLocation::TempLocation));
    if (m_dataDir.isEmpty())
        setDataExportDir("/tmp");

    QSettings settings(qApp->organizationName(), qApp->applicationName());
    ui->actionUseDarkTheme->setVisible(false);
    if (darkColorSchemeAvailable()) {
        ui->actionUseDarkTheme->setVisible(true);
        ui->actionUseDarkTheme->setChecked(settings.value("ui/useDarkStyle", true).toBool());
    }
    setUseUnixTimestamps(settings.value("recording/useUnixTimestamps", false).toBool());
    ui->sliceIntervalSpinBox->setValue(settings.value("recording/videoSliceInterval", 5).toInt());

    // set display modes
    ui->displayModeCB->addItem(QStringLiteral("Raw Data"), QVariant::fromValue(DisplayMode::RawFrames));
    ui->displayModeCB->addItem(QStringLiteral("F - F₀"), QVariant::fromValue(DisplayMode::BackgroundDiff));

    // set device list
    ui->deviceTypeComboBox->addItems(m_mscope->availableDeviceTypes());

    // set the right first toolbox page
    ui->toolBox->setCurrentIndex(0);

    // select our default Miniscope device choice, or the one last used by the user
    const auto selectedDevice = settings.value("device/type", QStringLiteral("Miniscope_V4")).toString().toLower();
    for (int i = 0; i < ui->deviceTypeComboBox->count(); ++i) {
        if (ui->deviceTypeComboBox->itemText(i).toLower() == selectedDevice) {
            ui->deviceTypeComboBox->setCurrentIndex(i);
            break;
        }
    }

    // restore window geometry and splitter position, if any is saved
    setGeometry(settings.value("ui/geometry", geometry()).toRect());
    if (settings.contains("ui/splitterSizes")) {
        QList<int> splitSizes;
        auto byteArray = settings.value("ui/splitterSizes").toByteArray();
        QDataStream stream(&byteArray, QIODevice::ReadWrite);
        stream >> splitSizes;
        if (!splitSizes.isEmpty())
            ui->splitter->setSizes(splitSizes);
    }

    // restore previous data storage location, if it is still valid
    const auto savedDataDir = settings.value("recording/dataDir").toString();
    if (!savedDataDir.isEmpty()) {
        QFileInfo fi(savedDataDir);
        if (fi.isDir()) {
            setDataExportDir(savedDataDir);
        } else {
            QMessageBox::warning(this,
                                 "Data directory changed",
                                 QStringLiteral("The previous data storage location ('%1') does no longer exist or is not writable. Falling back to default, temporary directory.")
                                 .arg(savedDataDir));
        }
    }

    // install new message handler so output can also be redirected to the GUI
    // log display (Windows users like this...)
    g_mainWin = this;
    qInstallMessageHandler(messageOutputHandler);
}

MainWindow::~MainWindow()
{
    // reset default message handler
    qInstallMessageHandler(nullptr);
    g_mainWin = nullptr;

    delete ui;
    delete m_mscope;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    m_mscope->disconnect();

    // transform splitter sizes list to byte array, a QVariant
    // doesn't work in the Windows Registry
    QByteArray splitterSizesBytes;
    QDataStream ssizesStream(&splitterSizesBytes, QIODevice::WriteOnly);
    ssizesStream << ui->splitter->sizes();

    // save settings
    QSettings settings(qApp->organizationName(), qApp->applicationName());
    settings.setValue("ui/geometry", geometry());
    settings.setValue("ui/splitterSizes", splitterSizesBytes);
    settings.setValue("recording/useUnixTimestamps", m_useUnixTimestamps);
    settings.setValue("recording/videoSliceInterval", ui->sliceIntervalSpinBox->value());
    settings.setValue("device/type", ui->deviceTypeComboBox->currentText());
    if (!m_dataDir.isEmpty())
        settings.setValue("recording/dataDir", m_dataDir);

    QMainWindow::closeEvent(event);
}

void MainWindow::writeLogMessage(const QString &msg)
{
    m_messageCount += 1;
    if (m_messageCount > 200) {
        m_messageCount = 1;
        ui->logTextList->clear();
    }

    ui->logTextList->appendPlainText(msg);
    ui->logTextList->verticalScrollBar()->setValue(ui->logTextList->verticalScrollBar()->maximum());
}

void MainWindow::queueLogMessage(const QString &msg)
{
    QMetaObject::invokeMethod(this, [=](){
        writeLogMessage(msg);
    }, Qt::QueuedConnection);
}

void MainWindow::setStatusText(const QString& msg)
{
    m_statusBarLabel->setText(msg);
    QApplication::processEvents();
}

void MainWindow::setDataExportDir(const QString &dir)
{
    setWindowTitle(QStringLiteral("Portable Miniscope DAQ - %1").arg(dir));
    m_dataDir = dir;
    ui->pathHintLabel->setText(m_dataDir);
    ui->pathHintLabel->setToolTip(m_dataDir);
}

void MainWindow::setUseUnixTimestamps(bool useUnixTimestamp)
{
    m_useUnixTimestamps = useUnixTimestamp;
    if (m_useUnixTimestamps)
        ui->labelTimestampStyle->setText(QStringLiteral("unix-epoch"));
    else
        ui->labelTimestampStyle->setText(QStringLiteral("start-at-zero"));
}

void MainWindow::on_deviceTypeComboBox_currentIndexChanged(const QString &arg1)
{
    // clear previous controls
    for (const auto &control : m_controls)
        delete control;
    m_controls.clear();

    // load new controls
    if (!m_mscope->loadDeviceConfig(arg1)) {
        QMessageBox::critical(this,
                              QStringLiteral("Error"),
                              QStringLiteral("Unable to load device configuration: %1")
                              .arg(m_mscope->lastError()));
        return;
    }

    // display widgets for new controls
    for (const auto &ctl : m_mscope->controls()) {
        const auto w = new MSControlWidget(ctl, ui->gbDeviceCtls);
        m_controlsLayout->insertWidget(0, w);
        connect(w, &MSControlWidget::valueChanged, [&](const QString ctlId, double value) {
            m_mscope->setControlValue(ctlId, value);
        });
        m_controls.append(w);
    }
}

void MainWindow::on_btnDevConnect_clicked()
{
    if (m_mscope->isRunning()) {
        ui->btnDevConnect->setEnabled(false);
        QApplication::processEvents();
        m_mscope->disconnect();
        ui->btnDevConnect->setEnabled(true);
        ui->sbCamId->setEnabled(true);
        ui->deviceTypeComboBox->setEnabled(true);
        return;
    }

    ui->btnDevConnect->setEnabled(false);
    m_mscope->setScopeCamId(ui->sbCamId->value());
    if (!m_mscope->connect()) {
        QMessageBox::critical(this,
                              "Error",
                              QStringLiteral("Unable to connect to Miniscope camera at '%1'. Is the DAQ board connected properly?")
                                             .arg(ui->sbCamId->value()));
        setStatusText("Connection error.");
        ui->btnDevConnect->setEnabled(true);
        return;
    }

    // set whether we use continuous zero-based timestamps,
    // or UNIX timestamps instead
    m_mscope->setUseUnixTimestamps(m_useUnixTimestamps);

    // reflect currently active control values in the UI
    for (const auto &w : m_controls)
        w->setValue(m_mscope->controlValue(w->controlId()));

    // run and display images
    m_mscope->run();

    ui->btnDevConnect->setText("Disconnect");
    ui->btnDevConnect->setChecked(true);
    ui->containerScopeControls->setEnabled(true);
    ui->groupBoxDisplay->setEnabled(true);
    ui->btnRecord->setEnabled(true);
    ui->btnDevConnect->setEnabled(true);
    ui->sbCamId->setEnabled(false);
    ui->deviceTypeComboBox->setEnabled(false);
    ui->actionSetTimestampStyle->setEnabled(false);

    // switch to controls page, the user will likely need to use that next
    ui->toolBox->setCurrentIndex(1);

    while (m_mscope->isRunning()) {
        auto frame = m_mscope->currentDisplayFrame();
        if (!frame.empty()) {
            m_scopeView->showImage(frame);

            ui->labelCurrentFPS->setText(QString::number(m_mscope->currentFps()));
            ui->labelDroppedFrames->setText(QString::number(m_mscope->droppedFramesCount()));

            ui->labelScopeMin->setText(QString::number(m_mscope->minFluor()).rightJustified(3, '0'));
            ui->labelScopeMax->setText(QString::number(m_mscope->maxFluor()).rightJustified(3, '0'));

            auto recMsecTimestamp = static_cast<int>(m_mscope->lastRecordedFrameTime().count());
            if (recMsecTimestamp > 0) {
                if (m_useUnixTimestamps) {
                    recMsecTimestamp = recMsecTimestamp - m_mscope->unixCaptureStartTime().count();
                }

                ui->labelRecordingTime->setText(QTime::fromMSecsSinceStartOfDay(recMsecTimestamp).toString("hh:mm:ss"));
            }
        }

        QApplication::processEvents();
    }

    // reset UI elements
    ui->btnDevConnect->setText("Connect");
    ui->btnDevConnect->setChecked(false);
    ui->btnRecord->setText("Record");

    ui->containerScopeControls->setEnabled(false);
    ui->groupBoxDisplay->setEnabled(false);
    ui->btnRecord->setEnabled(false);
    ui->btnDevConnect->setEnabled(true);
    ui->labelCurrentFPS->setText(QStringLiteral("???"));
    ui->sbCamId->setEnabled(true);
    ui->deviceTypeComboBox->setEnabled(true);
    ui->actionSetTimestampStyle->setEnabled(true);

    if (!m_mscope->lastError().isEmpty())
        QMessageBox::critical(this, "Error", m_mscope->lastError());

    // switch back to connect page, as this is the
    // only useful page when no scope is connected
    ui->toolBox->setCurrentIndex(0);
}

void MainWindow::on_btnRecord_toggled(bool checked)
{
    // don't do anything if miniscope isn't running
    if (!m_mscope->isRunning())
        return;

    QFileInfo dataLocation(m_dataDir);
    if (!dataLocation.isDir() || !dataLocation.isWritable() || m_dataDir.isEmpty()) {
        QMessageBox::critical(this,
                              "Recording Error",
                              QStringLiteral("Data location '%1' is not a directory or not writable.").arg(m_dataDir));
        return;
    }

    if (checked) {
        const auto videoFname = QDir(m_dataDir).filePath(QDateTime::currentDateTime().toString("yy-MM-dd-hhmm") + "_scope");
        if (m_mscope->startRecording(videoFname)) {
            ui->pageRecord->setEnabled(false);
            ui->btnDevConnect->setEnabled(false);
            ui->btnRecord->setText("Recording...");
        } else {
            ui->btnRecord->setChecked(false);
        }

    } else {
        m_mscope->stopRecording();
        ui->pageRecord->setEnabled(true);
        ui->btnDevConnect->setEnabled(true);
        ui->btnRecord->setText("Record");
    }
}

void MainWindow::on_codecComboBox_currentIndexChanged(const QString &arg1)
{
    // reset state of lossless infobox
    ui->losslessCheckBox->setEnabled(true);
    ui->losslessCheckBox->setChecked(m_mscope->recordLossless());
    ui->containerComboBox->setEnabled(true);

    if (arg1 == "AV1") {
        m_mscope->setVideoCodec(VideoCodec::AV1);

    } else if (arg1 == "FFV1") {
        m_mscope->setVideoCodec(VideoCodec::FFV1);

        // FFV1 is always lossless
        ui->losslessCheckBox->setEnabled(false);
        ui->losslessCheckBox->setChecked(true);

    } else if (arg1 == "VP9") {
        m_mscope->setVideoCodec(VideoCodec::VP9);

    } else if (arg1 == "HEVC") {
        m_mscope->setVideoCodec(VideoCodec::HEVC);

        // H.256 only works with MKV and MP4 containers, select MKV by default
        ui->containerComboBox->setCurrentIndex(0);
        ui->containerComboBox->setEnabled(false);

    } else if (arg1 == "MPEG-4") {
        m_mscope->setVideoCodec(VideoCodec::MPEG4);

        // MPEG-4 can't do lossless encoding
        ui->losslessCheckBox->setEnabled(false);
        ui->losslessCheckBox->setChecked(false);

    } else if (arg1 == "Raw") {
        m_mscope->setVideoCodec(VideoCodec::Raw);

        // Raw is always lossless
        ui->losslessCheckBox->setEnabled(false);
        ui->losslessCheckBox->setChecked(true);

        // Raw RGB only works with AVI containers
        ui->containerComboBox->setCurrentIndex(1);
        ui->containerComboBox->setEnabled(false);

    } else
        qCritical() << "Unknown video codec option selected:" << arg1;
}

void MainWindow::on_containerComboBox_currentIndexChanged(const QString &arg1)
{
    if (arg1 == "MKV")
        m_mscope->setVideoContainer(VideoContainer::Matroska);
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
    ui->btnDispLimitsReset->setEnabled(true);
}


void MainWindow::on_sbDisplayMax_valueChanged(int arg1)
{
    m_mscope->setMaxFluorDisplay(arg1);
    ui->btnDispLimitsReset->setEnabled(true);

}

void MainWindow::on_btnDispLimitsReset_clicked()
{
    ui->sbDisplayMin->setValue(ui->sbDisplayMin->minimum());
    ui->sbDisplayMax->setValue(ui->sbDisplayMax->maximum());
    ui->btnDispLimitsReset->setEnabled(false);
}

void MainWindow::on_btnOpenSaveDir_clicked()
{
    on_actionSetDataLocation_triggered();
}

void MainWindow::on_actionSetDataLocation_triggered()
{
    auto dir = QFileDialog::getExistingDirectory(this,
                                                 "Select Directory",
                                                 QStandardPaths::writableLocation(QStandardPaths::HomeLocation),
                                                 QFileDialog::ShowDirsOnly);
    if (!dir.isEmpty())
        setDataExportDir(dir);
}

void MainWindow::on_actionQuit_triggered()
{
    this->close();
}

void MainWindow::on_actionAboutVideoFormats_triggered()
{
    const auto infoText = QStringLiteral(
                "<html>"
                "<h3>Which video codec/container should I use?</h3>"
                "<p>PoMiDAQ allows the selction of a few different containers and codecs to store recorded videos. "
                "This brief information may help you decide which format is best suited for your application.</p>"
                "<h4>Matroska (MKV) Container</h4>"
                "<p>This is the most flexible container format. It is fully open-source and patent-free and suitable for long-term storage of "
                "videos. However, some tools such as MATLAB do not natively support it, so if you use MKV you may need to use 3rd-party toolboxes.</p>"
                "<h4>Audio Video Interleave (AVI) Container</h4>"
                "<p>AVI is an old and less flexible container format, which lacks a few features such as standardized ways to store timestamps and aspect ratios. "
                "Due to its age it is very well supported in many tools and may be your first choice if you are aiming for maximum compatibility.</p>"
                "<h4>FFV1 Codec</h4>"
                "<p>This lossless codec is designed for archivability of data and relatively good compression while preserving all information that was present in "
                "the uncompressed image. It is used by many institutions and broadcasting companies and widely supported. Yet, a few tools (such as MATLAB again) may "
                "not natively support it, so you may need to use 3rd-party tools to read the generated data.</p>"
                "<h4>No Codec</h4>"
                "<p>No compression is used to store the images. This will yield very large files, but reading the generated data is relatively easy and supported by many tools.</p>"
                "<h4>Any Other Selectable Codec</h4>"
                "<p>The AV1 codec may become very useful in future, as it is high-quality and open-source and patent-free and an industry standard. However, it is currently too slow "
                "for real-time data acquisition. The same applies to the VP9 codec, unless you record with lower framerates.</p>"
                "<p>H.265 is a popular codec for video compression. It is widely supported and already has fast encoders, but is patent encumbered. You may decide to use it if you need "
                "better compression than FFV1 can offer you and you can read the generated movies.</p>"
                "<p>MPEG-4 is an older video compression standard. You pretty much never want to use it (except for testing), as it is inferior to the other supported codecs.</p>"
                );

    QMessageBox dialog(this);
    dialog.setWindowTitle(QStringLiteral("Video format help"));
    dialog.setInformativeText(infoText);
    dialog.setStandardButtons(QMessageBox::Ok);
    dialog.exec();
}

void MainWindow::on_actionAbout_triggered()
{
    const auto text = QStringLiteral(
                "PoMiDAQ Version " PROJECT_VERSION " \n\n"
                "(c) 2019-2020 Matthias Klumpp\n\n"
                "PoMiDAQ is free software: you can redistribute it and/or modify "
                "it under the terms of the GNU Lesser General Public License as published by "
                "the Free Software Foundation, either version 3 of the License, or "
                "(at your option) any later version.\n"
                "\n"
                "PoMiDAQ is distributed in the hope that it will be useful, "
                "but WITHOUT ANY WARRANTY; without even the implied warranty of "
                "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the "
                "GNU Lesser General Public License for more details.");
    QMessageBox::about(this, QStringLiteral("About this tool"), text);
}

void MainWindow::on_displayModeCB_currentIndexChanged(int)
{
    const auto mode = ui->displayModeCB->currentData().value<DisplayMode>();

    ui->accAlphaSpinBox->setEnabled(mode == DisplayMode::BackgroundDiff);
    ui->accumulateAlphaLabel->setEnabled(mode == DisplayMode::BackgroundDiff);

    m_mscope->setDisplayMode(mode);
}

void MainWindow::on_sliceIntervalSpinBox_valueChanged(int arg1)
{
    m_mscope->setRecordingSliceInterval(static_cast<uint>(arg1));
}

void MainWindow::on_accAlphaSpinBox_valueChanged(double arg1)
{
    m_mscope->setBgAccumulateAlpha(arg1);
}

void MainWindow::on_actionShowMiniscopeLog_toggled(bool arg1)
{
    ui->logTextList->setVisible(arg1);
}

void MainWindow::on_actionUseDarkTheme_toggled(bool arg1)
{
#ifdef Q_OS_LINUX
    changeColorsDarkmode(arg1);

    QSettings settings(qApp->organizationName(), qApp->applicationName());
    settings.setValue("ui/useDarkStyle", arg1);
#endif
}

void MainWindow::on_actionSetTimestampStyle_triggered()
{
    QStringList items;
    items << QStringLiteral("Start at Zero") << QStringLiteral("Use UNIX timestamps");

    bool ok;
    auto item = QInputDialog::getItem(this,
                                      QStringLiteral("Set a timestamp style"),
                                      QStringLiteral("Style:"),
                                      items, m_useUnixTimestamps? 1 : 0, false,
                                      &ok);
    if (ok && !item.isEmpty()) {
        if (item.startsWith(QStringLiteral("Start at Zero"), Qt::CaseInsensitive))
            setUseUnixTimestamps(false);
        else
            setUseUnixTimestamps(true);
    }
}
