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

#include "miniscope.h"

#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <QDebug>
#include <QQueue>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QVector>
#include <QHash>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/videoio.hpp>

#include "scopeintf.h"
#include "videowriter.h"

namespace MScope
{

/**
 * @brief Defines a rule to scale values and convert them to a packet
 */
class ControlCommandRule
{
public:
    QVector<QHash<QString, int>> command;
    double valueScale;
    double valueOffset;
    QVector<double> valueMap;
};

#pragma GCC diagnostic ignored "-Wpadded"
class Miniscope::Private
{
public:
    Private()
        : thread(nullptr),
          scopeCamId(0),
          connected(false),
          running(false),
          recording(false),
          failed(false),
          checkRecTrigger(false),
          droppedFramesCount(0),
          useColor(false)
    {
        fps = 30;
        displayQueue.clear();
        videoCodec = VideoCodec::FFV1;
        videoContainer = VideoContainer::Matroska;

        printMessagesToStdout = false;

        showRed = true;
        showGreen = true;
        showBlue = true;

        bgDiffMethod = BackgroundDiffMethod::None;

        minFluorDisplay = 0;
        maxFluorDisplay = 255;

        recordingSliceInterval = 0; // don't slice
        bgAccumulateAlpha = 0.01;

        startTimepoint = std::chrono::time_point<std::chrono::steady_clock>::min();
        useUnixTime = false; // no timestamps in UNIX time by default
        unixCaptureStartTime = milliseconds_t(0);

        frameCallback.first = nullptr;
        displayFrameCallback.first = nullptr;
    }

    std::thread *thread;
    std::mutex frameRingMutex;
    std::mutex timeMutex;
    std::mutex msgMutex;

    cv::VideoCapture cam;
    int scopeCamId;

    QJsonObject deviceConfig;
    QString deviceType;
    cv::Size resolution;
    bool supportsColor;
    QString sensorType;

    QList<ControlDefinition> controls;
    QHash<QString, ControlCommandRule> controlRules;




    std::atomic_uint fps;
    QString videoFname;
    bool useUnixTime;
    std::atomic<milliseconds_t> unixCaptureStartTime;
    std::chrono::time_point<std::chrono::steady_clock> startTimepoint;
    std::atomic_bool captureStartTimeInitialized;

    std::atomic_int minFluor;
    std::atomic_int maxFluor;
    int minFluorDisplay;
    int maxFluorDisplay;

    std::atomic<BackgroundDiffMethod> bgDiffMethod;
    std::atomic<double> bgAccumulateAlpha;  // NOTE: Double may not actually be atomic

    bool connected;
    std::atomic_bool running;
    std::atomic_bool recording;
    std::atomic_bool failed;
    std::atomic_bool checkRecTrigger;

    std::atomic<size_t> droppedFramesCount;
    std::atomic_uint currentFPS;
    std::atomic<milliseconds_t> lastRecordedFrameTime;

    QQueue<cv::Mat> displayQueue;
    std::pair<RawFrameCallback, void*> frameCallback;
    std::pair<DisplayFrameCallback, void*> displayFrameCallback;

    bool printMessagesToStdout;

    bool useColor;
    bool showRed;
    bool showGreen;
    bool showBlue;

    VideoCodec videoCodec;
    VideoContainer videoContainer;
    bool recordLossless;
    uint recordingSliceInterval;

    QString lastError;
};
#pragma GCC diagnostic pop

} // end of namespace MScope


Miniscope::Miniscope(QObject *parent)
    : QObject(parent),
      d(new Miniscope::Private())
{
    d->fps = 20;
    d->deviceConfig = QJsonObject();
}

Miniscope::~Miniscope()
{
    finishCaptureThread();
    disconnect();
}

static QJsonObject msconfGetDevicesJson()
{
    QFile msTypesRc(QStringLiteral(":/config/miniscopes.json"));
    if (!msTypesRc.open(QIODevice::ReadOnly))
        return QJsonObject();
    const auto jDoc = QJsonDocument::fromJson(msTypesRc.readAll());

    return jDoc.object();
}

static int msconfStringToInt(const QString &s)
{
    // Should return a uint8 type of value (0 to 255)
    bool ok = false;
    int value;
    int size = s.size();
    if (size == 0) {
        qDebug() << "No data in string to convert to int";
        value = SEND_COMMAND_ERROR;
        ok = false;
    }
    else if (s.left(2) == "0x"){
        // HEX
        value = s.right(size - 2).toUInt(&ok, 16);
    }
    else if (s.left(2) == "0b"){
        // Binary
        value = s.right(size - 2).toUInt(&ok, 2);
    }
    else {
        value = s.toUInt(&ok, 10);
        if (ok == false) {
            // This is then a string
            if (s == "I2C")
                value = PROTOCOL_I2C;
            else if (s == "SPI")
                value = PROTOCOL_SPI;
            else if (s == "valueH24")
                value = SEND_COMMAND_VALUE_H24;
            else if (s == "valueH16")
                value = SEND_COMMAND_VALUE_H16;
            else if (s == "valueH")
                value = SEND_COMMAND_VALUE_H;
            else if (s == "valueL")
                value = SEND_COMMAND_VALUE_L;
            else if (s == "value")
                value = SEND_COMMAND_VALUE;
            else if (s == "value2H")
                value = SEND_COMMAND_VALUE2_H;
            else if (s == "value2L")
                value = SEND_COMMAND_VALUE2_L;
            else
                value = SEND_COMMAND_ERROR;
            ok = true;
        }
    }

    if (ok == true)
        return value;
    else
        return SEND_COMMAND_ERROR;
}

static QVector<QHash<QString, int>> msconfParseSendCommand(const QJsonArray &sendCommand)
{
    // creates a mapping to handle future I2C/SPI slider value send commands
    QVector<QHash<QString, int>> output;
    QHash<QString, int> commandStructure;
    QJsonObject jObj;
    QStringList keys;

    for (int i = 0; i < sendCommand.size(); i++) {
        jObj = sendCommand[i].toObject();
        keys = jObj.keys();

        for (int j = 0; j < keys.size(); j++) {
            // -1 = controlValue, -2 = error
            if (jObj[keys[j]].isString())
                commandStructure[keys[j]] = msconfStringToInt(jObj[keys[j]].toString());
            else if (jObj[keys[j]].isDouble())
                commandStructure[keys[j]] = jObj[keys[j]].toInt();
        }
        output.append(commandStructure);
    }

    return output;
}

QStringList Miniscope::availableMiniscopeTypes() const
{
    auto deviceTypes = msconfGetDevicesJson().keys();
    std::sort(deviceTypes.begin(), deviceTypes.end());
    return deviceTypes;
}

bool Miniscope::loadDeviceConfig(const QString &deviceType)
{
    const auto allDevConfigs = msconfGetDevicesJson();
    if (!allDevConfigs.contains(deviceType)) {
        d->lastError = QStringLiteral("Unable to find device configuration with name '%1'").arg(deviceType);
        return false;
    }
    d->deviceConfig = allDevConfigs[deviceType].toObject();
    d->deviceType = deviceType;

    // load basic settings
    d->resolution = cv::Size(d->deviceConfig["width"].toInt(-1), d->deviceConfig["height"].toInt(-1));
    d->supportsColor = d->deviceConfig["isColor"].toBool(false);
    d->sensorType = d->deviceConfig["sensor"].toString("unknown");

    // load information about available controls
    d->controls.clear();
    d->controlRules.clear();
    const auto controlSettings = d->deviceConfig["controlSettings"].toObject();
    if (controlSettings.isEmpty()) {
        qWarning() << "controlSettings missing from miniscopes.json for deviceType = " << d->deviceType;
        return true;
    }

    for (const QString& controlName : controlSettings.keys()) {
        const auto values = controlSettings.value(controlName).toObject();
        const auto keys = values.keys();
        ControlCommandRule commandRule;
        ControlDefinition control;
        control.name = controlName;

        QJsonValue startValue;
        for (const auto &key : values.keys()) {
            const auto value = values[key];
            if (key == "sendCommand") {
                commandRule.command = msconfParseSendCommand(value.toArray());
            } else if (key == "min") {
                control.valueMin = value.toInt();
            } else if (key == "max") {
                control.valueMax = value.toInt();
            } else if (key == "stepSize") {
                control.stepSize = value.toInt();
            } else if (key == "startValue") {
                startValue = value;
            } else if (key == "displayValueScale") {
                commandRule.valueScale = value.toDouble();
            } else if (key == "displayValueOffset") {
                commandRule.valueOffset = value.toDouble();
            } else if (key == "displaySpinBoxValues") {
                QStringList labels;
                for (const auto &text : value.toArray())
                    labels.append(text.toString());
                control.labels = labels;
            } else if (key == "outputValues") {
                QVector<double> outVals;
                for (const auto &v : value.toArray())
                    outVals.push_back(v.toDouble());
                commandRule.valueMap = outVals;
            }
        }

        // if we have a list of labels, we are a selector, otherwise
        // we assume a sliding-value controller
        if (control.labels.isEmpty()) {
            control.kind = ControlKind::Slider;
        } else {
            control.kind = ControlKind::Selector;
            control.valueMin = 0;
            control.valueMax = control.labels.length() - 1;
        }

        // set the start value
        if (!startValue.isNull()) {
            if (startValue.isString()) {
                control.startValue = control.labels.indexOf(startValue.toString());
            } else {
                control.startValue = startValue.toInt();
            }
        }

        d->controlRules[controlName] = commandRule;
        d->controls.append(control);
    }

    return true;
}

void Miniscope::startCaptureThread()
{
    finishCaptureThread();
    d->running = true;
    d->thread = new std::thread(captureThread, this);
}

void Miniscope::finishCaptureThread()
{
    if (d->thread != nullptr) {
        d->running = false;
        d->thread->join();
        delete d->thread;
        d->thread = nullptr;
    }
}

void Miniscope::emitMessage(const QString &msg)
{
    if (d->printMessagesToStdout)
        std::cout << msg.toStdString() << std::endl;

    // we don't want to call this callback twice simultaneously
    std::lock_guard<std::mutex> lock(d->msgMutex);
    Q_EMIT statusMessage(msg);
}

void Miniscope::fail(const QString &msg)
{
    d->recording = false;
    d->running = false;
    d->failed = true;
    d->lastError = msg;
    emitMessage(msg);
}

void Miniscope::setScopeCamId(int id)
{
    d->scopeCamId = id;
}

int Miniscope::scopeCamId() const
{
    return d->scopeCamId;
}

bool Miniscope::openCamera()
{
    if (d->connected)
        std::cerr << "Trying to open an already opened camera connection. This is likely not intended." << std::endl;

    // Use V4L on Linux, as apparently the GStreamer backend, if automatically chosen,
    // has issues with some properties of the Miniscope camera and will refuse to
    // grab any proper frame.
    // On Windows on the other hand, the DirectShow backend seems to be the best option.
    // If any of them fail, just try the API autodetection in OpenCV.
    auto apiPreference = cv::CAP_ANY;
#ifdef __linux__
    apiPreference = cv::CAP_V4L2;
#elif defined(_WIN64) || defined(_WIN32)
    apiPreference = cv::CAP_DSHOW;
#endif
    auto ret = d->cam.open(d->scopeCamId, apiPreference);
    if (!ret) {
        // we failed opening the camera - try again using OpenCV's backend autodetection
        emitMessage("Unable to use preferred camera backend, falling back to autodetection.");
        ret = d->cam.open(d->scopeCamId);
    }

    // if a connection attempt succeeded, we can now consider the camera
    // to be connected
    if (ret)
        d->connected = true;

    return ret;
}

bool Miniscope::connect()
{
    if (d->connected) {
        if (d->failed) {
            disconnect();
        } else {
            std::cerr << "Tried to reconnect already connected camera." << std::endl;
            return false;
        }
    }

    if (!openCamera()) {
        fail("Unable to connect to Miniscope camera. Is the DAQ board connected?");
        return false;
    }

    d->failed = false;
    d->connected = true;

    emitMessage(QStringLiteral("Initialized camera %1").arg(d->scopeCamId));
    return true;
}

void Miniscope::disconnect()
{
    stop();
    d->cam.release();
    if (d->connected)
        emitMessage(QStringLiteral("Disconnected camera %1").arg(d->scopeCamId));
    d->connected = false;
}

QList<ControlDefinition> Miniscope::controls() const
{
    return d->controls;
}

bool Miniscope::run()
{
    if (!d->connected)
        return false;
    if (d->failed) {
        // try to recover from failed state by reconnecting
        emitMessage("Reconnecting to recover from previous failure.");
        disconnect();
        if (!connect())
            return false;
    }

    startCaptureThread();
    return true;
}

void Miniscope::stop()
{
    d->running = false;
    d->recording = false;
    finishCaptureThread();
}

bool Miniscope::startRecording(const QString &fname)
{
    if (!d->connected)
        return false;
    if (!d->running) {
        if (!run())
            return false;
    }

    if (!fname.isEmpty())
        d->videoFname = fname;
    d->recording = true;

    return true;
}

void Miniscope::stopRecording()
{
    d->recording = false;
}

bool Miniscope::running() const
{
    return d->running;
}

bool Miniscope::recording() const
{
    return d->running && d->recording;
}

bool Miniscope::captureStartTimeInitialized() const
{
    return d->captureStartTimeInitialized;
}

void Miniscope::setPrintMessagesToStdout(bool enabled)
{
    d->printMessagesToStdout = enabled;
}

void Miniscope::setVisibleChannels(bool red, bool green, bool blue)
{
    d->showRed = red;
    d->showGreen = green;
    d->showBlue = blue;
}

bool Miniscope::showRedChannel() const
{
    return d->showRed;
}

bool Miniscope::showGreenChannel() const
{
    return d->showGreen;
}

bool Miniscope::showBlueChannel() const
{
    return d->showBlue;
}

void Miniscope::setOnFrame(MScope::RawFrameCallback callback, void *udata)
{
    d->frameCallback = std::make_pair(callback, udata);
}

void Miniscope::setOnDisplayFrame(DisplayFrameCallback callback, void *udata)
{
    d->displayFrameCallback = std::make_pair(callback, udata);
}

cv::Mat Miniscope::currentDisplayFrame()
{
    std::lock_guard<std::mutex> lock(d->frameRingMutex);
    cv::Mat frame;
    const auto queueSize = d->displayQueue.size();
    if (queueSize == 0)
        return frame;
    return d->displayQueue.dequeue();
}

uint Miniscope::currentFps() const
{
    return d->currentFPS;
}

size_t Miniscope::droppedFramesCount() const
{
    return d->droppedFramesCount;
}

uint Miniscope::fps() const
{
    return d->fps;
}

void Miniscope::setFps(uint fps)
{
    d->fps = fps;
}

void Miniscope::setCaptureStartTime(const std::chrono::time_point<std::chrono::steady_clock>& startTime)
{
    // changing the start timestamp is protected
    const std::lock_guard<std::mutex> lock(d->timeMutex);

    d->startTimepoint = startTime;
    d->useUnixTime = false; // we have a custom start time, no UNIX epoch will be used
    d->unixCaptureStartTime = milliseconds_t(0);
    d->captureStartTimeInitialized = false; // reinitialize frame time with new start time, in case we are already running
}

bool Miniscope::useUnixTimestamps() const
{
    return d->useUnixTime;
}

void Miniscope::setUseUnixTimestamps(bool useUnixTime)
{
    // changing the start timestamp is protected
    const std::lock_guard<std::mutex> lock(d->timeMutex);

    d->useUnixTime = useUnixTime;
    d->startTimepoint = std::chrono::time_point<std::chrono::steady_clock>::min();
    d->captureStartTimeInitialized = false; // reinitialize frame time with new start time, in case we are already running
}

milliseconds_t Miniscope::unixCaptureStartTime() const
{
    return d->unixCaptureStartTime;
}

bool Miniscope::externalRecordTrigger() const
{
    return d->checkRecTrigger;
}

void Miniscope::setExternalRecordTrigger(bool enabled)
{
    d->checkRecTrigger = enabled;
}

QString Miniscope::videoFilename() const
{
    return d->videoFname;
}

void Miniscope::setVideoFilename(const QString& fname)
{
    // TODO: Maybe mutex this, to prevent API users from doing the wrong thing
    // and checking the value directly after the recording was started?
    d->videoFname = fname;
}

VideoCodec Miniscope::videoCodec() const
{
    return d->videoCodec;
}

void Miniscope::setVideoCodec(VideoCodec codec)
{
    d->videoCodec = codec;
}

VideoContainer Miniscope::videoContainer() const
{
    return d->videoContainer;
}

void Miniscope::setVideoContainer(VideoContainer container)
{
    d->videoContainer = container;
}

bool Miniscope::recordLossless() const
{
    return d->recordLossless;
}

void Miniscope::setRecordLossless(bool lossless)
{
    d->recordLossless = lossless;
}

int Miniscope::minFluorDisplay() const
{
    return d->minFluorDisplay;
}

void Miniscope::setMinFluorDisplay(int value)
{
    d->minFluorDisplay = value;
}

int Miniscope::maxFluorDisplay() const
{
    return d->maxFluorDisplay;
}

void Miniscope::setMaxFluorDisplay(int value)
{
    d->maxFluorDisplay = value;
}

int Miniscope::minFluor() const
{
    return d->minFluor;
}

int Miniscope::maxFluor() const
{
    return d->maxFluor;
}

BackgroundDiffMethod Miniscope::displayBgDiffMethod() const
{
    return d->bgDiffMethod;
}

void Miniscope::setDisplayBgDiffMethod(BackgroundDiffMethod method)
{
    d->bgDiffMethod = method;
}

double Miniscope::bgAccumulateAlpha() const
{
    return d->bgAccumulateAlpha;
}

void Miniscope::setBgAccumulateAlpha(double value)
{
    if (value > 1)
        value = 1;
    d->bgAccumulateAlpha = value;
}

uint Miniscope::recordingSliceInterval() const
{
    return d->recordingSliceInterval;
}

void Miniscope::setRecordingSliceInterval(uint minutes)
{
    d->recordingSliceInterval = minutes;
}

QString Miniscope::lastError() const
{
    return d->lastError;
}

milliseconds_t Miniscope::lastRecordedFrameTime() const
{
    return d->lastRecordedFrameTime;
}

void Miniscope::setLed(double value)
{
    // sanitize value
    if (value > 100)
        value = 100;

    // NOTE: With V4L, max value seems to be 125 here
    double ledPower = value * 1.25;
    if (d->connected) {
        d->cam.set(cv::CAP_PROP_HUE, ledPower);
    }
}

void Miniscope::addDisplayFrameToBuffer(const cv::Mat &frame, const milliseconds_t &timestamp)
{
    // call potential callback on this possibly edited "to be displayed" frame
    const auto displayFrameCB = d->displayFrameCallback.first;
    if (displayFrameCB != nullptr)
        displayFrameCB(frame, timestamp, d->displayFrameCallback.second);

    // the display frame queue is protected
    std::lock_guard<std::mutex> lock(d->frameRingMutex);

    // drop frames if we are displaying too slowly, otherwise add new stuff to queue
    if (d->displayQueue.size() < 28)
        d->displayQueue.enqueue(frame);
}

void Miniscope::captureThread(void* msPtr)
{
    const auto self = static_cast<Miniscope*> (msPtr);
    const auto d = self->d.get();

    // unpack raw frame callback pair
    const auto frameCB = d->frameCallback.first;
    auto frameCB_udata = d->frameCallback.second;

    // make a dummy "dropped frame" matrix to display when we drop frames
    cv::Mat droppedFrameImage(cv::Size(752, 480), CV_8UC3);
    droppedFrameImage.setTo(cv::Scalar(255, 0, 0));
    cv::putText(droppedFrameImage,
                "Frame Dropped!",
                cv::Point(24, 240),
                cv::FONT_HERSHEY_COMPLEX,
                1.5,
                cv::Scalar(255,255,255));

    d->droppedFramesCount = 0;
    d->currentFPS = static_cast<uint>(d->fps);

    // reset errors
    d->failed = false;
    d->lastError.clear();

    // prepare accumulator image for running average (for dF/F)
    cv::Mat accumulatedMat;

    // prepare for recording
    d->cam.set(cv::CAP_PROP_FPS, d->fps);
    std::unique_ptr<VideoWriter> vwriter(new VideoWriter());
    auto recordFrames = false;

    // use custom timepoint as start time, in case we have one set - use current time otherwise
    auto threadStartTime = std::chrono::steady_clock::now();
    auto driverStartTimestamp = milliseconds_t(0);

    // use UNIX timestamp as start time, in case that's the desired setting
    const auto captureStartUnixTime = std::chrono::duration_cast<milliseconds_t>(std::chrono::system_clock::now().time_since_epoch());
    if (d->useUnixTime) {
        // FIXME: When running on Windows, we get the Windows system time instead, which makes the returned files non-portable
        // between operating systems. Not ideal, so we should actually universally convert this to UNIX time here.
        threadStartTime = std::chrono::steady_clock::now() + captureStartUnixTime;
    }

    // align our start time, initially
    d->captureStartTimeInitialized = false;

    while (d->running) {
        cv::Mat frame;
        const auto cycleStartTime = std::chrono::steady_clock::now();

        // protect against race-condition when the `captureStartTimeInitialized` var is changed
        // while we are grabbing a frame, or in any other intermediate state before we have
        // initialized the timestamps
        const auto reinitStartTime = !d->captureStartTimeInitialized;

        // we set the start time here, as the start time may be changed while the thread is already running
        if (reinitStartTime) {
            // this is a critical section, we do not want the capture start time to be changed from outside this
            // thread while working with it
            const std::lock_guard<std::mutex> lock(d->timeMutex);
            if (d->startTimepoint > std::chrono::time_point<std::chrono::steady_clock>::min())
                threadStartTime = d->startTimepoint;
            d->captureStartTimeInitialized = true;
        }

        // acquire a timestamp when we received the frame on our clock, as well as retrieving the driver/device
        // timestamp in milliseconds
        const auto __stime = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - threadStartTime);
        auto status = d->cam.grab();
        auto masterRecvTimestamp = std::chrono::round<milliseconds_t>((__stime + std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - threadStartTime)) / 2.0);
        const auto driverFrameTimestamp = std::chrono::milliseconds (static_cast<long>(d->cam.get(cv::CAP_PROP_POS_MSEC)));

        if (reinitStartTime) {
            // perform timestamp sanity check - occasionally we get bad timestamps, and we must not
            // initialize our timer with those.
            if (driverFrameTimestamp.count() <= 0) {
                self->emitMessage("Frame with timestamp of zero ignored for timer initialization.");

                // We fail quickly here in case we actually failed to grab a falid frame (status == false),
                // which may indicate issues in the DAQ board itself or connectivity problems.
                // Otherwise we fail after 20 dropped frames.
                d->droppedFramesCount++;
                if (d->droppedFramesCount >= 4) {
                    if (!status) {
                        self->fail("Unable to grab valid frames for initialization. (You may try to physically reconnect the DAQ board to resolve this issue)");
                        break;
                    } else if (d->droppedFramesCount >= 20) {
                        self->fail("Unable to get valid timestamps for initialization.");
                        break;
                    }
                }

                d->captureStartTimeInitialized = false;
                continue;
            }
            d->droppedFramesCount = 0;

            if (d->useUnixTime) {
                // apply a UNIX-time offset to make all subsequent timestamps be UNIX timestamps
                driverStartTimestamp = driverFrameTimestamp - captureStartUnixTime - milliseconds_t(static_cast<long>(1000.0 / d->fps));
                threadStartTime = threadStartTime + (masterRecvTimestamp - captureStartUnixTime - milliseconds_t(static_cast<long>(1000.0 / d->fps)));
                masterRecvTimestamp = masterRecvTimestamp + captureStartUnixTime;
            } else {
                driverStartTimestamp = driverFrameTimestamp - (masterRecvTimestamp - milliseconds_t(static_cast<long>(1000.0 / d->fps)));
            }
        }
        const auto frameDeviceTimestamp = driverFrameTimestamp - driverStartTimestamp;

        if (!status) {
            self->fail("Failed to grab frame.");
            break;
        }

        try {
            status = d->cam.retrieve(frame);
            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        } catch (const cv::Exception& e) {
            status = false;
            std::cerr << "Caught OpenCV exception:" << e.what() << std::endl;
        }
        if (!status)
            frame = cv::Mat();

        // call frame callback, so the callee can also adjust the frame
        // timestamp (if it wants to) before we save any data to disk or
        // process it further.
        auto frameTimestamp = frameDeviceTimestamp;
        if (frameCB != nullptr)
            frameCB(frame, frameTimestamp, masterRecvTimestamp, frameDeviceTimestamp, frameCB_udata);

        if (!status) {
            // terminate recording
            d->recording = false;

            self->emitMessage("Dropped frame.");
            self->addDisplayFrameToBuffer(droppedFrameImage, frameTimestamp);
            if (d->droppedFramesCount > 0) {
                // reconnect in case we run into multiple failures when trying
                // to acquire a timestamp
                // NOTE: This behaviour was copied from the original Miniscope DAQ software
                self->emitMessage("Reconnecting Miniscope...");
                d->cam.release();
                d->connected = false;
                if (self->openCamera()) {
                    self->emitMessage("Miniscope reconnected.");
                } else {
                    self->fail("Unable to reopen camera connection.");
                    break;
                }
            }
            d->droppedFramesCount++;

            if (d->droppedFramesCount > 80)
                self->fail("Too many dropped frames. Giving up.");
            continue;
        }

        // prepare video recording if it was enabled while we were running
        if (self->recording()) {
            if (!vwriter->initialized()) {
                self->emitMessage("Recording enabled.");
                // we want to record, but are not initialized yet
                vwriter->setFileSliceInterval(d->recordingSliceInterval);
                vwriter->setCodec(d->videoCodec);
                vwriter->setContainer(d->videoContainer);
                vwriter->setLossless(d->recordLossless);

                try {
                    vwriter->initialize(d->videoFname,
                                        frame.cols,
                                        frame.rows,
                                        static_cast<int>(d->fps),
                                        frame.channels() == 3);
                } catch (const std::runtime_error& e) {
                    self->fail(QStringLiteral("Unable to initialize recording: %1").arg(e.what()));
                    break;
                }

                // we are set for recording and initialized the video writer,
                // so we allow recording frames now
                recordFrames = true;
                self->emitMessage("Initialized video recording.");

                // first frame happens at 0 time elapsed, so we cheat here and manipulate the current frame timestamp,
                // as the current frame will already be added to the recording.
                if (d->useUnixTime) {
                    const auto currentUnixTS = std::chrono::duration_cast<milliseconds_t>(std::chrono::system_clock::now().time_since_epoch());
                    driverStartTimestamp = driverFrameTimestamp - currentUnixTS;
                } else {
                    driverStartTimestamp = driverFrameTimestamp;
                }
                frameTimestamp = driverFrameTimestamp - driverStartTimestamp;
                vwriter->setCaptureStartTimestamp(frameTimestamp);
            }
        } else {
            // we are not recording or stopped recording
            if (recordFrames) {
                // we were recording previously, so finalize the movie and stop adding
                // new frames to the video.
                // Also reset the video writer for a clean start
                vwriter->finalize();
                vwriter.reset(new VideoWriter());
                recordFrames = false;
                self->emitMessage("Recording finalized.");
                d->lastRecordedFrameTime = std::chrono::milliseconds(0);
            }
        }

        // "frame" is the frame that we record to disk, while the "displayFrame"
        // is the one that we may also record as a video file
        cv::Mat displayFrame;
        frame.copyTo(displayFrame);

        // calculate various background differences, if selected
        if (accumulatedMat.rows == 0)
            accumulatedMat = cv::Mat::zeros(frame.rows, frame.cols, CV_32FC(frame.channels()));

        cv::Mat displayF32;
        displayFrame.convertTo(displayF32, CV_32F, 1.0 / 255.0);
        cv::accumulateWeighted(displayF32, accumulatedMat, d->bgAccumulateAlpha);
        if (d->bgDiffMethod == BackgroundDiffMethod::Division) {
            cv::Mat tmpMat;
            cv::divide(displayF32, accumulatedMat, tmpMat, 1, CV_32FC(frame.channels()));
            tmpMat.convertTo(displayFrame, displayFrame.type(), 250.0);
        } else if (d->bgDiffMethod == BackgroundDiffMethod::Subtraction) {
            cv::Mat tmpBgMat;
            accumulatedMat.convertTo(tmpBgMat, CV_8UC1, 255.0);
            cv::subtract(displayFrame, tmpBgMat, displayFrame);
        }

        if (d->useColor) {
            cv::cvtColor(displayFrame, displayFrame, cv::COLOR_GRAY2BGR);

            // we want a colored image
            if (d->showRed || d->showGreen || d->showBlue) {
                cv::Mat bgrChannels[3];
                cv::split(displayFrame, bgrChannels);

                if (!d->showBlue)
                    bgrChannels[0] = cv::Mat::zeros(displayFrame.rows, displayFrame.cols, CV_8UC1);
                if (!d->showGreen)
                    bgrChannels[1] = cv::Mat::zeros(displayFrame.rows, displayFrame.cols, CV_8UC1);
                if (!d->showRed)
                    bgrChannels[2] = cv::Mat::zeros(displayFrame.rows, displayFrame.cols, CV_8UC1);

                cv::merge(bgrChannels, 3, displayFrame);
            }
         } else {
            // grayscale image
            double minF, maxF;
            cv::minMaxLoc(displayFrame, &minF, &maxF);
            d->minFluor = static_cast<int>(minF);
            d->maxFluor = static_cast<int>(maxF);

            displayFrame.convertTo(displayFrame, CV_8U, 255.0 / (d->maxFluorDisplay - d->minFluorDisplay), -d->minFluorDisplay * 255.0 / (d->maxFluorDisplay - d->minFluorDisplay));
        }

        // add display frame to ringbuffer, and record the raw
        // frame to disk if we want to record it.
        self->addDisplayFrameToBuffer(displayFrame, frameTimestamp);
        if (recordFrames) {
            if (!vwriter->pushFrame(frame, frameTimestamp))
                self->fail(QStringLiteral("Unable to send frames to encoder: %1").arg(vwriter->lastError()));
            d->lastRecordedFrameTime = frameTimestamp;
        }

        // wait a bit if necessary, to keep the right framerate
        const auto cycleTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - cycleStartTime);
        const auto extraWaitTime = std::chrono::milliseconds((1000 / d->fps) - cycleTime.count());
        if (extraWaitTime.count() > 0)
            std::this_thread::sleep_for(extraWaitTime);

        const auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - cycleStartTime);
        d->currentFPS = static_cast<uint>(1 / (totalTime.count() / static_cast<double>(1000)));
    }

    // finalize recording (if there was any still ongoing)
    vwriter->finalize();
    d->lastRecordedFrameTime = std::chrono::milliseconds(0);
}
