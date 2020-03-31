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
#include <boost/circular_buffer.hpp>
#include <boost/format.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/videoio.hpp>

#include "definitions.h"
#include "videowriter.h"

namespace MScope
{

#pragma GCC diagnostic ignored "-Wpadded"
class MiniScopeData
{
public:
    MiniScopeData()
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
        frameRing = boost::circular_buffer<cv::Mat>(32);
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
        messageCallback.first = nullptr;
        frameTimestampCallback.first = nullptr;
    }

    std::thread *thread;
    std::mutex mutex;

    cv::VideoCapture cam;
    int scopeCamId;

    double exposure;
    double gain;
    double excitation;
    std::atomic_uint fps;
    std::string videoFname;
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

    boost::circular_buffer<cv::Mat> frameRing;
    std::pair<std::function<void (const cv::Mat&, const milliseconds_t &, void*)>, void*> frameCallback;
    std::pair<std::function<void (const cv::Mat&, const milliseconds_t &, void*)>, void*> displayFrameCallback;
    std::pair<std::function<void (milliseconds_t &, const milliseconds_t &, void*)>, void*> frameTimestampCallback;

    std::pair<std::function<void (const std::string&, void*)>, void*> messageCallback;
    bool printMessagesToStdout;

    bool useColor;
    bool showRed;
    bool showGreen;
    bool showBlue;

    VideoCodec videoCodec;
    VideoContainer videoContainer;
    bool recordLossless;
    uint recordingSliceInterval;

    std::string lastError;
};
#pragma GCC diagnostic pop

} // end of namespace MScope


MiniScope::MiniScope()
    : d(new MiniScopeData())
{
    d->exposure = 100;
    d->gain = 32;
    d->excitation = 1;
    d->fps = 20;
}

MiniScope::~MiniScope()
{
    finishCaptureThread();
    setExcitation(0);
    disconnect();

    delete d;
}

void MiniScope::startCaptureThread()
{
    finishCaptureThread();
    d->running = true;
    d->thread = new std::thread(captureThread, this);
}

void MiniScope::finishCaptureThread()
{
    if (d->thread != nullptr) {
        d->running = false;
        d->thread->join();
        delete d->thread;
        d->thread = nullptr;
    }
}

void MiniScope::emitMessage(const std::string &msg)
{
    if (d->printMessagesToStdout)
        std::cout << msg << std::endl;
    if (!d->messageCallback.first)
        return;

    d->mutex.lock();
    d->messageCallback.first(msg, d->messageCallback.second);
    d->mutex.unlock();
}

void MiniScope::fail(const std::string &msg)
{
    d->recording = false;
    d->running = false;
    d->failed = true;
    d->lastError = msg;
    emitMessage(msg);
}

void MiniScope::setScopeCamId(int id)
{
    d->scopeCamId = id;
}

int MiniScope::scopeCamId() const
{
    return d->scopeCamId;
}

void MiniScope::setExposure(double value)
{
    if (floor(value) == 0)
        value = 1;
    if (value > 100)
        value = 100;

    // NOTE: With V4L as backend, 255 seems to be the max value here

    d->exposure = value;
    d->cam.set(cv::CAP_PROP_BRIGHTNESS, value * 2.55);
}

double MiniScope::exposure() const
{
    return d->exposure;
}

void MiniScope::setGain(double value)
{
    // NOTE: With V4L as backend, 100 seems to be the max value here

    d->gain = value;
    d->cam.set(cv::CAP_PROP_GAIN, value);
}

double MiniScope::gain() const
{
    return d->gain;
}

void MiniScope::setExcitation(double value)
{
    d->excitation = value;
    setLed(value);
}

double MiniScope::excitation() const
{
    return d->excitation;
}

bool MiniScope::connect()
{
    if (d->connected) {
        if (d->failed) {
            disconnect();
        } else {
            std::cerr << "Tried to reconnect already connected camera." << std::endl;
            return false;
        }
    }

    d->cam.open(d->scopeCamId);

    d->cam.set(cv::CAP_PROP_SATURATION, SET_CMOS_SETTINGS); // Initiallizes CMOS sensor (FPS, gain and exposure enabled...)

    // set default values
    setExposure(100);
    setGain(32);
    setExcitation(1);

    d->failed = false;
    d->connected = true;

    emitMessage(boost::str(boost::format("Initialized camera %1%") % d->scopeCamId));
    return true;
}

void MiniScope::disconnect()
{
    stop();
    d->cam.release();
    if (d->connected)
        emitMessage(boost::str(boost::format("Disconnected camera %1%") % d->scopeCamId));
    d->connected = false;
}

bool MiniScope::run()
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

void MiniScope::stop()
{
    d->running = false;
    d->recording = false;
    finishCaptureThread();
}

bool MiniScope::startRecording(const std::string &fname)
{
    if (!d->connected)
        return false;
    if (!d->running) {
        if (!run())
            return false;
    }

    if (!fname.empty())
        d->videoFname = fname;
    d->recording = true;

    return true;
}

void MiniScope::stopRecording()
{
    d->recording = false;
}

bool MiniScope::running() const
{
    return d->running;
}

bool MiniScope::recording() const
{
    return d->running && d->recording;
}

bool MiniScope::captureStartTimeInitialized() const
{
    return d->captureStartTimeInitialized;
}

void MiniScope::setOnMessage(std::function<void(const std::string &, void *)> callback, void *udata)
{
    d->messageCallback = std::make_pair(callback, udata);
}

void MiniScope::setPrintMessagesToStdout(bool enabled)
{
    d->printMessagesToStdout = enabled;
}

bool MiniScope::useColor() const
{
    return d->useColor;
}

void MiniScope::setUseColor(bool color)
{
    d->useColor = color;
}

void MiniScope::setVisibleChannels(bool red, bool green, bool blue)
{
    d->showRed = red;
    d->showGreen = green;
    d->showBlue = blue;
}

bool MiniScope::showRedChannel() const
{
    return d->showRed;
}

bool MiniScope::showGreenChannel() const
{
    return d->showGreen;
}

bool MiniScope::showBlueChannel() const
{
    return d->showBlue;
}

void MiniScope::setOnFrame(std::function<void (const cv::Mat &, const milliseconds_t &, void *)> callback, void *udata)
{
    d->frameCallback = std::make_pair(callback, udata);
}

void MiniScope::setOnDisplayFrame(std::function<void (const cv::Mat &, const milliseconds_t &, void *)> callback, void *udata)
{
    d->displayFrameCallback = std::make_pair(callback, udata);
}

void MiniScope::setOnFrameTimestamp(std::function<void (milliseconds_t &, const milliseconds_t &, void *)> callback, void *udata)
{
    d->frameTimestampCallback = std::make_pair(callback, udata);
}

cv::Mat MiniScope::currentDisplayFrame()
{
    std::lock_guard<std::mutex> lock(d->mutex);
    cv::Mat frame;
    if (d->frameRing.size() == 0)
        return frame;

    frame = d->frameRing.front();
    d->frameRing.pop_front();
    return frame;
}

uint MiniScope::currentFps() const
{
    return d->currentFPS;
}

size_t MiniScope::droppedFramesCount() const
{
    return d->droppedFramesCount;
}

uint MiniScope::fps() const
{
    return d->fps;
}

void MiniScope::setFps(uint fps)
{
    d->fps = fps;
}

void MiniScope::setCaptureStartTime(const std::chrono::time_point<std::chrono::steady_clock>& startTime)
{
    // changing the start timestamp is protected
    const std::lock_guard<std::mutex> lock(d->mutex);

    d->startTimepoint = startTime;
    d->useUnixTime = false; // we have a custom start time, no UNIX epoch will be used
    d->unixCaptureStartTime = milliseconds_t(0);
    d->captureStartTimeInitialized = false; // reinitialize frame time with new start time, in case we are already running
}

bool MiniScope::useUnixTimestamps() const
{
    return d->useUnixTime;
}

void MiniScope::setUseUnixTimestamps(bool useUnixTime)
{
    // changing the start timestamp is protected
    const std::lock_guard<std::mutex> lock(d->mutex);

    d->useUnixTime = useUnixTime;
    d->startTimepoint = std::chrono::time_point<std::chrono::steady_clock>::min();
    d->captureStartTimeInitialized = false; // reinitialize frame time with new start time, in case we are already running
}

milliseconds_t MiniScope::unixCaptureStartTime() const
{
    return d->unixCaptureStartTime;
}

bool MiniScope::externalRecordTrigger() const
{
    return d->checkRecTrigger;
}

void MiniScope::setExternalRecordTrigger(bool enabled)
{
    d->checkRecTrigger = enabled;
}

std::string MiniScope::videoFilename() const
{
    return d->videoFname;
}

void MiniScope::setVideoFilename(const std::string &fname)
{
    // TODO: Maybe mutex this, to prevent API users from doing the wrong thing
    // and checking the value directly after the recording was started?
    d->videoFname = fname;
}

VideoCodec MiniScope::videoCodec() const
{
    return d->videoCodec;
}

void MiniScope::setVideoCodec(VideoCodec codec)
{
    d->videoCodec = codec;
}

VideoContainer MiniScope::videoContainer() const
{
    return d->videoContainer;
}

void MiniScope::setVideoContainer(VideoContainer container)
{
    d->videoContainer = container;
}

bool MiniScope::recordLossless() const
{
    return d->recordLossless;
}

void MiniScope::setRecordLossless(bool lossless)
{
    d->recordLossless = lossless;
}

int MiniScope::minFluorDisplay() const
{
    return d->minFluorDisplay;
}

void MiniScope::setMinFluorDisplay(int value)
{
    d->minFluorDisplay = value;
}

int MiniScope::maxFluorDisplay() const
{
    return d->maxFluorDisplay;
}

void MiniScope::setMaxFluorDisplay(int value)
{
    d->maxFluorDisplay = value;
}

int MiniScope::minFluor() const
{
    return d->minFluor;
}

int MiniScope::maxFluor() const
{
    return d->maxFluor;
}

BackgroundDiffMethod MiniScope::displayBgDiffMethod() const
{
    return d->bgDiffMethod;
}

void MiniScope::setDisplayBgDiffMethod(BackgroundDiffMethod method)
{
    d->bgDiffMethod = method;
}

double MiniScope::bgAccumulateAlpha() const
{
    return d->bgAccumulateAlpha;
}

void MiniScope::setBgAccumulateAlpha(double value)
{
    if (value > 1)
        value = 1;
    d->bgAccumulateAlpha = value;
}

uint MiniScope::recordingSliceInterval() const
{
    return d->recordingSliceInterval;
}

void MiniScope::setRecordingSliceInterval(uint minutes)
{
    d->recordingSliceInterval = minutes;
}

std::string MiniScope::lastError() const
{
    return d->lastError;
}

milliseconds_t MiniScope::lastRecordedFrameTime() const
{
    return d->lastRecordedFrameTime;
}

void MiniScope::setLed(double value)
{
    // sanitize value
    if (value > 100)
        value = 100;

    // NOTE: With V4L, max value seems to be 125 here
    double ledPower = value * 0.8;
    if (d->connected) {
        d->cam.set(cv::CAP_PROP_HUE, ledPower);
    }
}

void MiniScope::addDisplayFrameToBuffer(const cv::Mat &frame, const milliseconds_t &timestamp)
{
    std::lock_guard<std::mutex> lock(d->mutex);
    // call potential callback on this possibly edited "to be displayed" frame
    const auto displayFrameCB = d->displayFrameCallback.first;
    if (displayFrameCB != nullptr)
        displayFrameCB(frame, timestamp, d->displayFrameCallback.second);

    d->frameRing.push_back(frame);
}

void MiniScope::captureThread(void* msPtr)
{
    const auto self = static_cast<MiniScope*> (msPtr);
    const auto d = self->d;

    // unpack timestamp callback pair
    const auto frameTimestampCB = d->frameTimestampCallback.first;
    auto frameTimestampCB_udata = d->frameTimestampCallback.second;

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

        // check if we might want to trigger a recording start via external input
        if (d->checkRecTrigger) {
            auto temp = static_cast<int>(d->cam.get(cv::CAP_PROP_SATURATION));

            //! std::cout << "GPIO state: " << d->cam.get(cv::CAP_PROP_SATURATION) << std::endl;
            if ((temp & TRIG_RECORD_EXT) == TRIG_RECORD_EXT) {
                if (!d->recording) {
                    // start recording
                    d->recording = true;
                }
            } else {
                // stop recording (if one was running)
                d->recording = false;
            }
        }

        // protect against race-condition when the `captureStartTimeInitialized` var is changed
        // while we are grabbing a frame, or in any other intermediate state before we have
        // initialized the timestamps
        const auto reinitStartTime = !d->captureStartTimeInitialized;

        // we set the start time here, as the start time may be changed while the thread is already running
        if (reinitStartTime) {
            // this is a critical section, we do not want the capture start time to be changed from outside this
            // thread while working with it
            const std::lock_guard<std::mutex> lock(d->mutex);
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

                d->droppedFramesCount++;
                if (d->droppedFramesCount > 20)
                    self->fail("Too many dropped frames. Giving up.");

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
        milliseconds_t frameTimestamp;
        if (frameTimestampCB != nullptr) {
            frameTimestamp = masterRecvTimestamp;
            frameTimestampCB(frameTimestamp, frameDeviceTimestamp, frameTimestampCB_udata);
        } else {
            frameTimestamp = frameDeviceTimestamp;
        }

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

        if (!status) {
            // terminate recording
            d->recording = false;

            d->droppedFramesCount++;
            self->emitMessage("Dropped frame.");
            self->addDisplayFrameToBuffer(droppedFrameImage, frameTimestamp);
            if (d->droppedFramesCount > 0) {
                self->emitMessage("Reconnecting Miniscope...");
                d->cam.release();
                d->cam.open(d->scopeCamId);
                self->emitMessage("Miniscope reconnected.");
            }

            if (d->droppedFramesCount > 80)
                self->fail("Too many dropped frames. Giving up.");
            continue;
        }

        // check if we are too slow, resend settings in case we are
        // NOTE: This behaviour was copied from the original Miniscope DAQ software
        if (d->droppedFramesCount > 0) {
            self->emitMessage("Sending settings again.");
            self->setExposure(d->exposure);
            self->setGain(d->gain);
            self->setExcitation(d->excitation);
            d->droppedFramesCount = 0;
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
                    self->fail(boost::str(boost::format("Unable to initialize recording: %1%") % e.what()));
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

        // call potential callback on the raw acquired frame
        if (frameCB != nullptr)
            frameCB(frame, frameTimestamp, frameCB_udata);

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
                self->fail(boost::str(boost::format("Unable to send frames to encoder: %1%") % vwriter->lastError()));
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
