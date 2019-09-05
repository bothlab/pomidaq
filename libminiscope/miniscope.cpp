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

using steady_hr_clock =
    std::conditional<std::chrono::high_resolution_clock::is_steady,
                     std::chrono::high_resolution_clock,
                     std::chrono::steady_clock
                    >::type;

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

        showRed = true;
        showGreen = true;
        showBlue = true;

        bgDiffMethod = BackgroundDiffMethod::NONE;

        minFluorDisplay = 0;
        maxFluorDisplay = 255;

        recordingSliceInterval = 0; // don't slice
        bgAccumulateAlpha = 0.01;
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
    std::atomic<std::chrono::milliseconds> lastRecordedFrameTime;

    boost::circular_buffer<cv::Mat> frameRing;

    std::function<void (std::string)> onMessageCallback;

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
    if (!d->onMessageCallback) {
        std::cout << msg << std::endl;
        return;
    }

    d->mutex.lock();
    d->onMessageCallback(msg);
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
    d->connected = false;
    emitMessage(boost::str(boost::format("Disconnected camera %1%") % d->scopeCamId));
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

void MiniScope::setOnMessage(std::function<void(const std::string&)> callback)
{
    d->onMessageCallback = callback;
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

cv::Mat MiniScope::currentFrame()
{
    std::lock_guard<std::mutex> lock(d->mutex);
    cv::Mat frame;
    if (d->frameRing.size() == 0)
        return frame;

    frame = d->frameRing.front();
    d->frameRing.pop_front();
    return frame;
}

uint MiniScope::currentFPS() const
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
    std::lock_guard<std::mutex> lock(d->mutex);
    d->fps = fps;
    d->cam.set(cv::CAP_PROP_FPS, d->fps);
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

std::chrono::milliseconds MiniScope::lastRecordedFrameTime() const
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

void MiniScope::addFrameToBuffer(const cv::Mat &frame)
{
    std::lock_guard<std::mutex> lock(d->mutex);
    d->frameRing.push_back(frame);
}

void MiniScope::captureThread(void* msPtr)
{
    MiniScope *self = static_cast<MiniScope*> (msPtr);

    cv::Mat droppedFrameImage(cv::Size(752, 480), CV_8UC3);
    droppedFrameImage.setTo(cv::Scalar(255, 0, 0));
    cv::putText(droppedFrameImage,
                "Frame Dropped!",
                cv::Point(24, 240),
                cv::FONT_HERSHEY_COMPLEX,
                1.5,
                cv::Scalar(255,255,255));

    self->d->droppedFramesCount = 0;
    self->d->currentFPS = static_cast<uint>(self->d->fps);

    // reset errors
    self->d->failed = false;
    self->d->lastError.clear();

    // prepare accumulator image for running average (for dF/F)
    cv::Mat accumulatedMat;

    // prepare for recording
    self->d->cam.set(cv::CAP_PROP_FPS, self->d->fps);
    std::unique_ptr<VideoWriter> vwriter(new VideoWriter());
    auto recordFrames = false;
    auto captureStartTime = steady_hr_clock::now();
    bool initCaptureStartTime = true;

    while (self->d->running) {
        cv::Mat frame;
        const auto cycleStartTime = steady_hr_clock::now();

        // check if we might want to trigger a recording start via external input
        if (self->d->checkRecTrigger) {
            auto temp = static_cast<int>(self->d->cam.get(cv::CAP_PROP_SATURATION));

            //! std::cout << "GPIO state: " << self->d->cam.get(cv::CAP_PROP_SATURATION) << std::endl;
            if ((temp & TRIG_RECORD_EXT) == TRIG_RECORD_EXT) {
                if (!self->d->recording) {
                    // start recording
                    self->d->recording = true;
                }
            } else {
                // stop recording (if one was running)
                self->d->recording = false;
            }
        }

        auto status = self->d->cam.grab();
        const auto driverFrameTimestamp = std::chrono::milliseconds (static_cast<long> (self->d->cam.get(cv::CAP_PROP_POS_MSEC)));
        const auto driverFrameTimepoint = std::chrono::time_point<steady_hr_clock> (driverFrameTimestamp);
        if (initCaptureStartTime) {
            initCaptureStartTime = false;
            captureStartTime = driverFrameTimepoint;
        }
        auto frameTimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(driverFrameTimepoint - captureStartTime);
        if (!status) {
            self->fail("Failed to grab frame.");
            break;
        }

        try {
            status = self->d->cam.retrieve(frame);
        } catch (const cv::Exception& e) {
            status = false;
            std::cerr << "Caught OpenCV exception:" << e.what() << std::endl;
        }

        if (!status) {
            // terminate recording
            self->d->recording = false;

            self->d->droppedFramesCount++;
            self->emitMessage("Dropped frame.");
            self->addFrameToBuffer(droppedFrameImage);
            if (self->d->droppedFramesCount > 0) {
                self->emitMessage("Reconnecting Miniscope...");
                self->d->cam.release();
                self->d->cam.open(self->d->scopeCamId);
                self->emitMessage("Miniscope reconnected.");
            }

            if (self->d->droppedFramesCount > 80)
                self->fail("Too many dropped frames. Giving up.");
            continue;
        }

        // check if we are too slow, resend settings in case we are
        // NOTE: This behaviour was copied from the original Miniscope DAQ software
        if ((self->d->droppedFramesCount > 0) || (self->d->currentFPS < self->d->fps / 2.0)) {
            self->emitMessage("Sending settings again.");
            self->setExposure(self->d->exposure);
            self->setGain(self->d->gain);
            self->setExcitation(self->d->excitation);
            self->d->droppedFramesCount = 0;
        }

        // prepare video recording if it was enabled while we were running
        if (self->recording()) {
            if (!vwriter->initialized()) {
                self->emitMessage("Recording enabled.");
                // we want to record, but are not initialized yet
                vwriter->setFileSliceInterval(self->d->recordingSliceInterval);
                vwriter->setCodec(self->d->videoCodec);
                vwriter->setContainer(self->d->videoContainer);
                vwriter->setLossless(self->d->recordLossless);

                try {
                    vwriter->initialize(self->d->videoFname,
                                        frame.cols,
                                        frame.rows,
                                        static_cast<int>(self->d->fps),
                                        frame.channels() == 3);
                } catch (const std::runtime_error& e) {
                    self->fail(boost::str(boost::format("Unable to initialize recording: %1%") % e.what()));
                    break;
                }

                // we are set for recording and initialized the video writer,
                // so we allow recording frames now
                recordFrames = true;
                self->emitMessage("Initialized video recording.");

                // first frame happens at 0 time elapsed, so we cheat here and manipulate the frame timestamp
                captureStartTime = driverFrameTimepoint;
                frameTimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(driverFrameTimepoint - captureStartTime);
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
                self->d->lastRecordedFrameTime = std::chrono::milliseconds(0);
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
        cv::accumulateWeighted(displayF32, accumulatedMat, self->d->bgAccumulateAlpha);
        if (self->d->bgDiffMethod == BackgroundDiffMethod::DIVISION) {
            cv::Mat tmpMat;
            cv::divide(displayF32, accumulatedMat, tmpMat, 1, CV_32FC(frame.channels()));
            tmpMat.convertTo(displayFrame, displayFrame.type(), 250.0);
        } else if (self->d->bgDiffMethod == BackgroundDiffMethod::SUBTRACTION) {
            cv::Mat tmpBgMat;
            accumulatedMat.convertTo(tmpBgMat, CV_8UC1, 255.0);
            cv::subtract(displayFrame, tmpBgMat, displayFrame);
        }

        if (self->d->useColor) {
            // we want a colored image
            if (self->d->showRed || self->d->showGreen || self->d->showBlue) {
                cv::Mat bgrChannels[3];
                cv::split(displayFrame, bgrChannels);

                if (!self->d->showBlue)
                    bgrChannels[0] = cv::Mat::zeros(displayFrame.rows, displayFrame.cols, CV_8UC1);
                if (!self->d->showGreen)
                    bgrChannels[1] = cv::Mat::zeros(displayFrame.rows, displayFrame.cols, CV_8UC1);
                if (!self->d->showRed)
                    bgrChannels[2] = cv::Mat::zeros(displayFrame.rows, displayFrame.cols, CV_8UC1);

                cv::merge(bgrChannels, 3, displayFrame);
            }
         } else {
            // grayscale image
            cv::cvtColor(displayFrame, displayFrame, cv::COLOR_BGR2GRAY);

            double minF, maxF;
            cv::minMaxLoc(displayFrame, &minF, &maxF);
            self->d->minFluor = static_cast<int>(minF);
            self->d->maxFluor = static_cast<int>(maxF);

            displayFrame.convertTo(displayFrame, CV_8U, 255.0 / (self->d->maxFluorDisplay - self->d->minFluorDisplay), -self->d->minFluorDisplay * 255.0 / (self->d->maxFluorDisplay - self->d->minFluorDisplay));
        }

        // add display frame to ringbuffer, and record the raw
        // frame to disk if we want to record it.
        self->addFrameToBuffer(displayFrame);
        if (recordFrames) {
            if (!vwriter->pushFrame(frame, frameTimestamp))
                self->fail(boost::str(boost::format("Unable to send frames to encoder: %1%") % vwriter->lastError()));
            self->d->lastRecordedFrameTime = frameTimestamp;
        }

        // wait a bit if necessary, to keep the right framerate
        const auto cycleTime = std::chrono::duration_cast<std::chrono::milliseconds>(steady_hr_clock::now() - cycleStartTime);
        const auto extraWaitTime = std::chrono::milliseconds((1000 / self->d->fps) - cycleTime.count());
        if (extraWaitTime.count() > 0)
            std::this_thread::sleep_for(extraWaitTime);

        const auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(steady_hr_clock::now() - cycleStartTime);
        self->d->currentFPS = static_cast<uint>(1 / (totalTime.count() / static_cast<double>(1000)));
    }

    // finalize recording (if there was any still ongoing)
    vwriter->finalize();
    self->d->lastRecordedFrameTime = std::chrono::milliseconds(0);
}
