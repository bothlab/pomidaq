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

#ifndef MINISCOPE_H
#define MINISCOPE_H

#include <memory>
#include <functional>
#include <opencv2/core.hpp>

#include "videowriter.h"

#ifdef _WIN32
#define MS_LIB_EXPORT __declspec(dllexport)
#else
#define MS_LIB_EXPORT __attribute__((visibility("default")))
#endif

enum class BackgroundDiffMethod {
    NONE,
    SUBTRACTION,
    DIVISION
};

class MiniScopeData;
class MS_LIB_EXPORT MiniScope
{
public:
    MiniScope();
    ~MiniScope();

    void setScopeCamId(int id);

    void setExposure(double value);
    double exposure() const;

    void setGain(double value);
    double gain() const;

    void setExcitation(double value);
    double excitation() const;

    bool connect();
    void disconnect();

    bool run();
    void stop();
    bool startRecording(const std::string& fname = "");
    void stopRecording();

    bool running() const;
    bool recording() const;

    void setOnMessage(std::function<void(const std::string&)> callback);

    bool useColor() const;
    void setUseColor(bool color);

    void setVisibleChannels(bool red, bool green, bool blue);
    bool showRedChannel() const;
    bool showGreenChannel() const;
    bool showBlueChannel() const;

    cv::Mat currentFrame();
    uint currentFPS() const;
    size_t droppedFramesCount() const;

    uint fps() const;
    void setFps(uint fps);

    bool externalRecordTrigger() const;
    void setExternalRecordTrigger(bool enabled);

    std::string videoFilename() const;
    void setVideoFilename(const std::string& fname);

    VideoCodec videoCodec() const;
    void setVideoCodec(VideoCodec codec);

    VideoContainer videoContainer() const;
    void setVideoContainer(VideoContainer container);

    bool recordLossless() const;
    void setRecordLossless(bool lossless);

    int minFluorDisplay() const;
    void setMinFluorDisplay(int value);

    int maxFluorDisplay() const;
    void setMaxFluorDisplay(int value);

    int minFluor() const;
    int maxFluor() const;

    BackgroundDiffMethod displayBgDiffMethod() const;
    void setDisplayBgDiffMethod(BackgroundDiffMethod method);

    double bgAccumulateAlpha() const;
    void setBgAccumulateAlpha(double value);

    uint recordingSliceInterval() const;
    void setRecordingSliceInterval(uint minutes);

    std::string lastError() const;

    std::chrono::milliseconds lastRecordedFrameTime() const;

private:
    std::unique_ptr<MiniScopeData> d;

    void setLed(double value);
    void addFrameToBuffer(const cv::Mat& frame);
    static void captureThread(void *msPtr);
    void startCaptureThread();
    void finishCaptureThread();
    void emitMessage(const std::string& msg);
    void fail(const std::string& msg);
};

#endif // MINISCOPE_H
