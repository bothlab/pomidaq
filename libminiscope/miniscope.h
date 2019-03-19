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

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/hal/hal.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/shape.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/superres.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videostab.hpp>

#define MS_BUFFER_LENGTH 256

class MiniScopeData;
class MiniScope
{
public:
    MiniScope();
    ~MiniScope();

    void setScopeCamId(int id);

    void setExposure(int value);
    int exposure() const;

    void setGain(int value);
    int gain() const;

    void setExcitation(int value);
    int excitation() const;

    bool connect();
    void disconnect();
    bool record();

    bool running() const;

    void setOnMessage(std::function<void(const std::string&)> callback);

    cv::Mat currentFrame();
    uint currentFPS() const;

private:
    std::unique_ptr<MiniScopeData> d;

    void setLed(int value);
    void addFrameToBuffer(const cv::Mat& frame);
    static void captureThread(void *msPtr);
    void startCaptureThread();
    void finishCaptureThread();
    void emitMessage(const std::string& msg);
};

#endif // MINISCOPE_H
