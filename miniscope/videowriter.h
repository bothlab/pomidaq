/*
 * Copyright (C) 2019-2024 Matthias Klumpp <matthias@tenstral.net>
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

#ifndef VIDEOWRITER_H
#define VIDEOWRITER_H

#include <chrono>
#include <expected>
#include <memory>
#include <string>
#include <opencv2/core.hpp>
#include "mediatypes.h"

/**
 * @brief The VideoWriter class
 *
 * A simple class to write OpenCV matrices to disk as quickly as possible,
 * with a pleasant but very simplified API and all the nasty video encoding
 * issues hidden away.
 * This class intentionally supports only few container/codec formats and options.
 */
class VideoWriter
{
public:
    VideoWriter();
    ~VideoWriter();

    std::expected<void, std::string> initialize(
        const std::string &fname,
        int width,
        int height,
        int fps,
        bool hasColor,
        bool saveTimestamps = true);
    void finalize();
    bool initialized() const;

    std::chrono::milliseconds captureStartTimestamp() const;
    void setCaptureStartTimestamp(const std::chrono::milliseconds &startTimestamp);

    /**
     * @brief Queue a frame for encoding.
     *
     * Fails if the queue is full, if the writer does not accept frames,
     * or if the encoder thread ran into an unrecoverable error.
     */
    std::expected<void, std::string> pushFrame(const cv::Mat &frame, const std::chrono::milliseconds &time);

    Miniscope::VideoCodec codec() const;
    void setCodec(Miniscope::VideoCodec codec);

    Miniscope::VideoContainer container() const;
    void setContainer(Miniscope::VideoContainer container);

    int width() const;
    int height() const;
    int fps() const;

    bool lossless() const;
    void setLossless(bool enabled);

    unsigned int fileSliceInterval() const;
    void setFileSliceInterval(unsigned int minutes);

private:
    class Private;
    VideoWriter(const VideoWriter &) = delete;
    VideoWriter &operator=(const VideoWriter &) = delete;
    std::unique_ptr<Private> d;

    std::expected<void, std::string> initializeInternal();
    void finalizeInternal(bool writeTrailer, bool stopRecThread = true);
    static void encodeThread(void *vwPtr);
    bool getNextFrameFromQueue(cv::Mat *frame, std::chrono::milliseconds *timestamp);
    std::expected<void, std::string> prepareFrame(const cv::Mat &inImage);
    std::expected<void, std::string> encodeFrame(const cv::Mat &frame, const std::chrono::milliseconds &timestamp);
    void startEncodeThread();
    void stopEncodeThread();
};

#endif // VIDEOWRITER_H
