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

#ifndef VIDEOWRITER_H
#define VIDEOWRITER_H

#include <memory>
#include <opencv2/core.hpp>

enum class VideoContainer {
    Matroska,
    WebM,
    AVI,
    OGV
};

enum class VideoCodec {
    AV1,
    VP9,
    Dirac,
    H264
};

class VideoWriter
{
public:
    VideoWriter();
    ~VideoWriter();

    void initialize(std::string fname, int width, int height, int fps, bool hasColor);
    void finalize(bool writeTrailer = true);
    bool initialized() const;

    bool encodeFrame(const cv::Mat &frame);

    VideoCodec codec() const;
    void setCodec(VideoCodec codec);

    VideoContainer container() const;
    void setContainer(VideoContainer container);

    int width() const;
    int height() const;
    int fps() const;

private:
    class VideoWriterData;
    std::unique_ptr<VideoWriterData> d;

    bool prepareFrame(const cv::Mat &image);
};

#endif // VIDEOWRITER_H
