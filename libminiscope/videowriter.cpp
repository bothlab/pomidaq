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

#include "videowriter.h"

#include <string.h>
#include <boost/format.hpp>
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libavutil/avconfig.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


#pragma GCC diagnostic ignored "-Wpadded"
class VideoWriter::VideoWriterData
{
public:
    VideoWriterData()
    {
        codec = VideoCodec::AV1;
        container = VideoContainer::Matroska;

        frame = nullptr;
        inputFrame = nullptr;
        alignedInput = nullptr;

        octx = nullptr;
        vstrm = nullptr;
        cctx = nullptr;
        swsctx = nullptr;
    }

    VideoCodec codec;
    VideoContainer container;

    bool initialized;
    int width;
    int height;
    AVRational fps;

    AVFrame *frame;
    AVFrame *inputFrame;
    int64_t framePts;
    uchar *alignedInput;

    AVFormatContext *octx;
    AVStream *vstrm;
    AVCodecContext *cctx;
    SwsContext *swsctx;
    AVPixelFormat inputPixFormat;

    ulong frames_n;
};
#pragma GCC diagnostic pop

VideoWriter::VideoWriter()
    : d(new VideoWriterData())
{
}

VideoWriter::~VideoWriter()
{
    finalize(true);
}

/**
 * the following function is a modified version of code
 * found in ffmpeg-0.4.9-pre1/output_example.c
 */
static AVFrame *vw_alloc_frame(int pix_fmt, int width, int height, bool allocate)
{
    AVFrame *aframe;
    uint8_t *aframe_buf;

    aframe = av_frame_alloc();
    if (!aframe)
        return nullptr;

    aframe->format = pix_fmt;
    aframe->width = width;
    aframe->height = height;

    auto size = av_image_get_buffer_size(static_cast<AVPixelFormat>(pix_fmt), width, height, 1);
    if (allocate) {
        aframe_buf = static_cast<uint8_t*>(malloc(static_cast<size_t>(size)));
        if (!aframe_buf) {
            av_free(aframe);
            return nullptr;
        }
        av_image_fill_arrays(aframe->data, aframe->linesize, aframe_buf, static_cast<AVPixelFormat>(pix_fmt), width, height, 1);
    }

    return aframe;
}

void VideoWriter::initialize(std::string fname, int width, int height, int fps, bool hasColor)
{
    if (d->initialized)
        throw std::runtime_error("Tried to initialize an already initialized video writer.");

    d->width = width;
    d->height = height;
    d->fps = {fps, 1};
    d->frames_n = 0;

    int ret;

    // open output format context
    d->octx = nullptr;
    ret = avformat_alloc_output_context2(&d->octx, nullptr, nullptr, fname.c_str());
    if (ret < 0)
        throw std::runtime_error(boost::str(boost::format("Failed to allocate output context: %1%") % ret));

    // open output IO context
    ret = avio_open2(&d->octx->pb, fname.c_str(), AVIO_FLAG_WRITE, nullptr, nullptr);
    if (ret < 0) {
        finalize(false);
        throw std::runtime_error(boost::str(boost::format("Failed to open output I/O context: %1%") % ret));
    }

    // set codec parameters
    auto vcodec = avcodec_find_encoder(AV_CODEC_ID_VP9);
    d->cctx = avcodec_alloc_context3(vcodec);
    d->cctx->pix_fmt = vcodec->pix_fmts[0];
    d->cctx->time_base = av_inv_q(d->fps);
    d->cctx->width = d->width;
    d->cctx->height = d->height;
    d->cctx->framerate = d->fps;
    d->cctx->workaround_bugs = FF_BUG_AUTODETECT;

    if (d->octx->oformat->flags & AVFMT_GLOBALHEADER)
        d->cctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    // create new video stream
    d->vstrm = avformat_new_stream(d->octx, vcodec);
    if (!d->vstrm)
        throw std::runtime_error("Failed to create new video stream.");
    avcodec_parameters_from_context(d->vstrm->codecpar, d->cctx);
    d->vstrm->r_frame_rate = d->vstrm->avg_frame_rate = d->fps;

    // open video encoder
    ret = avcodec_open2(d->cctx, vcodec, nullptr);
    if (ret < 0) {
        finalize(false);
        throw std::runtime_error(boost::str(boost::format("Failed to open video encoder: %1%") % ret));
    }

    std::cout
        << "outfile: " << fname << "\n"
        << "format:  " << d->octx->oformat->name << "\n"
        << "vcodec:  " << vcodec->name << "\n"
        << "size:    " << d->width << 'x' << d->height << "\n"
        << "fps:     " << av_q2d(d->fps) << "\n"
        << "color:   " << hasColor << "\n"
        << "pixfmt:  " << av_get_pix_fmt_name(d->cctx->pix_fmt) << "\n"
        << std::flush;

    // initialize sample scaler
    d->inputPixFormat = hasColor? AV_PIX_FMT_BGR24 : AV_PIX_FMT_GRAY8;
    d->swsctx = sws_getCachedContext(nullptr,
                                     d->width,
                                     d->height,
                                     d->inputPixFormat,
                                     d->width,
                                     d->height,
                                     d->cctx->pix_fmt,
                                     SWS_BICUBIC,
                                     nullptr,
                                     nullptr,
                                     nullptr);

    if (!d->swsctx) {
        finalize(false);
        throw std::runtime_error("Failed to initialize sample scaler.");
    }

    // allocate frame buffer for encoding
    d->frame = vw_alloc_frame(d->cctx->pix_fmt, d->width, d->height, true);

    // allocate input buffer for color conversion
    d->inputFrame = vw_alloc_frame(d->cctx->pix_fmt, d->width, d->height, false);

    // write format header, after this we are ready to encode frames
    ret = avformat_write_header(d->octx, nullptr);
    if (ret < 0) {
        finalize(false);
        throw std::runtime_error(boost::str(boost::format("Failed to write format header: %1%") % ret));
    }
    d->framePts = 0;

    d->initialized = true;
}

void VideoWriter::finalize(bool writeTrailer)
{
    if (d->initialized) {
        if (d->vstrm != nullptr)
            avcodec_send_frame(d->cctx, nullptr);

        // write trailer
        if (writeTrailer && (d->octx != nullptr))
            av_write_trailer(d->octx);
    }

    // free all FFmpeg resources
    if (d->frame != nullptr) {
        av_frame_free(&d->frame);
        d->frame = nullptr;
    }
    if (d->inputFrame != nullptr) {
        av_frame_free(&d->inputFrame);
        d->inputFrame = nullptr;
    }

    if (d->cctx != nullptr) {
        avcodec_free_context(&d->cctx);
        d->cctx = nullptr;
    }
    if (d->octx != nullptr) {
        if (d->octx->pb != nullptr)
            avio_close(d->octx->pb);
        avformat_free_context(d->octx);
        d->octx = nullptr;
    }

    if (d->alignedInput != nullptr)
        av_freep(&d->alignedInput);

    d->initialized = false;
}

bool VideoWriter::initialized() const
{
    return d->initialized;
}

bool VideoWriter::prepareFrame(const cv::Mat &image)
{
    const auto channels = image.channels();

    auto step = image.step[0];
    auto data = image.ptr();

    const auto height = image.rows;
    const auto width = image.cols;

    // sanity checks
    if ((static_cast<int>(height) > d->height) || (static_cast<int>(width) > d->width))
        throw std::runtime_error(boost::str(boost::format("Received bigger frame than we expected (%1%x%2% instead %3%x%4%)") % width % height % d->width % d->height));
    if ((d->inputPixFormat == AV_PIX_FMT_BGR24) && (channels != 3))
        return false;
    else if ((d->inputPixFormat == AV_PIX_FMT_GRAY8) && (channels != 1))
        return false;

    // FFmpeg contains SIMD optimizations which can sometimes read data past
    // the supplied input buffer. To ensure that doesn't happen, we pad the
    // step to a multiple of 32 (that's the minimal alignment for which Valgrind
    // doesn't raise any warnings).
    const size_t STEP_ALIGNMENT = 32;
    if (step % STEP_ALIGNMENT != 0) {
        auto aligned_step = (step + STEP_ALIGNMENT - 1) & -STEP_ALIGNMENT;

        if (d->alignedInput == nullptr)
            d->alignedInput = static_cast<uchar*>(av_mallocz(aligned_step * static_cast<size_t>(height)));

        for (size_t y = 0; y < static_cast<size_t>(height); y++)
            memcpy(d->alignedInput + y*aligned_step, image.ptr() + y*step, step);

        data = d->alignedInput;
        step = aligned_step;
    }

    if (d->cctx->pix_fmt != d->inputPixFormat) {
        // let input_picture point to the raw data buffer of 'image'
        av_image_fill_arrays(d->inputFrame->data, d->inputFrame->linesize, static_cast<const uint8_t*>(data), d->inputPixFormat, width, height, 1);
        d->inputFrame->linesize[0] = static_cast<int>(step);

        if (sws_scale(d->swsctx, d->inputFrame->data,
                               d->inputFrame->linesize, 0,
                               d->height,
                               d->frame->data, d->frame->linesize) < 0)
                    return false;


    } else {
        av_image_fill_arrays(d->frame->data, d->frame->linesize, static_cast<const uint8_t*>(data), d->inputPixFormat, width, height, 1);
        d->frame->linesize[0] = static_cast<int>(step);
    }

    d->frame->pts = d->framePts++;
    return true;
}

bool VideoWriter::encodeFrame(const cv::Mat &frame)
{
    int ret;

    if (!prepareFrame(frame)) {
        std::cerr << "Unable to prepare frame. N:" << d->frames_n + 1 << std::endl;
        return false;
    }

    // encode video frame
    ret = avcodec_send_frame(d->cctx, d->frame);
    if (ret < 0) {
        std::cerr << "Unable to send frame to encoder. N:" << d->frames_n + 1 << std::endl;
        return false;
    }

    AVPacket pkt;
    pkt.data = nullptr;
    pkt.size = 0;
    av_init_packet(&pkt);
    ret = avcodec_receive_packet(d->cctx, &pkt);
    if (ret != 0) {
        std::cerr << "Unable to encode frame." << std::endl;
        return false;
    }

    // rescale packet timestamp
    pkt.duration = 1;
    av_packet_rescale_ts(&pkt, d->cctx->time_base, d->vstrm->time_base);
    // write packet
    av_write_frame(d->octx, &pkt);
    d->frames_n++;
    av_packet_unref(&pkt);

    return true;
}

VideoCodec VideoWriter::codec() const
{
    return d->codec;
}

void VideoWriter::setCodec(VideoCodec codec)
{
    d->codec = codec;
}

VideoContainer VideoWriter::container() const
{
    return d->container;
}

int VideoWriter::width() const
{
    return d->width;
}

int VideoWriter::height() const
{
    return d->height;
}

int VideoWriter::fps() const
{
    return d->fps.num;
}

void VideoWriter::setContainer(VideoContainer container)
{
    d->container = container;
}
