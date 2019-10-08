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

#include <string>
#include <sstream>

#include "miniscope.h"

using namespace MScope;

#include <boost/python.hpp>
#include "cvmatndsliceconvert.h"
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>
using namespace boost::python;

#pragma GCC diagnostic ignored "-Wold-style-cast"
struct cvmat_to_ndarray
{
    static PyObject* convert(const cv::Mat& mat)
    {
        return cvMatToNDArray(mat);
    }
    static PyTypeObject const* get_pytype()
    {
        return &PyArray_Type;
    }
};
#pragma GCC diagnostic pop

BOOST_PYTHON_MODULE(miniscope)
{
    initNDArray();
    to_python_converter<cv::Mat, cvmat_to_ndarray, true>();

    enum_<VideoCodec>("VideoCodec")
            .value("UNKNOWN", VideoCodec::Unknown)
            .value("RAW", VideoCodec::Raw)
            .value("FFV1", VideoCodec::FFV1)
            .value("AV1", VideoCodec::AV1)
            .value("VP9", VideoCodec::VP9)
            .value("H265", VideoCodec::H265)
            .value("MPEG4", VideoCodec::MPEG4)
            .export_values()
            ;

    enum_<VideoContainer>("VideoContainer")
            .value("UNKNOWN", VideoContainer::Unknown)
            .value("MATROSKA", VideoContainer::Matroska)
            .value("AVI", VideoContainer::AVI)
            .export_values()
            ;

    enum_<BackgroundDiffMethod>("BackgroundDiffMethod")
            .value("NONE", BackgroundDiffMethod::None)
            .value("SUBTRACTION", BackgroundDiffMethod::Subtraction)
            .value("DIVISION", BackgroundDiffMethod::Division)
            .export_values()
            ;

    class_<MiniScope>("MiniScope")
        .def("set_cam_id", &MiniScope::setScopeCamId, "Set the Miniscope camera ID")

        .def("connect", &MiniScope::connect, "Connect the selected Miniscope")
        .def("disconnect", &MiniScope::disconnect, "Disconnect the selected Miniscope and stop all operations")
        .def("run", &MiniScope::run, "Start image acquisition with the selected settings")
        .def("stop", &MiniScope::stop, "Stop image acquisition")
        .def("start_recording", &MiniScope::startRecording, "Start recording a video file")
        .def("stop_recording", &MiniScope::stopRecording, "Finish the current recording")

        // TODO: This needs special plumbing, so we don't make this function available for now and have
        // all messages printed to stdut when called from a Python script
        .def("set_on_message", &MiniScope::setOnMessage, "Set message handling function")

        .def("set_visible_channels", &MiniScope::setVisibleChannels, "Set which channels (red, green, blue) should be visible")
        .add_property("show_red_channels", &MiniScope::showRedChannel)
        .add_property("show_green_channels", &MiniScope::showGreenChannel)
        .add_property("show_blue_channels", &MiniScope::showBlueChannel)

        .add_property("exposure", &MiniScope::exposure, &MiniScope::setExposure, "Exposure setting")
        .add_property("gain", &MiniScope::gain, &MiniScope::setGain, "Gain setting")
        .add_property("excitation", &MiniScope::excitation, &MiniScope::setExcitation, "Excitation LED power setting")

        .add_property("running", &MiniScope::running, "Is True if we are acquiring images from the Miniscope")
        .add_property("recording", &MiniScope::recording, "Is True if we are recording data")

        .add_property("use_color", &MiniScope::useColor, &MiniScope::setUseColor)

        .add_property("current_frame", &MiniScope::currentFrame, "Retrieve the current frame intended for display. May not be the recorded frame.")
        .add_property("current_fps", &MiniScope::currentFps)
        .add_property("dropped_frames_count", &MiniScope::droppedFramesCount)
        .add_property("last_recorded_frame_time", &MiniScope::lastRecordedFrameTime) // TODO: we need to add a converter for the return type

        .add_property("fps", &MiniScope::fps, &MiniScope::setFps, "Target frames per second")

        .add_property("video_filename", &MiniScope::videoFilename, &MiniScope::setVideoFilename, "The name of the saved video")
        .add_property("video_codec", &MiniScope::videoCodec, &MiniScope::setVideoCodec, "The video codec to use")
        .add_property("video_container", &MiniScope::videoContainer, &MiniScope::setVideoContainer, "The video container to use")
        .add_property("record_lossless", &MiniScope::recordLossless, &MiniScope::setRecordLossless, "Toggle lossless recording, if the codec supports it")

        .add_property("min_fluor_display", &MiniScope::minFluorDisplay, &MiniScope::setMinFluorDisplay, "Minimum fluorescence to display")
        .add_property("max_fluor_display", &MiniScope::maxFluorDisplay, &MiniScope::setMaxFluorDisplay, "Maximum fluorescence to display")
        .add_property("min_fluor", &MiniScope::minFluor, "Minimum fluorescence (pixel value) in the current image")
        .add_property("max_fluor", &MiniScope::maxFluor, "Maximum fluorescence (pixel value) in the current image")

        .add_property("display_bg_diff_method", &MiniScope::displayBgDiffMethod, &MiniScope::setDisplayBgDiffMethod, "Set background elimination method for the displayed image")
        .add_property("bg_accumulate_alpha", &MiniScope::bgAccumulateAlpha, &MiniScope::setBgAccumulateAlpha)

        .add_property("recording_slice_interval", &MiniScope::recordingSliceInterval, &MiniScope::setRecordingSliceInterval, "The interval at which new video files should be started when recording, in minutes")

        .add_property("last_error", &MiniScope::lastError, "Message of the last error, if there was one")
    ;
}
