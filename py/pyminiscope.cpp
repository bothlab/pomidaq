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

#include <string>
#include <sstream>

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <pybind11/chrono.h>
#include "qstringtopy.h"
#include "cvmatndsliceconvert.h"
#include "miniscope.h"

using namespace MScope;
namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::vector<ControlDefinition>);
PYBIND11_MAKE_OPAQUE(std::vector<double>);

PYBIND11_MODULE(miniscope, m) {
    m.doc() = "Access a Miniscope through Python"; // optional module docstring

    NDArrayConverter::initNDArray();
    py::bind_vector<std::vector<double>>(m, "VectorDouble");
    py::bind_vector<std::vector<ControlDefinition>>(m, "VectorControlDefinition");

    py::enum_<VideoCodec>(m, "VideoCodec", py::arithmetic())
            .value("UNKNOWN", VideoCodec::Unknown)
            .value("RAW", VideoCodec::Raw)
            .value("FFV1", VideoCodec::FFV1)
            .value("AV1", VideoCodec::AV1)
            .value("VP9", VideoCodec::VP9)
            .value("HEVC", VideoCodec::HEVC)
            .value("MPEG4", VideoCodec::MPEG4)
            .export_values()
    ;

    py::enum_<VideoContainer>(m, "VideoContainer", py::arithmetic())
            .value("UNKNOWN", VideoContainer::Unknown)
            .value("MATROSKA", VideoContainer::Matroska)
            .value("AVI", VideoContainer::AVI)
            .export_values()
    ;

    py::enum_<DisplayMode>(m, "DisplayMode", py::arithmetic())
            .value("RAW_FRAMES", DisplayMode::RawFrames)
            .value("BACKGROUND_DIFF", DisplayMode::BackgroundDiff)
            .export_values()
    ;

    py::enum_<ControlKind>(m, "ControlKind", py::arithmetic())
            .value("UNKNOWN", ControlKind::Unknown)
            .value("SELECTOR", ControlKind::Selector)
            .value("SLIDER", ControlKind::Slider)
            .export_values()
    ;

    py::class_<ControlDefinition>(m, "ControlDefinition")
        .def(py::init<>())

        .def_readwrite("kind", &ControlDefinition::kind, "Type of this control")
        .def_readwrite("id", &ControlDefinition::id, "Identifier of this control")
        .def_readwrite("name", &ControlDefinition::name, "Human-readable name for this control")

        .def_readwrite("value_min", &ControlDefinition::valueMin, "Minimum possible value for this control")
        .def_readwrite("value_max", &ControlDefinition::valueMax, "Maximum possible value for this control")
        .def_readwrite("value_start", &ControlDefinition::valueStart, "Initial value for this control")
        .def_readwrite("step_size", &ControlDefinition::stepSize, "Size of a single value step")

        .def_readwrite("labels", &ControlDefinition::labels, "Labels for individual values (mostly used for ControlKind.SELECTOR types, their index can be set as control value)")
        .def_readwrite("values", &ControlDefinition::values, "Possible values for this control")
    ;

    py::class_<Miniscope>(m, "Miniscope")
        .def(py::init<>())

        .def_property_readonly("available_device_types", &Miniscope::availableDeviceTypes, "Get a list of all Miniscope variants we can communicate with")
        .def("load_device_config", &Miniscope::loadDeviceConfig, "Load harware definition for a given Miniscope device type")

        .def_property_readonly("device_type", &Miniscope::deviceType, "get the name of the currently loaded Miniscope device type")
        .def("set_cam_id", &Miniscope::setScopeCamId, "Set the Miniscope camera ID")

        .def("connect", &Miniscope::connect, "Connect the selected Miniscope")
        .def("disconnect", &Miniscope::disconnect, "Disconnect the selected Miniscope and stop all operations")
        .def("run", &Miniscope::run, "Start image acquisition with the selected settings")
        .def("stop", &Miniscope::stop, "Stop image acquisition")
        .def("start_recording", &Miniscope::startRecording, "Start recording a video file")
        .def("stop_recording", &Miniscope::stopRecording, "Finish the current recording")

        .def_property_readonly("controls", &Miniscope::controls, "Get available controls for this device")
        .def("control_value", &Miniscope::controlValue, "Retrieve current control value for the given control ID")
        .def("set_control_value", &Miniscope::setControlValue, "Set new value for control with the given ID")

        .def("set_visible_channels", &Miniscope::setVisibleChannels, "Set which channels (red, green, blue) should be visible")
        .def_property_readonly("show_red_channels", &Miniscope::showRedChannel)
        .def_property_readonly("show_green_channels", &Miniscope::showGreenChannel)
        .def_property_readonly("show_blue_channels", &Miniscope::showBlueChannel)

        .def_property_readonly("is_connected", &Miniscope::isConnected, "Is True if a Miniscope is connected")
        .def_property_readonly("is_running", &Miniscope::isRunning, "Is True if we are acquiring images from the Miniscope")
        .def_property_readonly("is_recording", &Miniscope::isRecording, "Is True if we are recording data")

        .def_property_readonly("current_disp_frame", &Miniscope::currentDisplayFrame, "Retrieve the current frame intended for display. May not be the recorded frame.")
        .def_property_readonly("current_fps", &Miniscope::currentFps)
        .def_property_readonly("dropped_frames_count", &Miniscope::droppedFramesCount)
        .def_property_readonly("last_recorded_frame_time", &Miniscope::lastRecordedFrameTime)

        .def_property("video_filename", &Miniscope::videoFilename, &Miniscope::setVideoFilename, "The name of the saved video")
        .def_property("video_codec", &Miniscope::videoCodec, &Miniscope::setVideoCodec, "The video codec to use")
        .def_property("video_container", &Miniscope::videoContainer, &Miniscope::setVideoContainer, "The video container to use")
        .def_property("record_lossless", &Miniscope::recordLossless, &Miniscope::setRecordLossless, "Toggle lossless recording, if the codec supports it")

        .def_property("min_fluor_display", &Miniscope::minFluorDisplay, &Miniscope::setMinFluorDisplay, "Minimum fluorescence to display")
        .def_property("max_fluor_display", &Miniscope::maxFluorDisplay, &Miniscope::setMaxFluorDisplay, "Maximum fluorescence to display")
        .def_property_readonly("min_fluor", &Miniscope::minFluor, "Minimum fluorescence (pixel value) in the current image")
        .def_property_readonly("max_fluor", &Miniscope::maxFluor, "Maximum fluorescence (pixel value) in the current image")

        .def_property("display_mode", &Miniscope::displayMode, &Miniscope::setDisplayMode, "Set styling mode for the displayed images")
        .def_property("bg_accumulate_alpha", &Miniscope::bgAccumulateAlpha, &Miniscope::setBgAccumulateAlpha)

        .def_property("recording_slice_interval", &Miniscope::recordingSliceInterval, &Miniscope::setRecordingSliceInterval, "The interval at which new video files should be started when recording, in minutes")

        .def("set_print_extra_debug", &Miniscope::setPrintExtraDebug, "Set whether protocol transmission debug messages should be printed to stdout")
        .def_property_readonly("last_error", &Miniscope::lastError, "Message of the last error, if there was one")
    ;
}
