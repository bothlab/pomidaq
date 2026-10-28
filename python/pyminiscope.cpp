/*
 * Copyright (C) 2019-2026 Matthias Klumpp <matthias@tenstral.net>
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

#include "config.h"
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/chrono.h>
#include "cvmatndsliceconvert.h"
#include <miniscope/miniscope.h>

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::vector<Miniscope::ControlDefinition>);
PYBIND11_MAKE_OPAQUE(std::vector<double>);

/**
 * Turn a failed library call into a Python RuntimeError.
 */
static void raiseOnError(const Miniscope::Result &res)
{
    if (!res)
        throw std::runtime_error(res.error());
}

/**
 * Deleter that releases the GIL while the Miniscope is destroyed.
 * Destruction disconnects the device and joins the capture thread, which may
 * still be calling into a Python log handler and therefore needs the GIL.
 */
struct GilReleasingDeleter {
    void operator()(Miniscope::Miniscope *mscope) const
    {
        py::gil_scoped_release release;
        delete mscope;
    }
};
using MiniscopeHolder = std::unique_ptr<Miniscope::Miniscope, GilReleasingDeleter>;

static bool pyIsFinalizing()
{
#if PY_VERSION_HEX >= 0x030D0000
    return Py_IsFinalizing() != 0;
#else
    return _Py_IsFinalizing() != 0;
#endif
}

/**
 * Install a Python callable as libminiscope log handler.
 * The callable is invoked from arbitrary library threads with the GIL held.
 */
static void setPyLogHandler(const py::object &handler)
{
    if (handler.is_none()) {
        Miniscope::resetLogHandler();
        return;
    }
    if (!py::isinstance<py::function>(handler) && !py::hasattr(handler, "__call__"))
        throw py::type_error("Log handler must be callable or None");

    Miniscope::setLogHandler([fn = handler](const Miniscope::LogMessage &lm) {
        // never try to grab the GIL while the interpreter shuts down
        if (pyIsFinalizing())
            return;

        py::gil_scoped_acquire gil;
        try {
            fn(lm.severity, py::str(lm.category), py::str(lm.message.data(), lm.message.size()));
        } catch (py::error_already_set &e) {
            e.discard_as_unraisable("miniscope log handler");
        }
    });
}

/**
 * Holds a Python callable used as library callback.
 * The callback may be invoked and destroyed from threads which do not hold the GIL
 * (e.g. when the Miniscope is destroyed by GilReleasingDeleter), so we acquire it in both cases.
 */
class PyCallback
{
public:
    PyCallback(py::object fn, const char *name)
        : m_fn(std::move(fn)),
          m_name(name)
    {
    }

    ~PyCallback()
    {
        // never try to grab the GIL while the interpreter shuts down, leak the object instead
        if (pyIsFinalizing()) {
            m_fn.release();
            return;
        }
        py::gil_scoped_acquire gil;
        m_fn = py::object();
    }

    PyCallback(const PyCallback &) = delete;
    PyCallback &operator=(const PyCallback &) = delete;

    template<typename... Args>
    void operator()(Args &&...args) const
    {
        if (pyIsFinalizing())
            return;

        py::gil_scoped_acquire gil;
        try {
            m_fn(std::forward<Args>(args)...);
        } catch (py::error_already_set &e) {
            e.discard_as_unraisable(m_name);
        }
    }

private:
    py::object m_fn;
    const char *m_name;
};

/**
 * Wrap a Python callable (or None) for use as library callback.
 * Returns nullptr for None, so the respective callback is unset.
 */
static std::shared_ptr<PyCallback> makePyCallback(const py::object &fn, const char *name)
{
    if (fn.is_none())
        return nullptr;
    if (!py::isinstance<py::function>(fn) && !py::hasattr(fn, "__call__"))
        throw py::type_error("Callback must be callable or None");
    return std::make_shared<PyCallback>(fn, name);
}

/**
 * Drop any Python log handler when the module is unloaded, so the library
 * never calls into a torn-down interpreter and the handler's Python object
 * is released while the GIL is still held.
 */
static void logHandlerModuleCleanup()
{
    Miniscope::resetLogHandler();
}

PYBIND11_MODULE(miniscope, m)
{
    m.doc() = "Access a Miniscope through Python"; // optional module docstring
    m.attr("__version__") = PROJECT_VERSION;

    // fail importing this module if NumPy is not usable, we would crash later otherwise
    if (!NDArrayConverter::initNDArray())
        throw py::error_already_set();
    py::bind_vector<std::vector<double>>(m, "VectorDouble");
    py::bind_vector<std::vector<Miniscope::ControlDefinition>>(m, "VectorControlDefinition");

    py::enum_<Miniscope::VideoCodec>(m, "VideoCodec", py::arithmetic())
        .value("UNKNOWN", Miniscope::VideoCodec::Unknown)
        .value("RAW", Miniscope::VideoCodec::Raw)
        .value("FFV1", Miniscope::VideoCodec::FFV1)
        .value("AV1", Miniscope::VideoCodec::AV1)
        .value("VP9", Miniscope::VideoCodec::VP9)
        .value("HEVC", Miniscope::VideoCodec::HEVC)
        .value("MPEG4", Miniscope::VideoCodec::MPEG4);

    py::enum_<Miniscope::VideoContainer>(m, "VideoContainer", py::arithmetic())
        .value("UNKNOWN", Miniscope::VideoContainer::Unknown)
        .value("MATROSKA", Miniscope::VideoContainer::Matroska)
        .value("AVI", Miniscope::VideoContainer::AVI);

    py::enum_<Miniscope::DisplayMode>(m, "DisplayMode", py::arithmetic())
        .value("RAW_FRAMES", Miniscope::DisplayMode::RawFrames)
        .value("BACKGROUND_DIFF", Miniscope::DisplayMode::BackgroundDiff);

    py::enum_<Miniscope::LogSeverity>(m, "LogSeverity", py::arithmetic())
        .value("DEBUG", Miniscope::LogSeverity::Debug)
        .value("INFO", Miniscope::LogSeverity::Info)
        .value("WARNING", Miniscope::LogSeverity::Warning)
        .value("ERROR", Miniscope::LogSeverity::Error)
        .value("CRITICAL", Miniscope::LogSeverity::Critical);

    m.def(
        "set_log_severity",
        py::overload_cast<Miniscope::LogSeverity>(&Miniscope::setLogSeverity),
        py::arg("min_severity"),
        "Set the minimum severity of messages emitted by all libminiscope log categories");
    m.def(
        "set_log_severity",
        [](const std::string &category, Miniscope::LogSeverity min) {
            Miniscope::setLogSeverity(category.c_str(), min);
        },
        py::arg("category"),
        py::arg("min_severity"),
        "Set the minimum severity of messages emitted by a specific libminiscope log category");
    m.def(
        "set_log_handler",
        &setPyLogHandler,
        py::arg("handler"),
        "Install a callable receiving all libminiscope log messages as (severity, category, message).\n"
        "The callable is invoked from library threads. Pass None to restore the default handler,\n"
        "which prints to stdout/stderr.\n"
        "This should be called once at startup: Changing the handler while acquisition is running is not safe.");
    m.add_object("_log_handler_cleanup", py::capsule(&logHandlerModuleCleanup));

    m.def(
        "video_device_name_from_id",
        &Miniscope::videoDeviceNameFromId,
        py::arg("id"),
        "Get the name of the video device with the given camera ID (currently only supported on Linux)");

    py::enum_<Miniscope::ControlKind>(m, "ControlKind", py::arithmetic())
        .value("UNKNOWN", Miniscope::ControlKind::Unknown)
        .value("SELECTOR", Miniscope::ControlKind::Selector)
        .value("SLIDER", Miniscope::ControlKind::Slider);

    py::class_<Miniscope::ControlDefinition>(m, "ControlDefinition")
        .def(py::init<>())

        .def_readwrite("kind", &Miniscope::ControlDefinition::kind, "Type of this control")
        .def_readwrite("id", &Miniscope::ControlDefinition::id, "Identifier of this control")
        .def_readwrite("name", &Miniscope::ControlDefinition::name, "Human-readable name for this control")

        .def_readwrite("value_min", &Miniscope::ControlDefinition::valueMin, "Minimum possible value for this control")
        .def_readwrite("value_max", &Miniscope::ControlDefinition::valueMax, "Maximum possible value for this control")
        .def_readwrite("value_start", &Miniscope::ControlDefinition::valueStart, "Initial value for this control")
        .def_readwrite("step_size", &Miniscope::ControlDefinition::stepSize, "Size of a single value step")

        .def_readwrite(
            "labels",
            &Miniscope::ControlDefinition::labels,
            "Labels for individual values (mostly used for ControlKind.SELECTOR types, their index can be set as "
            "control value)");

    py::class_<Miniscope::Miniscope, MiniscopeHolder>(m, "Miniscope")
        .def(py::init<>())

        .def_property_readonly(
            "available_device_types",
            py::cpp_function(&Miniscope::Miniscope::availableDeviceTypes, py::call_guard<py::gil_scoped_release>()),
            "Get a list of all Miniscope variants we can communicate with")
        .def(
            "load_device_config",
            [](Miniscope::Miniscope &self, const std::string &deviceType) {
                raiseOnError(self.loadDeviceConfig(deviceType));
            },
            py::call_guard<py::gil_scoped_release>(),
            "Load hardware definition for a given Miniscope device type (raises RuntimeError on failure)")

        .def_property_readonly(
            "device_type",
            &Miniscope::Miniscope::deviceType,
            "get the name of the currently loaded Miniscope device type")
        .def_property(
            "cam_id",
            &Miniscope::Miniscope::scopeCamId,
            &Miniscope::Miniscope::setScopeCamId,
            "ID of the camera device of the Miniscope DAQ board to connect to")

        .def(
            "connect",
            [](Miniscope::Miniscope &self) {
                raiseOnError(self.connect());
            },
            py::call_guard<py::gil_scoped_release>(),
            "Connect the selected Miniscope (raises RuntimeError on failure)")
        .def(
            "disconnect",
            &Miniscope::Miniscope::disconnect,
            py::call_guard<py::gil_scoped_release>(),
            "Disconnect the selected Miniscope and stop all operations")
        .def(
            "hard_reset",
            [](Miniscope::Miniscope &self) {
                raiseOnError(self.hardReset());
            },
            py::call_guard<py::gil_scoped_release>(),
            "Forcefully reset the selected Miniscope DAQ box and make it reboot (raises RuntimeError on failure)")
        .def(
            "run",
            [](Miniscope::Miniscope &self) {
                raiseOnError(self.run());
            },
            py::call_guard<py::gil_scoped_release>(),
            "Start image acquisition with the selected settings (raises RuntimeError on failure)")
        .def("stop", &Miniscope::Miniscope::stop, py::call_guard<py::gil_scoped_release>(), "Stop image acquisition")
        .def(
            "wait_for_result",
            [](Miniscope::Miniscope &self) {
                // wait in small steps while the acquisition thread is active, so signal handlers
                // (e.g. for Ctrl+C) still get a chance to run
                auto result = self.runResult();
                while (!result) {
                    {
                        py::gil_scoped_release release;
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    if (PyErr_CheckSignals() != 0)
                        throw py::error_already_set();
                    result = self.runResult();
                }

                raiseOnError(*result);
            },
            "Wait for image acquisition to end (raises RuntimeError if acquisition failed)")
        .def(
            "start_recording",
            [](Miniscope::Miniscope &self, const std::string &fname) {
                raiseOnError(self.startRecording(fname));
            },
            py::arg("fname") = "",
            py::call_guard<py::gil_scoped_release>(),
            "Start recording a video file (raises RuntimeError on failure)")
        .def(
            "stop_recording",
            &Miniscope::Miniscope::stopRecording,
            py::call_guard<py::gil_scoped_release>(),
            "Finish the current recording")

        .def_property_readonly("controls", &Miniscope::Miniscope::controls, "Get available controls for this device")
        .def(
            "control_value",
            &Miniscope::Miniscope::controlValue,
            py::call_guard<py::gil_scoped_release>(),
            "Retrieve current control value for the given control ID (None if the control does not exist)")
        .def(
            "set_control_value",
            [](Miniscope::Miniscope &self, const std::string &id, double value) {
                raiseOnError(self.setControlValue(id, value));
            },
            py::call_guard<py::gil_scoped_release>(),
            "Set new value for control with the given ID (raises RuntimeError on failure)")

        .def(
            "set_on_status_message",
            [](Miniscope::Miniscope &self, const py::object &callback) {
                Miniscope::StatusMessageCallback fn;
                if (auto cb = makePyCallback(callback, "miniscope status message callback")) {
                    fn = [cb](const std::string &message) {
                        (*cb)(message);
                    };
                }
                if (!self.setOnStatusMessage(std::move(fn)))
                    throw std::runtime_error("Callbacks can not be changed while acquisition is running.");
            },
            py::arg("callback"),
            "Set a callable receiving status messages as (message). It may be invoked from library threads.\n"
            "Pass None to remove it. Can not be changed while acquisition is running.")
        .def(
            "set_on_control_value_change",
            [](Miniscope::Miniscope &self, const py::object &callback) {
                Miniscope::ControlChangeCallback fn;
                if (auto cb = makePyCallback(callback, "miniscope control value change callback")) {
                    fn = [cb](const std::string &id, double displayValue, double deviceValue) {
                        (*cb)(id, displayValue, deviceValue);
                    };
                }
                if (!self.setOnControlValueChange(std::move(fn)))
                    throw std::runtime_error("Callbacks can not be changed while acquisition is running.");
            },
            py::arg("callback"),
            "Set a callable receiving control changes as (id, display_value, device_value). It may be invoked\n"
            "from library threads. Pass None to remove it. Can not be changed while acquisition is running.")

        .def_property_readonly(
            "is_connected", &Miniscope::Miniscope::isConnected, "Is True if a Miniscope is connected")
        .def_property_readonly(
            "is_running", &Miniscope::Miniscope::isRunning, "Is True if we are acquiring images from the Miniscope")
        .def_property_readonly("is_recording", &Miniscope::Miniscope::isRecording, "Is True if we are recording data")

        .def(
            "fetch_display_frame",
            &Miniscope::Miniscope::fetchDisplayFrame,
            "Take the next frame intended for display (None if there is no new frame). May not be the recorded frame.")
        .def_property_readonly("current_fps", &Miniscope::Miniscope::currentFps)
        .def_property_readonly("dropped_frame_count", &Miniscope::Miniscope::droppedFrameCount)
        .def_property_readonly("last_recorded_frame_time", &Miniscope::Miniscope::lastRecordedFrameTime)
        .def(
            "fetch_last_raw_frame",
            &Miniscope::Miniscope::fetchLastRawFrame,
            "Get the raw frame that was acquired last (None if there is no new frame since the last call).\n"
            "Not suitable to get every frame, as frames may be missed.")
        .def_property_readonly(
            "acquired_frame_count",
            &Miniscope::Miniscope::acquiredFrameCount,
            "The number of valid frames acquired since acquisition was started")
        .def(
            "wait_for_acquired_frame_count",
            [](Miniscope::Miniscope &self, unsigned int count) {
                raiseOnError(self.waitForAcquiredFrameCount(count));
            },
            py::arg("count"),
            py::call_guard<py::gil_scoped_release>(),
            "Wait for the given number of frames to be acquired (raises RuntimeError if acquisition stops)")
        .def_property_readonly(
            "fps", &Miniscope::Miniscope::fps, "The framerate the device is configured for (see current_fps)")
        .def_property(
            "use_unix_timestamps",
            &Miniscope::Miniscope::useUnixTimestamps,
            &Miniscope::Miniscope::setUseUnixTimestamps,
            "Whether frame timestamps are UNIX timestamps, instead of starting at zero")
        .def_property_readonly(
            "unix_capture_start_time",
            &Miniscope::Miniscope::unixCaptureStartTime,
            "The UNIX time frame timestamps are based on (zero if UNIX timestamps are not used)")

        .def_property(
            "video_filename",
            &Miniscope::Miniscope::videoFilename,
            &Miniscope::Miniscope::setVideoFilename,
            "The name of the saved video")
        .def_property(
            "video_codec",
            &Miniscope::Miniscope::videoCodec,
            &Miniscope::Miniscope::setVideoCodec,
            "The video codec to use")
        .def_property(
            "video_container",
            &Miniscope::Miniscope::videoContainer,
            &Miniscope::Miniscope::setVideoContainer,
            "The video container to use")
        .def_property(
            "record_lossless",
            &Miniscope::Miniscope::recordLossless,
            &Miniscope::Miniscope::setRecordLossless,
            "Toggle lossless recording, if the codec supports it")

        .def_property(
            "min_fluor_display",
            &Miniscope::Miniscope::minFluorDisplay,
            &Miniscope::Miniscope::setMinFluorDisplay,
            "Minimum fluorescence to display")
        .def_property(
            "max_fluor_display",
            &Miniscope::Miniscope::maxFluorDisplay,
            &Miniscope::Miniscope::setMaxFluorDisplay,
            "Maximum fluorescence to display")
        .def_property_readonly(
            "min_fluor", &Miniscope::Miniscope::minFluor, "Minimum fluorescence (pixel value) in the current image")
        .def_property_readonly(
            "max_fluor", &Miniscope::Miniscope::maxFluor, "Maximum fluorescence (pixel value) in the current image")

        .def_property(
            "display_mode",
            &Miniscope::Miniscope::displayMode,
            &Miniscope::Miniscope::setDisplayMode,
            "Set styling mode for the displayed images")
        .def_property(
            "bg_accumulate_alpha",
            &Miniscope::Miniscope::bgAccumulateAlpha,
            &Miniscope::Miniscope::setBgAccumulateAlpha)

        .def_property(
            "recording_slice_interval",
            &Miniscope::Miniscope::recordingSliceInterval,
            &Miniscope::Miniscope::setRecordingSliceInterval,
            "The interval at which new video files should be started when recording, in minutes")

        .def_property_readonly(
            "has_orientation_support",
            &Miniscope::Miniscope::hasHeadOrientationSupport,
            "Check whether head orientation support from a BNO sensor is available")
        .def_property(
            "bno_indicator_visible",
            &Miniscope::Miniscope::isBNOIndicatorVisible,
            &Miniscope::Miniscope::setBNOIndicatorVisible,
            "Whether an indicator for the BNO orientation should be rendered")
        .def_property(
            "save_orientation_data",
            &Miniscope::Miniscope::saveOrientationData,
            &Miniscope::Miniscope::setSaveOrientationData,
            "Whether orientation data from the BNO should be saved as CSV file");
}
