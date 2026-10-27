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

#ifndef MINISCOPE_H
#define MINISCOPE_H

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

#include "miniscope_export.h"
#include "mediatypes.h"
#include "logging.h"
#include "asynctask.h"

namespace Miniscope
{

using milliseconds_t = std::chrono::milliseconds;

/**
 * @brief Outcome of an operation that can fail.
 *
 * Holds nothing on success, or a human-readable error message on failure.
 * Use `if (auto r = mscope.connect(); !r) show(r.error());` at the call site.
 */
using Result = std::expected<void, std::string>;

using StatusMessageCallback = std::function<void(const std::string &)>;
using ControlChangeCallback = std::function<void(const std::string &, double, double)>;
using RawDataCallback = std::function<void(
    const cv::Mat &,
    milliseconds_t &,
    const milliseconds_t &,
    const milliseconds_t &,
    const std::vector<float> &orientation)>;
using DisplayFrameCallback = std::function<void(const cv::Mat &, const milliseconds_t &)>;

enum class DisplayMode {
    RawFrames,
    BackgroundDiff
};

/**
 * @brief Set which type of control is needed
 */
enum class ControlKind {
    Unknown,  /// Unknown scope control
    Selector, /// switch between a set of predefined values
    Slider    /// slide between an min and a max value
};

/**
 * @brief Miniscope control definition
 */
struct ControlDefinition {
    ControlKind kind = ControlKind::Unknown;
    std::string id;
    std::string name;

    int valueMin = -1;
    int valueMax = -1;
    double valueStart = 0;
    int stepSize = 1;

    std::vector<std::string> labels;
};

/**
 * @brief Controller for the UCLA Miniscope family
 */
class MINISCOPE_EXPORT Miniscope
{
public:
    explicit Miniscope();
    ~Miniscope();

    std::vector<std::string> availableDeviceTypes() const;

    /**
     * @brief Load the hardware definition for the given device type.
     *
     * Disconnects any currently connected device first.
     * Fails if no device with this name is known.
     */
    Result loadDeviceConfig(const std::string &deviceType);
    std::string deviceType() const;

    void setScopeCamId(int id);
    int scopeCamId() const;

    /**
     * @brief Open the camera connection to the selected device.
     *
     * Fails if no device type was loaded or the camera can not be opened.
     */
    Result connect();
    void disconnect();

    /**
     * @brief Request a hard reset of the DAQ board and reboot it.
     */
    Result hardReset();

    std::vector<ControlDefinition> controls() const;

    /**
     * @brief Get the current value of a Miniscope control.
     *
     * The device can not be queried for its control values, so this is the value
     * that was last set, or the control's start value if it was not set yet.
     * Returns nothing if no control with this ID exists.
     */
    std::optional<double> controlValue(const std::string &id) const;

    /**
     * @brief Set a control of the Miniscope to a new value.
     *
     * Fails if the control does not exist, or if the value is out of the control's
     * range (for selector controls, the value must be the index of one of its labels).
     * This function may be called from any thread, calls are serialized.
     */
    Result setControlValue(const std::string &id, double value);

    /**
     * @brief Start frame acquisition in a background thread.
     *
     * Does nothing if acquisition is already running.
     * Fails if no device is connected or a reconnect after a previous failure
     * does not succeed. Errors that happen later during acquisition are not
     * reported here, but by waitForResult().
     */
    Result run();

    /**
     * @brief Stop frame acquisition.
     *
     * Blocks until the acquisition thread has finished, so any ongoing recording is
     * finalized once this returns. Use waitForResult() to find out whether the
     * acquisition run ended because of an error.
     */
    void stop();

    /**
     * @brief Wait for the acquisition run to end, and return how it ended.
     *
     * The acquisition thread may stop on its own because of an error (dropped frames,
     * encoder problems, lost connection, ...), in which case isRunning() turns false.
     * This function blocks until the run has ended, either on its own or because stop()
     * was called (possibly from a different thread), and returns the error that ended it.
     * Any ongoing recording is finalized once this returns.
     * Returns immediately if acquisition is not running. Calling it again returns the same
     * result until the next call to run(). Returns success if acquisition was never started.
     */
    Result waitForResult();

    /**
     * @brief Start recording acquired frames to disk.
     *
     * Starts the acquisition first if it is not running yet.
     * Fails if no device is connected, if a recording is already running,
     * or if acquisition can not be started.
     */
    Result startRecording(const std::string &fname = "");
    void stopRecording();
    AsyncTask acquireZStack(
        int fromEWL,
        int toEWL,
        unsigned int step,
        unsigned int averageCount,
        const std::string &outFilename);
    AsyncTask accumulate3DView(
        int fromEWL,
        int toEWL,
        unsigned int step,
        unsigned int count,
        bool saveRaw,
        const std::string &outDir,
        const std::string &outName);

    bool isConnected() const;
    bool isRunning() const;
    bool isRecording() const;
    bool captureStartTimeInitialized() const;

    void setVisibleChannels(bool red, bool green, bool blue);
    bool showRedChannel() const;
    bool showGreenChannel() const;
    bool showBlueChannel() const;

    /**
     * @brief Called when a new status message is generated.
     *
     * Status messages reflect the general device state. They are not used for logging
     * or detailed event reporting.
     */
    void setOnStatusMessage(StatusMessageCallback callback);

    /**
     * @brief Called when a Miniscope setting is changes.
     *
     * The callback is run in the thread that set the value. That is the thread which called
     * setControlValue(), but it can also be the DAQ thread, which sets all controls again when it
     * had to reconnect to the device, or the thread of an async task like the z-stack capture.
     * The callback is never run by two threads at the same time, and it may call setControlValue()
     * itself.
     */
    void setOnControlValueChange(ControlChangeCallback callback);

    /**
     * @brief Called *in the DAQ thread* when a frame was acquired.
     *
     * This callback is executed for each raw frame acquired from the Miniscope, and is equivalent
     * to what would be recorded to a video file.
     * The first timestamp is the timestamp of the frame in milliseconds. The callee may modify it,
     * to change the timestamp of a frame while it is being processed.
     *
     * The second timestamp parameter is the timestamp of the computer's clock when the frame
     * was - highly likely - acquired ("master timestamp").
     * The third timestamp parameter is the adjusted timestamp generated by the driver or device.
     * Both timestamps may already have been preprocessed a bit (adjusted for the selected start-time
     * and timestamp type), so you may not get completely "raw" timestamps.
     *
     * Please note that this function will also be called in case we are dropping frames. In this case, the
     * frame data matrix will be empty.
     */
    void setOnFrame(RawDataCallback callback);

    /**
     * @brief Called *in the DAQ thread* when a frame was acquired on the edited frame.
     *
     * This callback is executed for each (possibly modified) "display frame" that an application like
     * PoMiDAQ would show to the user.
     */
    void setOnDisplayFrame(DisplayFrameCallback callback);

    /**
     * @brief Retrieve the current display frame from the display queue.
     */
    cv::Mat currentDisplayFrame();

    /**
     * @brief Retrieve the raw frame that was acquired last.
     *
     * This function is *not* suitable to store the acquired image to disk,
     * as it is not guaranteed that all frames will be obtained even if the
     * function is called frequently. Use the callback set via  setOnFrame()
     * to retrieve all raw frames as they are received.
     *
     * @return The frame if it was not retrieved before, or nothing if there is no new frame yet.
     */
    std::optional<cv::Mat> fetchLastRawFrame();

    /**
     * @brief The number of frames that were acquired in the last second.
     */
    unsigned int currentFps() const;
    size_t droppedFramesCount() const;

    double fps() const;

    void setCaptureStartTime(const std::chrono::time_point<std::chrono::steady_clock> &startTime);
    bool useUnixTimestamps() const;
    void setUseUnixTimestamps(bool useUnixTime);
    milliseconds_t unixCaptureStartTime() const;

    /**
     * @brief Start and stop recording via the DAQ board's external trigger input.
     *
     * NOTE: This is not implemented yet. The setting is stored, but currently has no effect.
     */
    bool externalRecordTrigger() const;
    void setExternalRecordTrigger(bool enabled);

    std::string videoFilename() const;

    /**
     * @brief Set the base filename for the next recording.
     *
     * The filename can not be changed while recording, calls are ignored in that case.
     */
    void setVideoFilename(const std::string &fname);

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

    DisplayMode displayMode() const;
    void setDisplayMode(DisplayMode mode);

    bool hasHeadOrientationSupport() const;
    bool isBNOIndicatorVisible() const;
    void setBNOIndicatorVisible(bool visible);

    bool saveOrientationData() const;
    void setSaveOrientationData(bool save);

    double bgAccumulateAlpha() const;
    void setBgAccumulateAlpha(double value);

    unsigned int recordingSliceInterval() const;
    void setRecordingSliceInterval(unsigned int minutes);

    void setPrintExtraDebug(bool enabled);

    milliseconds_t lastRecordedFrameTime() const;

    /**
     * @brief The number of valid frames acquired since acquisition was started.
     *
     * Dropped frames are not counted.
     */
    long acquiredFrameCount() const;

    /**
     * @brief Wait for an amount of frames to be acquired.
     * @param The amount of frames to wait for.
     *
     * Fails if acquisition is not running, or if it stops while we are waiting.
     */
    Result waitForAcquiredFrameCount(unsigned int count);

private:
    class Private;
    Miniscope(const Miniscope &) = delete;
    Miniscope &operator=(const Miniscope &) = delete;
    std::unique_ptr<Private> d;

    bool openCamera();
    void enqueueI2CCommand(long preambleKey, std::vector<uint8_t> packet);
    void sendCommandsToDevice();
    void addDisplayFrameToBuffer(const cv::Mat &frame, const milliseconds_t &timestamp);
    void setLastRawFrame(const cv::Mat &frame);
    static void captureThread(void *msPtr);
    void startCaptureThread();
    void finishCaptureThread();
    void joinCaptureThread();
    milliseconds_t getCurrentFrameTimestamp();
    void statusMessage(const std::string &msg);
    void fail(const std::string &msg);
};

/**
 * @brief Return a device name for an integer ID
 * This function is only supported on some operating systems, currently only on Linux.
 */
MINISCOPE_EXPORT
std::string videoDeviceNameFromId(int id);

} // namespace Miniscope

#endif // MINISCOPE_H
