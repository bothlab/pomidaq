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

#include "mediatypes.h"
#include "logging.h"
#include "asynctask.h"

#ifdef _WIN32
#define MS_LIB_EXPORT __declspec(dllexport)
#else
#define MS_LIB_EXPORT __attribute__((visibility("default")))
#endif

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

using StatusMessageCallback = std::function<void(const std::string &, void *)>;
using ControlChangeCallback = std::function<void(const std::string &, double, double, void *)>;
using RawDataCallback = std::function<void(
    const cv::Mat &,
    milliseconds_t &,
    const milliseconds_t &,
    const milliseconds_t &,
    const std::vector<float> &orientation,
    void *)>;
using DisplayFrameCallback = std::function<void(const cv::Mat &, const milliseconds_t &, void *)>;

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
class ControlDefinition
{
public:
    explicit ControlDefinition()
        : valueMin(-1),
          valueMax(-1)
    {
    }

    ControlKind kind;
    std::string id;
    std::string name;

    int valueMin;
    int valueMax;
    double valueStart;
    int stepSize;

    std::vector<std::string> labels;
    std::vector<double> values;
};

/**
 * @brief Controller for the UCLA Miniscope family
 */
class MS_LIB_EXPORT Miniscope
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
    double controlValue(const std::string &id);
    void setControlValue(const std::string &id, double value);

    /**
     * @brief Start frame acquisition in a background thread.
     *
     * Fails if no device is connected or a reconnect after a previous failure
     * does not succeed. Errors that happen later during acquisition are not
     * reported here, but via lastError().
     */
    Result run();
    void stop();

    /**
     * @brief Start recording acquired frames to disk.
     *
     * Starts the acquisition first if it is not running yet.
     * Fails if no device is connected, or if acquisition can not be started.
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
    void setOnStatusMessage(StatusMessageCallback callback, void *udata = nullptr);

    /**
     * @brief Called when a Miniscope setting is changes.
     */
    void setOnControlValueChange(ControlChangeCallback callback, void *udata = nullptr);

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
    void setOnFrame(RawDataCallback callback, void *udata = nullptr);

    /**
     * @brief Called *in the DAQ thread* when a frame was acquired on the edited frame.
     *
     * This callback is executed for each (possibly modified) "display frame" that an application like
     * PoMiDAQ would show to the user.
     */
    void setOnDisplayFrame(DisplayFrameCallback callback, void *udata = nullptr);

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

    unsigned int currentFps() const;
    size_t droppedFramesCount() const;

    double fps() const;

    void setCaptureStartTime(const std::chrono::time_point<std::chrono::steady_clock> &startTime);
    bool useUnixTimestamps() const;
    void setUseUnixTimestamps(bool useUnixTime);
    milliseconds_t unixCaptureStartTime() const;

    bool externalRecordTrigger() const;
    void setExternalRecordTrigger(bool enabled);

    std::string videoFilename() const;
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

    /**
     * @brief The error that stopped frame acquisition, if it failed.
     *
     * The acquisition thread reports errors (dropped frames, encoder problems, lost
     * connection, ...) asynchronously: it stops and puts the device into a failed state.
     * Poll this after noticing that isRunning() turned false. Errors of synchronous
     * calls such as connect() are also retained here until the next successful connect().
     */
    std::optional<std::string> lastError() const;

    milliseconds_t lastRecordedFrameTime() const;

    long acquiredFrameCount() const;

    /**
     * @brief Wait for an amount of frames to be acquired.
     * @param The amount of frames to wait for.
     * @return True if we waited for the appropriate amount of frames,
     *         False in case of an error.
     */
    bool waitForAcquiredFrameCount(unsigned int count);

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
    milliseconds_t getCurrentFrameTimestamp();
    void statusMessage(const std::string &msg);
    void fail(const std::string &msg);
};

/**
 * @brief Return a device name for an integer ID
 * This function is only supported on some operating systems, currently only on Linux.
 */
MS_LIB_EXPORT
std::string videoDeviceNameFromId(int id);

} // namespace Miniscope

#endif // MINISCOPE_H
