/*
 * Copyright (C) 2020-2024 Matthias Klumpp <matthias@tenstral.net>
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

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>

#include "zstackcapture.h"
#include "asynctask-private.h"
#include "loginternal.h"
#include "utils.h"

namespace fs = std::filesystem;

namespace Miniscope
{

MS_DEFINE_LOG_CATEGORY(logMScope, "miniscope");

class ZStackException : public std::runtime_error
{
public:
    explicit ZStackException(const std::string &message)
        : std::runtime_error(message)
    {
    }
    explicit ZStackException(const char *message)
        : std::runtime_error(message)
    {
    }
    explicit ZStackException(const std::exception &error)
        : std::runtime_error(error.what())
    {
    }
};

static bool nameContainsEWL(const std::string &name)
{
    return Utils::asciiToLower(name).find("ewl") != std::string::npos;
}

static std::vector<cv::Mat> acquire3DData(
    Miniscope *mscope,
    const ControlDefinition &ewlControl,
    int fromEWL,
    int toEWL,
    unsigned int step,
    unsigned int averageCount,
    unsigned int adjFrameWaitTime = 2,
    TaskProgress *progress = nullptr)
{
    if (fromEWL - toEWL == 0)
        throw ZStackException("EWL start and end positions must be different.");
    if (step == 0)
        throw ZStackException("Step size can not be zero.");

    if (fromEWL < ewlControl.valueMin || fromEWL > ewlControl.valueMax)
        throw ZStackException("First EWL position value is out of range.");
    if (toEWL < ewlControl.valueMin || toEWL > ewlControl.valueMax)
        throw ZStackException("Second EWL position value is out of range.");

    if (averageCount == 0)
        throw ZStackException("Image average count must not be zero.");
    if (averageCount > 36000)
        throw ZStackException("Image average count is too large.");

    int stepSigned;
    if (fromEWL - toEWL < 0)
        stepSigned = step;
    else
        stepSigned = step * -1;

    int maxProgress = abs(fromEWL - toEWL) / abs(stepSigned);
    std::vector<cv::Mat> stack;
    for (int currentPos = fromEWL; currentPos != toEWL; currentPos += stepSigned) {
        std::vector<cv::Mat> currentMats;

        // sanity check
        if (currentPos < ewlControl.valueMin || currentPos > ewlControl.valueMax)
            break;

        // adjust
        mscope->setControlValue(ewlControl.id, currentPos);
        // wait for some frames to give the EWL time to adjust
        mscope->waitForAcquiredFrameCount(adjFrameWaitTime);

        // FIXME: Ideally we should verify that the device has actually adjusted the EWL,
        // but this feature is not yet iplemented in the library (we currently always
        // return the initial value)

        for (unsigned int i = 0; i < averageCount; i++) {
            cv::Mat raw;
            while (true) {
                if (mscope->fetchLastRawFrame(raw))
                    break;
                // wait a bit of time (~1 frame)
                mscope->waitForAcquiredFrameCount(1);
            }
            currentMats.push_back(raw);
        }

        // calculate image average
        cv::Mat accMat(currentMats[0].rows, currentMats[0].cols, CV_32F);
        accMat.setTo(cv::Scalar(0, 0, 0));

        for (const cv::Mat &raw : currentMats) {
            cv::Mat mat32;
            raw.convertTo(mat32, CV_32F);
            accMat += mat32;
        }

        accMat.convertTo(accMat, CV_8U, 1. / currentMats.size());
        stack.push_back(accMat);

        if (progress != nullptr)
            progress->setValue((100.0 / maxProgress) * stack.size());
    }

    return stack;
}

static bool captureZStack(
    TaskProgress &progress,
    Miniscope *mscope,
    int fromEWL,
    int toEWL,
    unsigned int step,
    unsigned int averageCount,
    const std::string &outFilename)
{
    progress.setValue(0);

    ControlDefinition ewlControl;
    const auto controls = mscope->controls();
    for (const auto &ctl : controls) {
        if (nameContainsEWL(ctl.name)) {
            ewlControl = ctl;
            break;
        }
    }

    std::string outFilenameReal = outFilename;
    if (!outFilename.ends_with(".tiff") && !outFilename.ends_with(".tif"))
        outFilenameReal = std::format("{}.tiff", outFilename);

    if (ewlControl.name.empty())
        throw ZStackException("Could not find EWL controller to acquire Z-Stack!");
    if (!mscope->isRunning())
        throw ZStackException("Can not acquire Z-Stack while Miniscope is not running.");

    // move in range already, in case we have a big jump from the current EWL setting
    mscope->setControlValue(ewlControl.id, fromEWL);
    mscope->waitForAcquiredFrameCount(5);

    auto stack = acquire3DData(mscope, ewlControl, fromEWL, toEWL, step, averageCount, 2, &progress);

    std::vector<int> tiffParams;
    tiffParams.push_back(cv::IMWRITE_TIFF_COMPRESSION);
    tiffParams.push_back(5 /* zlib compression */);

    try {
        cv::imwrite(outFilenameReal, stack, tiffParams);
    } catch (const std::exception &e) {
        throw ZStackException(e);
    }

    progress.setValue(100);
    return true;
}

AsyncTask launchZStackCapture(
    Miniscope *mscope,
    int fromEWL,
    int toEWL,
    unsigned int step,
    unsigned int averageCount,
    const std::string &outFilename)
{
    return launchAsyncTask([=](TaskProgress &progress) {
        return captureZStack(progress, mscope, fromEWL, toEWL, step, averageCount, outFilename);
    });
}

struct Accu3DProgress {
    int maxSteps{1};
    int currentStep{0};
    TaskProgress *progress;

    explicit Accu3DProgress(TaskProgress *p)
        : progress(p)
    {
    }

    void progressStep(unsigned int stride = 1)
    {
        for (unsigned int i = 0; i < stride; ++i)
            progress->setValue((100.0 / maxSteps) * currentStep++);
    }
};

static double medianBrightness(const cv::Mat &img)
{
    std::vector<uchar> pixels;
    img.reshape(1, img.total()).copyTo(pixels);
    std::nth_element(pixels.begin(), pixels.begin() + pixels.size() / 2, pixels.end());
    double median = pixels[pixels.size() / 2];
    if (pixels.size() % 2 == 0) {
        std::nth_element(pixels.begin(), pixels.begin() + pixels.size() / 2 - 1, pixels.end());
        median = (median + pixels[pixels.size() / 2 - 1]) / 2.0;
    }
    return median;
}

static std::vector<double> globalSliceBrightnessMedianForFiles(
    const std::vector<std::string> &rawImageFiles,
    std::vector<std::vector<double>> &fileStackMedians)
{
    std::vector<double> result;

    for (const auto &path : rawImageFiles) {
        std::vector<cv::Mat> stack;
        std::vector<double> stackMedians;
        cv::imreadmulti(path, stack, cv::IMREAD_GRAYSCALE);
        if (stack.empty() || stack[0].empty())
            continue;

        for (unsigned int i = 0; i < stack.size(); ++i)
            stackMedians.push_back(medianBrightness(stack[i]));

        fileStackMedians.push_back(stackMedians);
    }

    // compute the median of medians for each slice
    for (unsigned int i = 0; i < fileStackMedians[0].size(); ++i) {
        std::vector<double> sliceMedians;
        for (unsigned int k = 0; k < fileStackMedians.size(); ++k)
            sliceMedians.push_back(fileStackMedians[k][i]);

        std::sort(sliceMedians.begin(), sliceMedians.end());
        size_t n = sliceMedians.size();
        result.push_back(n % 2 == 0 ? ((sliceMedians[n / 2 - 1] + sliceMedians[n / 2]) / 2.0) : sliceMedians[n / 2]);
    }

    return result;
}

static std::vector<cv::Mat> computeBalanced3DMIP(
    const std::vector<std::string> &rawImageFiles,
    Accu3DProgress &progress)
{
    // we load the data from disk twice to (cheaply) save on used memory
    std::vector<std::vector<double>> fileStackMedians;
    progress.progressStep();
    auto sliceMedMed = globalSliceBrightnessMedianForFiles(rawImageFiles, fileStackMedians);
    progress.progressStep();

    std::vector<cv::Mat> mipStack;
    for (size_t i = 0; i < rawImageFiles.size(); ++i) {
        std::vector<cv::Mat> stack;
        cv::imreadmulti(rawImageFiles[i], stack, cv::IMREAD_GRAYSCALE);
        if (stack.empty()) {
            MS_LOG_WARNING(logMScope, "Read empty stack from {} This may be a bug!", rawImageFiles[i]);
            continue;
        }
        if (i == 0) {
            // initialize the MIP stack
            for (unsigned int s = 0; s < sliceMedMed.size(); ++s)
                mipStack.push_back(cv::Mat());
        }

        for (unsigned int s = 0; s < sliceMedMed.size(); ++s) {
            // check if within 20% of the global median
            if (fileStackMedians[i][s] <= sliceMedMed[s] * 1.2) {
                if (mipStack[s].empty())
                    mipStack[s] = stack[s];
                else
                    cv::max(mipStack[s], stack[s], mipStack[s]);
            } else {
                MS_LOG_DEBUG(
                    logMScope, "Filtered out slice {} with suspicious brightness from {}", s, rawImageFiles[i]);
            }
        }
        progress.progressStep();
    }

    return mipStack;
}

static bool acquire3DAccumulation(
    TaskProgress &progress,
    Miniscope *mscope,
    int fromEWL,
    int toEWL,
    unsigned int step,
    unsigned int count,
    bool saveRaw,
    const std::string &outDir,
    const std::string &outName)
{
    progress.setValue(0);

    ControlDefinition ewlControl;
    const auto controls = mscope->controls();
    for (const auto &ctl : controls) {
        if (nameContainsEWL(ctl.name)) {
            ewlControl = ctl;
            break;
        }
    }

    if (ewlControl.name.empty())
        throw ZStackException("Could not find EWL controller to acquire Z-Stack!");
    if (!mscope->isRunning())
        throw ZStackException("Can not acquire Z-Stack while Miniscope is not running.");

    const fs::path outDirNamed = fs::absolute(fs::path(outDir) / outName);
    const fs::path outDirRaw = outDirNamed / "raw";
    std::error_code ec;
    fs::create_directories(outDirRaw, ec);
    if (ec || !fs::is_directory(outDirRaw))
        throw ZStackException(std::format("Unable to create directory '{}'.", outDirRaw.string()));

    // TIFF storage settings
    std::vector<int> tiffSaveParams;
    tiffSaveParams.push_back(cv::IMWRITE_TIFF_COMPRESSION);
    tiffSaveParams.push_back(5 /* LZW compression */);
    tiffSaveParams.push_back(317 /* TIFFTAG_PREDICTOR */);
    tiffSaveParams.push_back(2 /* PREDICTOR_HORIZONTAL */);

    Accu3DProgress aprog(&progress);
    aprog.maxSteps = (count * 3) + 2 + 1 + count + 2;

    // move in range already, in case we have a big jump from the current EWL setting
    aprog.progressStep();
    mscope->setControlValue(ewlControl.id, fromEWL);
    mscope->waitForAcquiredFrameCount(5);

    progress.setValueAndText(aprog.currentStep, "Acquiring data...");
    std::vector<std::string> rawFileList;
    for (unsigned int i = 0; i < count; ++i) {
        auto fnameRaw = (outDirRaw / std::format("{}_zstack_{}.tiff", outName, i)).string();

        int hwFromEWL;
        int hwToEWL;
        bool recForward = i % 2 == 0;

        if (recForward) {
            // forward
            hwFromEWL = fromEWL;
            hwToEWL = toEWL;
        } else {
            // reverse
            hwFromEWL = toEWL;
            hwToEWL = fromEWL;
        }

        auto stack = acquire3DData(
            mscope,
            ewlControl,
            hwFromEWL,
            hwToEWL,
            step,
            1, /* average count */
            1 /* frame acq wait time */);
        aprog.progressStep(2);

        // reverse stack if we recorded backwards
        if (!recForward)
            std::reverse(stack.begin(), stack.end());

        // store raw data for future use
        try {
            cv::imwrite(fnameRaw, stack, tiffSaveParams);
        } catch (const std::exception &e) {
            throw ZStackException(e);
        }

        rawFileList.push_back(fnameRaw);
        aprog.progressStep();
    }

    progress.setValueAndText(aprog.currentStep, "Computing MIPs...");
    auto balancedMipStack = computeBalanced3DMIP(rawFileList, aprog);
    const auto mipStackFname = (outDirNamed / std::format("{}_mip3D.tiff", outName)).string();
    try {
        cv::imwrite(mipStackFname, balancedMipStack, tiffSaveParams);
    } catch (const std::exception &e) {
        throw ZStackException(e);
    }
    aprog.progressStep();

    // cleanup temporary raw data, if requested
    if (!saveRaw) {
        for (const auto &fname : rawFileList)
            fs::remove(fname, ec);
        fs::remove(outDirRaw, ec);
    }

    progress.setValue(100);
    return true;
}

AsyncTask launch3DAccumulation(
    Miniscope *mscope,
    int fromEWL,
    int toEWL,
    unsigned int step,
    unsigned int count,
    bool saveRaw,
    const std::string &outDir,
    const std::string &outName)
{
    return launchAsyncTask([=](TaskProgress &progress) {
        return acquire3DAccumulation(progress, mscope, fromEWL, toEWL, step, count, saveRaw, outDir, outName);
    });
}

} // namespace Miniscope
