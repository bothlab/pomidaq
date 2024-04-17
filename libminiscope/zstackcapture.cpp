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

#include <QtConcurrent>
#include <opencv2/imgcodecs.hpp>

#include "zstackcapture.h"

namespace MScope
{

static void emitProgress(TaskProgressEmitter *progress, int value)
{
    if (progress != nullptr)
        emit progress->progress(value);
}

class ZStackException : public QException
{
public:
    explicit ZStackException(const QString &message)
        : msg(message)
    {
    }
    explicit ZStackException(const char *message)
        : msg(QString::fromUtf8(message))
    {
    }
    explicit ZStackException(const std::exception &error)
        : msg(QString::fromUtf8(error.what()))
    {
    }

    void raise() const override
    {
        throw *this;
    }

    QException *clone() const override
    {
        return new ZStackException(*this);
    }

    const char *what() const noexcept override
    {
        return qPrintable(msg);
    }

private:
    QString msg;
};

static std::vector<cv::Mat> acquire3DData(
    Miniscope *mscope,
    const ControlDefinition &ewlControl,
    int fromEWL,
    int toEWL,
    uint step,
    uint averageCount,
    TaskProgressEmitter *progress,
    uint adjFrameWaitTime = 2)
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

        for (uint i = 0; i < averageCount; i++) {
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

        emitProgress(progress, (100.0 / maxProgress) * stack.size());
    }

    return stack;
}

static void captureZStack(
    Miniscope *mscope,
    int fromEWL,
    int toEWL,
    uint step,
    uint averageCount,
    const QString &outFilename,
    TaskProgressEmitter *progress)
{
    ControlDefinition ewlControl;
    const auto controls = mscope->controls();
    for (const auto &ctl : controls) {
        if (ctl.name.toLower().contains("ewl")) {
            ewlControl = ctl;
            break;
        }
    }

    QString outFilenameReal = outFilename;
    if (!outFilename.endsWith(".tiff") && !outFilename.endsWith(".tif"))
        outFilenameReal = QStringLiteral("%1.tiff").arg(outFilename);

    if (ewlControl.name.isEmpty())
        throw ZStackException("Could not find EWL controller to acquire Z-Stack!");
    if (!mscope->isRunning())
        throw ZStackException("Can not acquire Z-Stack while Miniscope is not running.");

    // move in range already, in case we have a big jump from the current EWL setting
    mscope->setControlValue(ewlControl.id, fromEWL);
    mscope->waitForAcquiredFrameCount(5);

    auto stack = acquire3DData(mscope, ewlControl, fromEWL, toEWL, step, averageCount, progress);

    std::vector<int> tiffParams;
    tiffParams.push_back(cv::IMWRITE_TIFF_COMPRESSION);
    tiffParams.push_back(5 /* zlib compression */);

    try {
        cv::imwrite(outFilenameReal.toStdString(), stack, tiffParams);
    } catch (const std::exception &e) {
        throw ZStackException(e);
    }

    emitProgress(progress, 100);
}

QFuture<void> launchZStackCapture(
    Miniscope *mscope,
    int fromEWL,
    int toEWL,
    uint step,
    uint averageCount,
    const QString &outFilename,
    TaskProgressEmitter *progress)
{
    // TODO: Make use of QPromise when we can switch to Qt6, obsolete ProgressEmitter
    emitProgress(progress, 0);
    return QtConcurrent::run([=]() {
        captureZStack(mscope, fromEWL, toEWL, step, averageCount, outFilename, progress);
    });
}

struct Accu3DProgress {
    int maxSteps;
    int currentStep{0};
    TaskProgressEmitter *emitter;

    void progressStep(uint stride = 1)
    {
        for (uint i = 0; i < stride; ++i)
            emitProgress(emitter, (100.0 / maxSteps) * currentStep++);
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
    const QStringList &rawImageFiles,
    std::vector<std::vector<double>> &fileStackMedians)
{
    std::vector<double> result;

    for (const auto &path : rawImageFiles) {
        std::vector<cv::Mat> stack;
        std::vector<double> stackMedians;
        cv::imreadmulti(path.toStdString(), stack, cv::IMREAD_GRAYSCALE);
        if (stack.empty() || stack[0].empty())
            continue;

        for (uint i = 0; i < stack.size(); ++i)
            stackMedians.push_back(medianBrightness(stack[i]));

        fileStackMedians.push_back(stackMedians);
    }

    // compute the median of medians for each slice
    for (uint i = 0; i < fileStackMedians[0].size(); ++i) {
        std::vector<double> sliceMedians;
        for (uint k = 0; k < fileStackMedians.size(); ++k)
            sliceMedians.push_back(fileStackMedians[k][i]);

        std::sort(sliceMedians.begin(), sliceMedians.end());
        size_t n = sliceMedians.size();
        result.push_back(n % 2 == 0 ? ((sliceMedians[n / 2 - 1] + sliceMedians[n / 2]) / 2.0) : sliceMedians[n / 2]);
    }

    return result;
}

static std::vector<cv::Mat> computeBalanced3DMIP(const QStringList &rawImageFiles, Accu3DProgress &progress)
{
    // we load the data from disk twice to (cheaply) save on used memory
    std::vector<std::vector<double>> fileStackMedians;
    progress.progressStep();
    auto sliceMedMed = globalSliceBrightnessMedianForFiles(rawImageFiles, fileStackMedians);
    progress.progressStep();

    std::vector<cv::Mat> mipStack;
    for (int i = 0; i < rawImageFiles.length(); ++i) {
        std::vector<cv::Mat> stack;
        cv::imreadmulti(rawImageFiles[i].toStdString(), stack, cv::IMREAD_GRAYSCALE);
        if (stack.empty()) {
            qWarning().noquote() << "Read empty stack from" << rawImageFiles[i] << "This may be a bug!";
            continue;
        }
        if (i == 0) {
            // initialize the MIP stack
            for (uint s = 0; s < sliceMedMed.size(); ++s)
                mipStack.push_back(cv::Mat());
        }

        for (uint s = 0; s < sliceMedMed.size(); ++s) {
            // check if within 20% of the global median
            if (fileStackMedians[i][s] <= sliceMedMed[s] * 1.2) {
                if (mipStack[s].empty())
                    mipStack[s] = stack[s];
                else
                    cv::max(mipStack[s], stack[s], mipStack[s]);
            } else {
                qDebug().noquote() << "Filtered out slice" << s << "with suspicious brightness from"
                                   << rawImageFiles[i];
            }
        }
        progress.progressStep();
    }

    return mipStack;
}

static void acquire3DAccumulation(
    Miniscope *mscope,
    int fromEWL,
    int toEWL,
    uint step,
    uint count,
    bool saveRaw,
    const QString &outDir,
    const QString &outName,
    TaskProgressEmitter *progress)
{
    ControlDefinition ewlControl;
    const auto controls = mscope->controls();
    for (const auto &ctl : controls) {
        if (ctl.name.toLower().contains("ewl")) {
            ewlControl = ctl;
            break;
        }
    }

    if (ewlControl.name.isEmpty())
        throw ZStackException("Could not find EWL controller to acquire Z-Stack!");
    if (!mscope->isRunning())
        throw ZStackException("Can not acquire Z-Stack while Miniscope is not running.");

    QDir outDirNamed(QStringLiteral("%1/%2/").arg(outDir, outName));
    QDir outDirRaw(QStringLiteral("%1/raw").arg(outDirNamed.absolutePath()));
    if (!outDirRaw.mkpath(outDirRaw.absolutePath()))
        throw ZStackException(QStringLiteral("Unable to create directory '%1'.").arg(outDirRaw.absolutePath()));

    // TIFF storage settings
    std::vector<int> tiffSaveParams;
    tiffSaveParams.push_back(cv::IMWRITE_TIFF_COMPRESSION);
    tiffSaveParams.push_back(5 /* LZW compression */);
    tiffSaveParams.push_back(317 /* TIFFTAG_PREDICTOR */);
    tiffSaveParams.push_back(2 /* PREDICTOR_HORIZONTAL */);

    Accu3DProgress aprog;
    aprog.maxSteps = (count * 4) + 2 + 2 + 1;
    aprog.emitter = progress;

    // move in range already, in case we have a big jump from the current EWL setting
    aprog.progressStep();
    mscope->setControlValue(ewlControl.id, fromEWL);
    mscope->waitForAcquiredFrameCount(5);

    QStringList rawFileList;
    for (uint i = 0; i < count; ++i) {
        auto fnameRaw = QStringLiteral("%1/%2_zstack_%3.tiff").arg(outDirRaw.absolutePath(), outName).arg(i);

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
            nullptr,
            1 /* frame acq wait time */);
        aprog.progressStep(2);

        // reverse stack if we recorded backwards
        if (!recForward)
            std::reverse(stack.begin(), stack.end());

        // store raw data for future use
        try {
            cv::imwrite(fnameRaw.toStdString(), stack, tiffSaveParams);
        } catch (const std::exception &e) {
            throw ZStackException(e);
        }

        rawFileList.append(fnameRaw);
        aprog.progressStep();
    }

    auto balancedMipStack = computeBalanced3DMIP(rawFileList, aprog);
    const auto mipStackFname = QStringLiteral("%1/%2_mip3D.tiff").arg(outDirNamed.absolutePath(), outName);
    try {
        cv::imwrite(mipStackFname.toStdString(), balancedMipStack, tiffSaveParams);
    } catch (const std::exception &e) {
        throw ZStackException(e);
    }
    aprog.progressStep();

    // cleanup temporary raw data, if requested
    if (!saveRaw) {
        for (const auto &fname : rawFileList) {
            QFile f(fname);
            f.remove();
        }
        outDirRaw.rmpath(outDirRaw.absolutePath());
    }

    emitProgress(progress, 100);
}

QFuture<void> launch3DAccumulation(
    Miniscope *mscope,
    int fromEWL,
    int toEWL,
    uint step,
    uint count,
    bool saveRaw,
    const QString &outDir,
    const QString &outName,
    TaskProgressEmitter *progress)
{
    // TODO: Make use of QPromise when we can switch to Qt6, obsolete ProgressEmitter
    emitProgress(progress, 0);
    return QtConcurrent::run([=]() {
        acquire3DAccumulation(mscope, fromEWL, toEWL, step, count, saveRaw, outDir, outName, progress);
    });
}

} // namespace MScope
