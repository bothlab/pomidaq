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
        // wait for ~4 frames to give the EWL time to adjust
        mscope->waitForAcquiredFrameCount(4);

        // FIXME: We should verify that the device has actually adjusted the EWL,
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

    try {
        cv::imwrite(outFilenameReal.toStdString(), stack);
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

} // namespace MScope
