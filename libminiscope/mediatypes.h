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

#ifndef MEDIATYPES_H
#define MEDIATYPES_H

#include <string>

namespace MScope
{

/**
 * @brief The VideoContainer enum
 *
 * Video container formats that we support in VideoWriter.
 * Each container must be compatible with every codec type
 * that we also support.
 */
enum class VideoContainer {
    Unknown,
    Matroska,
    AVI
};

std::string videoContainerToString(VideoContainer container);
VideoContainer stringToVideoContainer(const std::string& str);

/**
 * @brief The VideoCodec enum
 *
 * Video codecs that we support in VideoWriter.
 * Each codec must be compatible with every container type
 * that we also support, to avoid unnecessary user confusion and
 * API errors.
 * Currently, the only permanent exception to this rule is the "Raw" encoder,
 * which only supports the AVI container.
 */
enum class VideoCodec {
    Unknown,
    Raw,
    FFV1,
    AV1,
    VP9,
    H265,
    MPEG4
};

std::string videoCodecToString(VideoCodec codec);
VideoCodec stringToVideoCodec(const std::string& str);

} // end of MiniScope namespace

#endif // MEDIATYPES_H
