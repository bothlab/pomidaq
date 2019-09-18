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

#include "mediatypes.h"

using namespace MScope;

VideoCodec MScope::stringToVideoCodec(const std::string &str)
{
    if (str == "Raw")
        return VideoCodec::Raw;
    if (str == "None")
        return VideoCodec::Raw;
    if (str == "FFV1")
        return VideoCodec::FFV1;
    if (str == "AV1")
        return VideoCodec::AV1;
    if (str == "VP9")
        return VideoCodec::VP9;
    if (str == "H.265")
        return VideoCodec::H265;
    if (str == "MPEG-4")
        return VideoCodec::MPEG4;

    return VideoCodec::Unknown;
}

std::string videoCodecToString(VideoCodec codec)
{
    switch (codec) {
    case VideoCodec::Raw:
        return "None";
    case VideoCodec::FFV1:
        return "FFV1";
    case VideoCodec::AV1:
        return "AV1";
    case VideoCodec::VP9:
        return "VP9";
    case VideoCodec::H265:
        return "H.265";
    case VideoCodec::MPEG4:
        return "MPEG-4";
    default:
        return "Unknown";
    }
}

std::string videoContainerToString(VideoContainer container)
{
    switch (container) {
    case VideoContainer::Matroska:
        return "Matroska";
    case VideoContainer::AVI:
        return "AVI";
    default:
        return "Unknown";
    }
}

VideoContainer stringToVideoContainer(const std::string &str)
{
    if (str == "Matroska")
        return VideoContainer::Matroska;
    if (str == "AVI")
        return VideoContainer::AVI;

    return VideoContainer::Unknown;
}
