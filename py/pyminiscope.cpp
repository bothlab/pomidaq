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
using namespace boost::python;

BOOST_PYTHON_MODULE(miniscope)
{
    class_<MiniScope>("MiniScope")
        .add_property("exposure", &MiniScope::exposure, &MiniScope::setExposure, "Miscroscope image exposure setting")
        .add_property("use_color", &MiniScope::useColor, &MiniScope::setUseColor)
        .add_property("current_frame", &MiniScope::currentFrame)
    ;

}

