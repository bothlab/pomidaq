PoMiDAQ
=========

PoMiDAQ is a recording software for [UCLA Miniscopes](http://miniscope.org/index.php/Main_Page).
Unlike the [preexisting](https://github.com/daharoni/Miniscope_DAQ_Software) Miniscope software, this program
is written in a portable way, so it will run on Windows, Linux and macOS. It will also run with the latest
OpenCV libraries and provides a shared library, so Miniscope recording can be embedded into other programs
and pipelines easily (writing Python bindings for this should be trivial).
Recorded data is encoded with the [FFV1](https://en.wikipedia.org/wiki/FFV1) codec by default, to allow for
smaller video files at the same quality.
Some features the original Miniscope DAQ software has, like the behavior recording, are intentionally left out.
The software is currently developed and tested only on Linux.

This code is a work-in-progress, expect bugs, unfinished features and unpredictable behavior!
