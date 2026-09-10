Miniscope Python module
=======================

This package provides Python bindings for `libminiscope`, the Miniscope data
acquisition library of [PoMiDAQ](https://github.com/bothlab/pomidaq).
It allows connecting to UCLA Miniscope V3/V4 and Siminiscope devices via their
DAQ box, controlling excitation, gain, focus (EWL) and other settings, receiving
frames as NumPy arrays and recording lossless video together with timestamps and
BNO orientation data.

The module has so far only been tested on Linux.

Dependencies
------------

The bindings are compiled from C++ and link against system libraries, which
must be present (including development headers) when building:

 * A C++23 capable compiler (GCC >= 14, Clang >= 17)
 * CMake (>= 3.18) and Ninja
 * OpenCV (>= 4.8)
 * FFmpeg (>= 6.1, `libavcodec`, `libavformat`, `libavutil`, `libswscale`)
 * [nlohmann-json](https://github.com/nlohmann/json) (>= 3.10)

On Debian-based systems:
```bash
sudo apt install build-essential cmake ninja-build libopencv-dev \
                 libavcodec-dev libavformat-dev libswscale-dev nlohmann-json3-dev \
                 python3-dev python3-pip
```
pybind11 and NumPy are fetched automatically as build requirements.

Installation
------------

Install directly from a source checkout:
```bash
pip install .
```

Or build a wheel and a source distribution:
```bash
python3 -m build
```
The resulting files are placed in the `dist/` directory. The wheel links
`libminiscope` statically, but still depends on the OpenCV and FFmpeg shared
libraries of the system it was built on. To distribute it to other machines,
run it through [auditwheel](https://github.com/pypa/auditwheel) (Linux) to bundle
these libraries.

The module can also be built and installed as part of a regular CMake build of
PoMiDAQ by enabling the `PYTHON` option.

Usage
-----

```python
from miniscope import Miniscope

mscope = Miniscope()
mscope.load_device_config('Miniscope_V4')
mscope.set_cam_id(0)

mscope.connect()
mscope.run()
mscope.set_control_value('led0', 20)

mscope.start_recording('/tmp/miniscope-test.mkv')
while mscope.is_running:
    frame = mscope.current_disp_frame  # NumPy array or None
    ...
mscope.stop()
mscope.disconnect()
```

Operations that can fail (`load_device_config`, `connect`, `run`,
`start_recording`, `hard_reset`) raise a `RuntimeError` with a descriptive
message. Errors that stop a running acquisition are available via the
`last_error` property.

A complete example can be found in
[`python/example.py`](https://github.com/bothlab/pomidaq/blob/master/python/example.py)
in the PoMiDAQ repository.

License
-------

LGPL-3.0-or-later
