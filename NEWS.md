Version 1.0.0
-------------
Released: 2026-10-28

### Features:
 * Remove Qt dependency from libminiscope
 * Prefer dark colors on non-Linux platforms as well
 * gui: Make gain/fps slider layout a lot more compact
 * python: Expose logging features
 * Use std::expected for error reporting where sensible
 * Make it easy to build the Python wheel and document the process
 * Poll the run result instead of blocking, and pass frame data as struct
 * ci: Build Linux binaries for arm64 as well
 * debian: Build Python package separately

### Bugfixes:
 * ci: Make setup build work on newer Inno-Setup-Action versions
 * gui: Recreate GL texture when the frame channel count changes
 * gui: Keep the record button in sync with the actual recording state
 * gui: Provide more details on connection issues
 * gui: Don't busy-wait on new frames
 * gui: Don't process events while we may be inside a library call
 * gui: Only upload new images to the texture, and request an OpenGL core profile
 * gui: Properly save window geometry/position and splitter state
 * Fix build with OpenCV 5.0
 * Fix unused-parameter warnings on Windows and drop GCC-unknown flags
 * lib: Ensure onControlValueChange is never run concurrently
 * Don't terminate recording on a single dropped frame
 * Re-enable sync pulses after an unexpected disconnect
 * videowriter: Drain encoder, fix timestamp alignment and stop busy-waiting
 * zstack: Never move past the end position, and use identical planes for 3D accumulation
 * Check at runtime that the FFmpeg version we use has the required codecs
 * Validate control value inputs more strictly and return errors on bad values
 * Make controlValue() return the current value of a control properly
 * Count our own acquired frames, don't trust the Miniscope hardware
 * Add proper export headers for Windows for the Miniscope library
 * Fix the displayed recording time, and set unixCaptureStartTime()
 * python: Raise proper errors instead of crashing in the NumPy converter
 * Don't report success when waiting for frames that did not arrive

### Miscellaneous:
 * README: Update APT repo install instructions
 * ci: Simplify Linux builds with debspawn
 * ci: Attach distro version slug to built package versions
 * Namespace miniscope library include path by directory
 * lib: Drop redundant udata in our callbacks
 * Document external triggers are unsupported
 * bno: Don't store the quality value in BNO output CSV table
 * Do nothing if run() is called multiple times in a row
 * Rename currentDisplayFrame() to fetchDisplayFrame(), and keep the display live
 * Harmonize frame count names in the API
 * Move private helper functions out of the public header
 * Remove the visible color channels API
 * python: Polish the example a little

Version 0.6.1
-------------
Released: 2026-03-10

### Features:
 * Build for Windows using MSYS2
 * windows: Embed icon in the executable
 * windows: Autobuild an installer for the app
 * Add working Miniscope firmware to the repository
 * Build binaries for Ubuntu 26.04

### Bugfixes:
 * Update image view widget with Syntalos version
 * Ensure libminiscope is included in Linux release tarballs

### Miscellaneous:
 * Update README and mention PPA
 * README: Add note on missing BNO info on newer Linux kernels
 * README: Add explicit Flatpak installation instructions
 * Add CITATION.cff
 * ci: Streamline build workflow, drop linux-release-binaries workflow

Version 0.6.0
-------------
Released: 2025-03-16

### Features:
 * Port to Qt6
 * Migrate zstack acquisition jobs to QPromise

### Bugfixes:
 * Resolve deprecation warning with newer FFmpeg
 * Use the older BNO query interface on Windows

### Miscellaneous:
 * Update Debian packaging, allow PPA upload

Version 0.5.2
-------------
Released: 2024-06-18

Features:
 * Display progress when acquiring a z-stack
 * Add experimental 3D stack accumulation function
 * Display acquisition progress in main window, instead of progress window

Bugfixes:
 * Ensure correct framerate is set for AVI containers
 * Enable sync pulses / recording mode even if we aren't recording

Miscellaneous:
 * py: Explicitly set video filename, rather than implicitly

Version 0.5.1
-------------
Released: 2023-08-16

Features:
 * Update to more modern image rendering code from Syntalos
 * Add option to highlight image saturation
 * Allow building for OpenGL ES, making arm64 builds easier
 * Implement a device hard-reset feature for developers

Bugfixes:
 * Clean up V4L buffers after OpenCV release for clean future device reconnect

Miscellaneous:
 * ci: Build for Debian 12 by default
 * cmake: Stop using PythonInterp, use Python3 module instead in FindNumPy

Version 0.5.0
-------------
Released: 2023-08-08

Features:
 * Implement BNO read support (if DAQ firmware version is high enough)
 * Perspective-warp BNO indicator according to received BNO data
 * Check firmware ABI when connecting to device
 * python: Update API to include the new BNO settings
 * zstack: Speed up acquisition by waiting for frame counts, not durations

Bugfixes:
 * Optimize PoMiDAQ SVG
 * Make "Stop recording" button more obviously clickable

Version 0.4.5
-------------
Released: 2022-08-20

Features:
 * Display device name for camera ID on Linux
 * Don't store data in temporary location by default
 * Update screenshots
 * Add Flatpak bundle build recipe
 * Validate & augment metainfo file using appstreamcli

Bugfixes:
 * Prevent in-tree builds
 * Give a better error when trying to build against GLES instead of OpenGL
 * cmake: Find more recent FFmpeg versions as well

Version 0.4.4
-------------
Released: 2021-12-16

Features:
 * Implement z-stack capture feature for Miniscopes with an EWL
 * Run display image rendering as event loop callback

Bugfixes:
 * Resolve a few compiler warnings

Version 0.4.3
-------------
Released: 2021-09-21

Features:
 * Make build instructions a bit more beginner-friendly

Bugfixes:
 * Give video display container a minimum width
 * Don't use deprecated FFmpeg API
 * Save view splitter sizes as byte array, instead of variant

Version 0.4.2
-------------
Released: 2021-05-24

Features:
 * Add spin boxes for sliding values, in addition to slider widgets
 * Start Miniscopes on highest framerate by default

Bugfixes:
 * Tell DAQ board when we are recording data
 * Consistently sort Miniscope controls
 * Save & restore main window splitter position

Version 0.4.1
-------------
Released: 2020-12-16

Features:
 * ci: Switch to GitHub Actions for CI
 * Build Win64 build with MSVC by default
 * Add ability to hotplug Miniscopes to DAQ board
 * Add reset button to display limits settings

Bugfixes:
 * Emulate timestamps on Windows if backend doesn't provide accurate ones
 * Update Miniscope hardware definition from upstream

Version 0.4.0
-------------
Released: 2020-08-08

Features:
 * Supports a lot more Miniscope devices, including the new Miniscope v4!
 * Drop dependency on Boost
 * Add Miniscope device type listing
 * Implement dynamic loading of scope controls
 * Implement the new DAQ board communication API
 * ui: Jump to the next useful toolbox page automatically
 * Simplify logging, write more detailed log in GUI
 * Add callback to be notified of control value changes
 * Don't call the raw encoding "No codec"/"None" in the UI
 * Drop background division, make background diff a display mode
 * Display storage location next to directory select button as well
 * Save & restore window geometry
 * Remember user data storage directory choice
 * Remember last Miniscope device choice
 * Add build instructions to README
 * Use newer compiler and libs on Windows
 * Update Python bindings

Bugfixes:
 * Convert more pixel formats in video writer
 * Only show tested and suitable codecs in selection box
 * Reset control value UI to device values upon reconnect
 * Remove explicit framerate delay code
 * Avoid sending lots of packets to the DAQ device in a short time
 * Make amount of permitted broken timestamps framerate-dependent
 * Auto-disconnect on device type change
 * Fix Python module build and switch to pybind11
 * Show GUI error message when failing to initialize OpenGL

Version 0.3.0
-------------
Released: 2020-04-04

Features:
 * Use the improved OpenGL image view widget from MazeAmaze/Syntalos
 * lib: Add callbacks to retrieve the current raw and displayed frames
 * Use C++17 as baseline
 * Add AppStream MetaInfo file
 * Simplify & improve time keeping code
 * lib: Permit API users to intercept & modify timestamps

Bugfixes:
 * Lower required OpenGL version, and fail if we can't initialize GL
 * Never even try to use dark mode on systems where it doesn't exist
 * Perform sanity check on timestamps before initializing timer with them
 * Do not expose clock details in API
 * Don't resend settings when framerate is low, this only causes the problem to get worse
 * Prefer V4L as camera API on Linux
 * Prefer DirectShow as camera API on Windows
 * Improve locking code and handle dropped init frames better

Version 0.2.0
-------------
Released: 2020-02-04

Features:
 * Allow setting an external capture start timepoint in libminiscope API
 * Add simple Python module on Linux
 * Actually record in grayscale, and store data as GRAY8 in FFV1
 * Set dark theme by default on some Linux desktops
 * Permit using UNIX timestamps (in msec) for data
 * Add options to change dark style, framerate ceiling and timestamp type

Bugfixes:
 * Rename stop button to disconnect, to make more clear what it does
 * Use camera driver timestamps instead of client timestamps

Version 0.1.0
-------------
Released: 2019-04-18

Features:
 * Initial release
