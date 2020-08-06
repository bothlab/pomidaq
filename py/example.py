#!/usr/bin/env python3

#
# Small example on how to use the Miniscope library from Python
# In order to make this example work, first install the "miniscope" Python
# module with CMake.
# Alternatively you can also place the compiled "miniscope" module into a
# path you like and then change PYTHONPATH (sys.path) so Python can find it.
#

import sys
import cv2
from miniscope import Miniscope, ControlKind

MINISCOPE_DEVICE = 'Miniscope_V4'  # The device type we want to connect to
DEVICE_ID = 0  # the video device ID of our DAQ box
VIDEO_FILENAME = '/tmp/miniscope-test.mkv'  # name of the saved video

# create new Miniscope instance
mscope = Miniscope()

# disable some debug/info messages about data transmission
# to make the console output of this example easier to read
mscope.set_print_extra_debug(False)

# list all available miniscope types
print('Available Miniscope harware types:')
for dname in mscope.available_device_types:
    print(' * {}'.format(dname))

print()
print('Selecting: {}'.format(MINISCOPE_DEVICE))
if not mscope.load_device_config(MINISCOPE_DEVICE):
    print('Unable to load device configuration for {}: {}'.format(MINISCOPE_DEVICE, mscope.last_error),
          file=sys.stderr)
    sys.exit(1)

print('Available controls:')
controls = {}
for ctl in mscope.controls:
    controls[ctl.id] = ctl
    value_start = ctl.value_start
    if ctl.kind == ControlKind.SELECTOR:
        default_val = value_start
        if int(value_start) < len(ctl.labels):
            # replace the number with a human-readable string´
            default_val = ctl.labels[int(value_start)]
        print(' * {}: {} (default value: {})'.format(ctl.id, ctl.name, default_val))
    else:
        print(' * {}: {} (default value: {})'.format(ctl.id, ctl.name, value_start))

print('Connecting to device with ID: {}\n'.format(DEVICE_ID))
mscope.set_cam_id(DEVICE_ID)
if not mscope.connect():
    print('Unable to connect to Miniscope: {}'.format(mscope.last_error), file=sys.stderr)
    sys.exit(1)

if not mscope.run():
    print('Unable to start data acquisition: {}'.format(mscope.last_error), file=sys.stderr)
    sys.exit(1)

# adjust some controls
GAIN_VAL = 1
print('Setting gain control to {}'.format(controls['gain'].labels[GAIN_VAL]))
mscope.set_control_value('gain', GAIN_VAL)

EXCITATION_VAL = 20
print('Setting excitation control to {}'.format(EXCITATION_VAL))
mscope.set_control_value('led0', EXCITATION_VAL)

# prepare video recording
print('\n--------')
mscope.video_filename = VIDEO_FILENAME
print('Codec used for recording: {}'.format(mscope.video_codec))
print('Container used for recording: {}'.format(mscope.video_container))
print('Saving video in: {}'.format(mscope.video_filename))
print('--------\n')

if not mscope.start_recording(''):
    print('Unable to start video recording: {}'.format(mscope.last_error), file=sys.stderr)
    sys.exit(1)

try:
    print('Recording... Terminate with CTL+C\n')
    while mscope.is_running:
        frame = mscope.current_disp_frame
        if frame is not None:
            cv2.imshow('Miniscope Display', frame)
            cv2.waitKey(50)
except KeyboardInterrupt:
    print('User terminated recording. Shutting down.')
    print('Timestamp of last recorded frame: {}'.format(mscope.last_recorded_frame_time))
    mscope.stop()

if mscope.last_error:
    print('Error while acquiring data from Miniscope: {}'.format(mscope.last_error), file=sys.stderr)

mscope.disconnect()
cv2.destroyAllWindows()
