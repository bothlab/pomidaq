#!/usr/bin/env python3

#
# Small example on how to use the Miniscope library from Python
# In order to make this example work, first install the Python
# module with CMake.
# Alternatively you can also place the miniscope library into a path you like
# and then change PYTHONPATH (sys.path) to find it.
#

import sys
import cv2
from miniscope import Miniscope, VideoCodec, VideoContainer

MINISCOPE_DEVICE = 'Miniscope_V4'

mscope = Miniscope()

# list all available miniscope types
print('Available Miniscope harware types:')
for dname in mscope.available_device_types:
    print(' * {}'.format(dname))

print('Selecting: {}'.format(MINISCOPE_DEVICE))

if not mscope.load_device_config(MINISCOPE_DEVICE):
    print('Unable to load device configuration for {}: {}'.format(MINISCOPE_DEVICE, mscope.last_error),
          file=sys.stderr)
    sys.exit(1)

mscope.set_cam_id(0)  # Connect to video camera 0

if not mscope.connect():
    print('Unable to connect to Miniscope: {}'.format(mscope.last_error), file=sys.stderr)
    sys.exit(1)

if not mscope.run():
    print('Unable to start data acquisition: {}'.format(mscope.last_error), file=sys.stderr)
    sys.exit(1)

print('\n--------')
mscope.video_filename = '/tmp/miniscope-test.mkv'
print('Codec used for recording: {}'.format(mscope.video_codec))
print('Container used for recording: {}'.format(mscope.video_container))
print('Saving video in: {}'.format(mscope.video_filename))
print('--------\n')

if not mscope.start_recording(''):
    print('Unable to start video recording: {}'.format(mscope.last_error), file=sys.stderr)
    sys.exit(1)

try:
    print('Recording... Terminate with CTL+C')
    while mscope.is_running:
        frame = mscope.current_disp_frame
        if frame is not None:
            cv2.imshow('Miniscope Display', frame)
            cv2.waitKey(50)
except KeyboardInterrupt:
    print('User terminated recording. Shutting down.')
    mscope.stop()

if mscope.last_error:
    print('Error while acquiring data from Miniscope: {}'.format(mscope.last_error), file=sys.stderr)

mscope.disconnect()
cv2.destroyAllWindows()
