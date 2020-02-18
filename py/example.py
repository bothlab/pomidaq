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
from miniscope import MiniScope, VideoCodec, VideoContainer

scope = MiniScope()
scope.set_cam_id(0)  # Connect to video camera 0

if not scope.connect():
    print('Unable to connect to Miniscope: {}'.format(scope.last_error), file=sys.stderr)
    sys.exit(1)

if not scope.run():
    print('Unable to start data acquisition: {}'.format(scope.last_error), file=sys.stderr)
    sys.exit(1)

print('\n--------')
scope.video_filename = '/tmp/miniscope-test.mkv'
print('Codec used for recording: {}'.format(scope.video_codec))
print('Container used for recording: {}'.format(scope.video_container))
print('Saving video in: {}'.format(scope.video_filename))
print('--------\n')

if not scope.start_recording(''):
    print('Unable to start video recording: {}'.format(scope.last_error), file=sys.stderr)
    sys.exit(1)

try:
    print('Recording... Terminate with CTL+C')
    while scope.running:
        frame = scope.current_disp_frame
        if frame is not None:
            cv2.imshow('Miniscope Display', frame)
            cv2.waitKey(50)
except KeyboardInterrupt:
    print('User terminated recording. Shutting down.')
    scope.stop()

if scope.last_error:
    print('Error while acquiring data from Miniscope: {}'.format(scope.last_error), file=sys.stderr)

scope.disconnect()
cv2.destroyAllWindows()
