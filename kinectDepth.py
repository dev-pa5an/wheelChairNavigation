import cv2
import numpy as np
import ctypes
import _ctypes
import sys
import pykinect2
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

class KinectHandler(object):
    def __init__(self):
        self.kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth)
        self.depth_width, self.depth_height = self.kinect.depth_frame_desc.Width, self.kinect.depth_frame_desc.Height

    def get_depth_frame(self):
        if self.kinect.has_new_depth_frame():
            depth_frame = self.kinect.get_last_depth_frame()
            depth_frame = depth_frame.reshape((self.depth_height, self.depth_width)).astype(np.uint16)
            return depth_frame
        return None

    def close(self):
        self.kinect.close()

def main():
    kinect = KinectHandler()

    while True:
        # Capture depth frame
        depth_frame = kinect.get_depth_frame()
        if depth_frame is not None:
            # Display depth frame
            cv2.imshow('Depth Frame', depth_frame.astype(np.uint8))

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the Kinect resources
    kinect.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
