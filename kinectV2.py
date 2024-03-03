import cv2
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
import numpy as np
import time

kinectD = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth)

while True:
    stime = time.time()

    if kinectD.has_new_depth_frame():
        frameD = kinectD.get_last_depth_frame()
        frameDepth = kinectD._depth_frame_data
        frameD = frameD.astype(np.uint8)
        frameD = np.reshape(frameD,(424, 512))
        frameD = cv2.cvtColor(frameD, cv2.COLOR_GRAY2BGR)

        def click_event(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                Pixel_Depth = frameDepth[(((y - 1) * 512) + x)]
                print(Pixel_Depth)

        cv2.imshow('frameD', frameD)
        cv2.setMouseCallback('frameD', click_event)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
