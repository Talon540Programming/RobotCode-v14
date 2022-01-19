# import packages
import cv2
import time
import numpy as np
import datetime
from threading import Thread
# defining class for frame update
class FPS:
    def __init__(self):
        self._start = None
        self._end = None
        self._numFrames = 0
    def start(self):
        self._start = datetime.datetime.now()
        return self
    def stop(self):
        self._end = datetime.datetime.now()
    def update(self):
        self._numFrames += 1
    def elapsed(self):
        return (self._end - self._start).total_seconds()
    def fps(self):
        return self._numFrames / self.elapsed()
# defining class for threading VideoCapture frame read
class WebcamVideoStream:
    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
    def start(self):
        Thread(target=self.update, args=()).start()
        return self
    def update(self):
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()
    def read(self):
        return self.frame
    def stop(self):
        self.stopped = True
# initialize video capture
fps = FPS().start()
vs = WebcamVideoStream(src=0).start()
# wait for camera to start
time.sleep(2)
# set color boundaries
lower_blue, upper_blue = np.array([100, 35, 140]), np.array([180, 255, 255])
lower_red, upper_red = np.array([0, 50, 50]), np.array([10, 255, 255])
def detectBalls():
    global fps, lower_blue, upper_blue, lower_red, upper_red
    start = time.perf_counter()
    stop = time.perf_counter()
    # intialize updating while loop
    while stop - start < 30:
        # reading video capture frame
        frame = vs.read()
        centerOfScreen = frame.shape[1] / 2
        # creating mask that is filtering for blue and red
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_blue, mask_red = cv2.inRange(hsv, lower_blue, upper_blue), cv2.inRange(hsv, lower_red, upper_red)
        result = cv2.bitwise_and(frame, frame, mask=mask_blue + mask_red)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        width, height = result.shape[:2]
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask_blue, mask_red = cv2.erode(mask_blue, None, iterations=2), cv2.erode(mask_red, None, iterations=2)
        mask_blue, mask_red = cv2.dilate(mask_blue, None, iterations=2), cv2.dilate(mask_red, None, iterations=2)
        # finding contour extremeties from mask
        cnts_blue, _ = cv2.findContours(mask_blue.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        cnts_red, _ = cv2.findContours(mask_red.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        centers_blue = []
        distances_from_center_blue = []
        centers_red = []
        # drawing circles for each area in cnts
        # for blue regions
        if len(cnts_blue) > 0:
            for c in range(len(cnts_blue) - 1):
                (x, y), radius = cv2.minEnclosingCircle(cnts_blue[c])
                M = cv2.moments(cnts_blue[c])
                try:
                    centers_blue.append((int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])))
                except ZeroDivisionError:
                    continue
                if radius > 100:
                    cv2.circle(result, (centers_blue[-1]), int(radius), (255, 0, 0), 5)
                    cv2.imwrite("circled_frame.png", cv2.resize(result, (int(height / 2), int(width / 2))))
                    cv2.circle(result, centers_blue[-1], 5, (0, 255, 255), -1)
                    distances_from_center_blue.append(centers_blue[-1][0] - centerOfScreen)
            # prints the distance from the closest ball to the center and how far to turn the robot;
            # will be replaced by calls to the RoboRio controls in the future
            if len(distances_from_center_blue) > 0:
                try:
                    min_distance = min(distances_from_center_blue)
                except TypeError:
                    continue
                if min_distance < 0:
                    print('turn left {}'.format(abs(min_distance)))
                else:
                    print('turn right {}'.format(min_distance))
            else:
                continue
        # for red regions
        if len(cnts_red) > 0:
            for c in range(len(cnts_red) - 1):
                ((x, y), radius) = cv2.minEnclosingCircle(cnts_red[c])
                M = cv2.moments(cnts_red[c])
                try:
                    centers_red.append((int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])))
                except ZeroDivisionError:
                    continue
                if radius > 100:
                    cv2.circle(result, (centers_red[-1]), int(radius), (0, 0, 255), 5)
                    cv2.imwrite("circled_frame.png", cv2.resize(result, (int(height / 2), int(width / 2))))
                    cv2.circle(result, centers_red[-1], 5, (0, 255, 255), -1)
        # showing current frame
        cv2.imshow("Result", result)
        # if q is pressed, the while loop breaks
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        stop = time.perf_counter()
        fps.update()
    # end video capture and destroy all video feeds
    fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
    vs.stop()
    cv2.destroyAllWindows()
