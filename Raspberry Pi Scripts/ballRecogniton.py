# import packages
import cv2
import imutils
import time
import numpy as np
# initialize video capture
cap = cv2.VideoCapture(0)
# wait for camera to start
time.sleep(2)
# set color boundaries
lower_blue, upper_blue = np.array([100, 35, 140]), np.array([180, 255, 255])
lower_red, upper_red = np.array([0, 50, 50]), np.array([10, 255, 255])
# intialize updating while loop
while True:
    # reading video capture frame
    _, frame = cap.read()
    centerOfScreen = frame.shape[1] / 2
    # creating mask that is filtering for blue and red
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_blue, mask_red = cv2.inRange(hsv, lower_blue, upper_blue), cv2.inRange(hsv, lower_red, upper_red)
    result = cv2.bitwise_and(frame, frame, mask=mask_blue + mask_red)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    width, height = result.shape[:2]
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask_blue, mask_red = cv2.erode(mask_blue, None, iterations=2), cv2.erode(mask_red, None, iterations=2)
    mask_blue, mask_red = cv2.dilate(mask_blue, None, iterations=2), cv2.erode(mask_red, None, iterations=2)
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
            ((x, y), radius) = cv2.minEnclosingCircle(cnts_blue[c])
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
        try:
            min_distance = min(distances_from_center_blue)
        except TypeError:
            continue
        if min_distance < 0:
            print(f'turn left {abs(min_distance)}')
        else:
            print(f'turn turn {min_distance}')

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
# end video capture and destroy all video feeds
cap.release()
cv2.destroyAllWindows()
