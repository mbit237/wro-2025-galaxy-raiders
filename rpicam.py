import cv2
from picamera2 import Picamera2


#green threshold
g_low_H = 29
g_high_H = 68
g_low_S = 47
g_high_S = 255
g_low_V = 41
g_high_V = 255

#red threshold
r_low_H = 112
r_high_H = 138
r_low_S = 41
r_high_S = 255
r_low_V = 114
r_high_V = 255

params = cv2.SimpleBlobDetector_Params()
params.filterByColor = True
params.blobColor = 255 # Detect white blobs. If set to 0, it'll detect black blobs.
params.filterByArea = True
params.minArea = 1000
params.maxArea = 100000
params.filterByCircularity = False
params.minCircularity = 0.1
params.filterByConvexity = False
params.minConvexity = 0.87
params.filterByInertia = False
params.minInertiaRatio = 0.01

detector = cv2.SimpleBlobDetector_create(params)

picam2 = Picamera2()
picam2.start() # Change the number to select a different camera

def detect_blob():
    frame = picam2.capture_array("main") # Grab one image frame from camera
    if frame is None:
        return None

    frame = cv2.GaussianBlur(frame, (5,5), 0)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    g_frame = cv2.inRange(hsv_frame, (g_low_H, g_low_S, g_low_V), (g_high_H, g_high_S, g_high_V))
    keypoints = detector.detect(g_frame) # Detects keypoints. Each keypoint will contain the x,y position, and size.
    largest_size = 0
    g_largest_keypoint = None
    for keypoint in keypoints:
        if keypoint.size > largest_size:
            largest_size = keypoint.size
            g_largest_keypoint = keypoint
    
    r_frame = cv2.inRange(hsv_frame, (r_low_H, r_low_S, r_low_V), (r_high_H, r_high_S, r_high_V))
    keypoints = detector.detect(r_frame) # Detects keypoints. Each keypoint will contain the x,y position, and size.
    largest_size = 0
    r_largest_keypoint = None
    for keypoint in keypoints:
        if keypoint.size > largest_size:
            largest_size = keypoint.size
            r_largest_keypoint = keypoint
    
    if g_largest_keypoint is None and r_largest_keypoint is None:
        return None
    elif g_largest_keypoint is None:
        return "r"
    elif r_largest_keypoint is None:
        return "g"
    else:
        if g_largest_keypoint.size > r_largest_keypoint.size:
            return "g"
        else:
            return "r"

    frame = cv2.drawKeypoints(frame, keypoints, 0, (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('result', frame) # Display the image in a window

while True:
    detect_blob()
    key = cv2.waitKey(30) # Wait 30ms for a key to be pressed
    if key == ord('q'): # If 'q' was pressed, exit the loop
        break

# while True:
#     frame = picam2.capture_array("main") # Grab one image frame from camera
#     if frame is None:
#         break

#     frame = cv2.GaussianBlur(frame, (5,5), 0)
#     frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     frame = cv2.inRange(frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
#     keypoints = detector.detect(frame) # Detects keypoints. Each keypoint will contain the x,y position, and size.
#     largest_size = 0
#     largest_keypoint = None
#     for keypoint in keypoints:
#         if keypoint.size > largest_size:
#             largest_size = keypoint.size
#             largest_keypoint = keypoint
#     if largest_keypoint:
#         print(largest_keypoint.pt)

#     frame = cv2.drawKeypoints(frame, keypoints, 0, (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


#     cv2.imshow('result', frame) # Display the image in a window

#     key = cv2.waitKey(30) # Wait 30ms for a key to be pressed
#     if key == ord('q'): # If 'q' was pressed, exit the loop
#         break
