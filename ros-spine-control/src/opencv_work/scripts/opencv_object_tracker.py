
# opencv_object_tracker.py
# Primary object tracking program, includes both the tracking initilaization (tracker_init())
# and the main tracking loop (tracking_main())

# import the necessary packages
from imutils.video import FPS
import numpy as np
import time
import cv2
import sys
import imutils
import math
import rospy
import calculate_homography2


# Define the total number of expected clicks for the homography.
TOT_H_CLICKS = 4

# Set desired frame width in pixels
# PIX_W = 1500
# Drew's laptop screen is 1600x900 so the above is too big.
PIX_W = 1750


def tracker_init():

    # Setup SimpleBlobDetector parameters and set desired
    # blob properties
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
    params.filterByColor = False
    params.blobColor = 255
    # params.minThreshold = 10
    # params.maxThreshold = 200
    # Filter by Area.
    params.filterByArea = True  # 1000 previously
    params.minArea = 500
    # params.maxArea = 2500
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.85
    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.95
    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.15
    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:
        detector = cv2.SimpleBlobDetector(params)
    else:
        detector = cv2.SimpleBlobDetector_create(params)

    # initialize the bounding box coordinates of the object we are going
    # to track, and scale object
    pix_com = np.zeros((2, 2), dtype=np.float32)
    # bkeypoints = None
    # rkeypoints = None

    # grab the reference to the web cam
    # to use PC webcam, change src=0
    # to use connected USB camera, change src=1 or src=2...
    print("[INFO] starting video stream...")
    vs = cv2.VideoCapture(1)

    # test
    # des_fps = 60
    # vs.set(cv2.CAP_PROP_FPS, des_fps)

    time.sleep(0.5)

    # initialize the FPS throughput estimator
    fps = FPS().start()

    # # Get the homography for this run of the tracker.
    # print("[INFO] Now calculating the homography.")
    # # We'll store the clicked points for the homography in this array:
    # h_pts = []
    # # First, create a callback for mouse clicks. (Adapted from EE206A Lab 4)

    # def on_mouse_click(event, x, y, flag, param):
    #     # we'll only capture four clicks.
    #     if len(h_pts) < TOT_H_CLICKS:
    #         if(event == cv2.EVENT_LBUTTONUP):
    #             point = (x, y)
    #             print "Point Captured: " + str(point)
    #             h_pts.append(point)

    # # Get the positions of the clicks.
    # # We want to wait until the user is ready to start.
    # # A nifty way to do so is to have them type something into the terminal, then discard what was typed.
    # raw_input("Please move out of the frame, then press enter to capture the frame that will be used for the homography.")
    # print("Click four times to get the points for the homography.")
    # # Next, get the four clicks. Read this frame:
    # # grab the current frame, then handle if we are using a
    # # VideoStream or VideoCapture object
    # frame = vs.read()[1]

    # # resize and show the newly-captured frame
    # frame = imutils.resize(frame, width=PIX_W)
    # cv2.imshow("Frame", frame)

    # # Tell OpenCV that it should call 'on_mouse_click' when the user
    # # clicks the window. This will add clicked points to our list
    # cv2.setMouseCallback("Frame", on_mouse_click, param=1)
    # # Finally, loop until we've got enough clicks. Just a blocking wait.
    # while len(h_pts) < TOT_H_CLICKS:
    #     if rospy.is_shutdown():
    #         raise KeyboardInterrupt
    #     # just block
    #     cv2.waitKey(10)
    # # We should now have four elements in the h_pts array now.
    # print("Captured points for the homography:")
    # print(h_pts)
    # # Convert the Python list of points to a NumPy array of the form
    # #   | u1 u2 u3 u4 |
    # #   | v1 v2 v3 v4 |
    # uv = np.array(h_pts).T

    # # Specify the points in the global frame, i.e. the frame of the vertebra.
    # # Assume the points go (bottom left, top left, top right, bottom right)
    # # and that the origin is at the bottom left.
    # global_pts = np.ndarray((4, 2))

    # print global_pts
    # # On 2018-12-5, the blue tape on the test setup was 16 squares of 2cm each,
    # # so that's 32 cm along each edge.
    # edge = 32
    # # the coordinates are then,
    # # for points 0 to 4 in the world frame,
    # global_pts[0, :] = [0, 0]
    # global_pts[1, :] = [0, edge]
    # global_pts[2, :] = [edge, edge]
    # global_pts[3, :] = [edge, 0]

    # # and can now call the function itself.
    # # as of 2018-12-5, uv is 2x4 but global_pts is 4x2. STANDARDIZE THIS.
    # H = calculate_homography.calc_H(uv, global_pts)
    # print("Calculated homography is:")
    # print(H)

    # # close the current window before proceeding.
    # cv2.destroyWindow("Frame")

    # # Testing:
    # # calculate the distance between two points in the local frame.
    # calculate_homography.test_H(H, vs)

    # # Back to the rest of the script.
    # print('Press <S> in the "Frame" window to select ROI of first object')

    # We want to wait until the user is ready to start.
    # A nifty way to do so is to have them type something into the terminal, then discard what was typed.
    # raw_input("Please move out of the frame, then press enter to capture the frame that will be used for the homography.")

    # resize the frame (so we can process it faster) and grab the
    # frame dimensions
    frame1 = vs.read()[1]
    frame1 = imutils.resize(frame1, width=PIX_W)
    cv2.imshow('Homography Frame', frame1)
    (Height, W) = frame1.shape[:2]

    # blob detection and visual output of keypoints (ie red and blue dots)
    blob_detection_data = blob_detection(detector, frame1)

    # Calculate homography matrix
    # H = auto_homography(blob_detection_data)

    # We can now start tracking
    print("Press <T> in any window to start tracking")

    # loop over frames from the video stream
    while not rospy.is_shutdown():

        # grab the current frame, then handle if we are using a
        # VideoStream or VideoCapture object
        frame = vs.read()[1]

        # update the FPS counter
        fps.update()
        fps.stop()

        # resize the frame (so we can process it faster) and grab the
        # frame dimensions
        frame = imutils.resize(frame, width=PIX_W)
        (Height, W) = frame.shape[:2]

        # blob detection and visual output of keypoints (ie red and blue dots)
        blob_detection_data = blob_detection(detector, frame)
        pix_com = blob_detection_data['pix_com']
        # cv2.imshow('Blue Keypoints', blob_detection_data['bim_with_keypoints'])
        # cv2.imshow('Red Keypoints', blob_detection_data['rim_with_keypoints'])
        # cv2.imshow('Black Keypoints', blob_detection_data['blim_with_keypoints'])
        cv2.imshow('Silver Keypoints', blob_detection_data['slim_with_keypoints'])
        print(blob_detection_data['blcom'])

        # initialize the set of information we'll be displaying on
        # the frame
        info = [
            ("Tracker", "Blob Detection"),
            ("Success", "Yes" if pix_com.any() else "No"),
            ("FPS", "{:.2f}".format(fps.fps())),
        ]

        # loop over the info tuples and draw them on our frame
        for (i, (k, v)) in enumerate(info):
            text = "{}: {}".format(k, v)
            cv2.putText(frame, text, (10, Height - ((i * 20) + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # cv2.imshow('frame', frame)

        # reset keyboar interrupt
        key = cv2.waitKey(1) & 0xFF

        # if the "q" key is pressed, quit the program
        if key == ord("q"):

            # release webcam image
            vs.release()
            print('[END OF TRACKING]')

            # close all windows and end program
            cv2.destroyAllWindows()
            sys.exit()

        # if the 't' key is selected, we are going to start tracking
        if key == ord('t'):

            print('[START OF TRACKING]' + '\n' + 'Press <Q> in any window to stop tracking')
            # start the FPS estimator
            fps = FPS().start()

            break

    return(detector, pix_com, vs, H, fps, key)


def tracker_main(detector, pix_com, vs, fps):

    # grab the current frame, then handle if we are using a
    # VideoStream or VideoCapture object
    frame = vs.read()[1]

    # resize the frame (so we can process it faster) and grab the
    # frame dimensions
    frame = imutils.resize(frame, width=PIX_W)
    (Height, W) = frame.shape[:2]

    # update the FPS counter
    fps.update()
    fps.stop()

    info = [
        ("Tracker", "Blob Detection"),
        ("Success", "Yes" if pix_com.any() else "No"),
        ("FPS", "{:.2f}".format(fps.fps())),
    ]

    # loop over the info tuples and draw them on our frame
    for (i, (k, v)) in enumerate(info):
        text = "{}: {}".format(k, v)
        cv2.putText(frame, text, (10, Height - ((i * 20) + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # show the output frame
    cv2.imshow("Frame", frame)

    # blob detection and visual output of keypoints (ie red and blue dots)
    blob_detection_data = blob_detection(detector, frame)
    pix_com = blob_detection_data['pix_com']
    cv2.imshow('Blue Keypoints', blob_detection_data['bim_with_keypoints'])
    cv2.imshow('Red Keypoints', blob_detection_data['rim_with_keypoints'])

    # reset keyboard interrupt key
    key = cv2.waitKey(1) & 0xFF

    return (pix_com, key)


def tracker_angle(com1, com2):

    x = com1[0, 0] - com2[0, 0]
    y = com1[0, 1] - com2[0, 1]
    theta_deg = np.array(math.degrees(math.asin(y / x)), dtype=np.float32)

    return theta_deg


def blob_detection(detector, frame):

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([120, 255, 255])
    lower_red = np.array([160, 50, 50])
    upper_red = np.array([180, 255, 255])
    lower_black = np.array([0, 0, 20])
    upper_black = np.array([180, 255, 60])
    lower_silver = np.array([0, 0, 160])
    upper_silver = np.array([180, 255, 240])

    # Threshold the HSV image to get only blue and red colors
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    black_mask = cv2.inRange(hsv, lower_black, upper_black)
    silver_mask = cv2.inRange(hsv, lower_silver, upper_silver)

    # Median blur to smooth edges and output image
    bmedian = cv2.medianBlur(blue_mask, 11)
    rmedian = cv2.medianBlur(red_mask, 11)
    blmedian = cv2.medianBlur(black_mask, 11)  # 2019_04.15 note: value of 5 works well for smaller homography dots
    smedian = cv2.medianBlur(silver_mask, 2)

    # Otsu's thresholding after Gaussian filtering
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray_frame, (5, 5), 0)
    ret3, th3 = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Detect blobs.
    bkeypoints = detector.detect(bmedian)
    rkeypoints = detector.detect(rmedian)
    blkeypoints = detector.detect(blmedian)
    skeypoints = detector.detect(smedian)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    bim_with_keypoints = cv2.drawKeypoints(bmedian, bkeypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    rim_with_keypoints = cv2.drawKeypoints(rmedian, rkeypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    blim_with_keypoints = cv2.drawKeypoints(blmedian, blkeypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    slim_with_keypoints = cv2.drawKeypoints(smedian, skeypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Finds center of mass of both red and blue dots in pixel values
    # and converts these float lists into integer numpy arrays
    bcom = np.array([int(i) for i in bkeypoints[0].pt]) if bkeypoints else np.array([0, 0])
    rcom = np.array([int(i) for i in rkeypoints[0].pt]) if rkeypoints else np.array([0, 0])
    blcom = np.array([blkeypoints[i].pt for i in range(int(len(blkeypoints)))]).astype(int)
    scom = np.array([skeypoints[i].pt for i in range(int(len(skeypoints)))]).astype(int)
    pix_com = np.array([bcom, rcom])

    return {'pix_com': pix_com, 'bim_with_keypoints': bim_with_keypoints, 'rim_with_keypoints': rim_with_keypoints,
            'blim_with_keypoints': blim_with_keypoints, 'slim_with_keypoints': slim_with_keypoints,
            'blcom': blcom}


def auto_homography(blob_detection_data):

    print("[INFO] Now calculating the homography.")

    # NOTE: blcom is pixel COM of the black dots in camera frame, of the form:
    # | x1 y1 |
    # | x2 y2 |
    # | ..... |
    # | xN yN |
    uv = blob_detection_data['blcom']

    # Specify global points in the global frame (i.e. vertebra frame)
    # This assumes all 8 homography dots have been located,
    # starting in bottom right corner (origin), with points alternating as follows:
    # | - 8 |
    # | 7 - |
    # | - 6 |
    # | ... |
    # | 1 - |
    # | - 0 |
    # center_dist_vert is the vertical distance between two consecutive points [in]
    # center_dist_horz is the horizontal distance between the two vertical sets of points [in]
    center_dist_vert = 2.5
    center_dist_horz = 15
    global_pts = np.vstack((np.array([i * center_dist_vert for i in range(8)]), np.tile([0, 1], 4) * -center_dist_horz)).transpose()
    # print global_pts

    # Call homography function
    H = calculate_homography2.calc_H(uv, global_pts)
    print("Calculated homography is:")
    print(H)

    return H


if __name__ == "__main__":
    tracker_init()
    tracker_main()
    tracker_angle()
