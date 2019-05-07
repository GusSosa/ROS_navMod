
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
import fisheye


# Define the total number of expected clicks for the homography.
TOT_H_CLICKS = 4

# Set desired frame width in pixels
# PIX_W = 1500
# Drew's laptop screen is 1600x900 so the above is too big.
PIX_W = 2400

# Define number of expected points
NUM_PT = 8


def tracker_init():

    # Setup SimpleBlobDetector parameters and set desired
    # blob properties
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
    params.filterByColor = False  # True previously
    params.blobColor = 255
    # params.minThreshold = 10
    # params.maxThreshold = 200
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 500  # NOTE: 750 for 1500W frame; 1500 for 3000W
    # params.maxArea = 2500
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.80  # 0.80 reguired for 3000W frame; 0.85 usable for 1500W
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

    # fisheye calibration
    calibration_data = fisheye.calibrate()
    [K, D] = [calibration_data['K'], calibration_data['D']]

    # grab the reference to the web cam
    # to use PC webcam, change src=0
    # to use connected USB camera, change src=1 or src=2...
    print("\n[INFO] starting video stream...")
    vs = cv2.VideoCapture(1)

    # set reolution
    print "Frame default resolution: (" + str(vs.get(cv2.CAP_PROP_FRAME_WIDTH)) + "; " + str(vs.get(cv2.CAP_PROP_FRAME_HEIGHT)) + ")"
    vs.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    print "Frame resolution set to: (" + str(vs.get(cv2.CAP_PROP_FRAME_WIDTH)) + "; " + str(vs.get(cv2.CAP_PROP_FRAME_HEIGHT)) + ")"
    print "Frame FPS: " + str(vs.get(cv2.CAP_PROP_FPS))

    time.sleep(0.5)

    # initialize the FPS throughput estimator
    fps = FPS().start()

    # We can now start tracking
    print("Press <T> in any window to start tracking")
    t1 = time.time()

    # loop over frames from the video stream
    while not rospy.is_shutdown():

        # grab the current frame, then handle if we are using a
        # VideoStream or VideoCapture object
        frame = vs.read()[1]

        # undistort fisheye
        dst = fisheye.undistort(K, D, frame)

        # update the FPS counter
        fps.update()
        fps.stop()

        # resize the frame (so we can process it faster) and grab the
        # frame dimensions
        # frame = imutils.resize(frame, width=PIX_W)
        dst = imutils.resize(dst, width=PIX_W)
        (Height, W) = frame.shape[:2]

        # blob detection and visual output of keypoints (ie red and blue dots)
        blob_detection_data = blob_detection(detector, dst)
        pix_com = blob_detection_data['pix_com']
        # # cv2.imshow('Blue Keypoints', blob_detection_data['bim_with_keypoints'])
        # # cv2.imshow('Red Keypoints', blob_detection_data['rim_with_keypoints'])
        cv2.imshow('Black Keypoints', blob_detection_data['blim_with_keypoints'])
        cv2.imshow('original', frame)
        cv2.imshow('undistorted', dst)
        # cv2.imshow('Silver Keypoints', blob_detection_data['slim_with_keypoints'])

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

        # auto homography testing
        # C2C distance between left most upper and second upper right holes should be ~56cm
        if key == ord('h'):

            print '\n' + 'Number of points found for homography: ' + str(len(blob_detection_data['blcom']))
            if len(blob_detection_data['blcom']) < 8:
                            # release webcam image
                vs.release()
                print('[ERROR: CORRECT NUMBER OF POINTS NOT FOUND]')

                # close all windows and end program
                cv2.destroyAllWindows()
                sys.exit()

            cv2.destroyAllWindows()
            print('[START OF HOMOGRPAHY TEST]')

            # Calculate homography matrix
            H = auto_homography(blob_detection_data)

            # Testing:
            # calculate the distance between two points in the local frame.
            calculate_homography2.test_H(H, vs, K, D)

            # release webcam image
            vs.release()
            print('[END OF TRACKING]')

            # close all windows and end program
            cv2.destroyAllWindows()
            sys.exit()

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
    lower_black = np.array([0, 0, 10])
    upper_black = np.array([180, 255, 110])
    #lower_black = np.array([0, 0, 20])
    #upper_black = np.array([180, 255, 60])
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
    blmedian = cv2.medianBlur(black_mask, 11)  # 2019_04.15 note: 5 works well for smaller homography dots/lower resolution
    smedian = cv2.medianBlur(silver_mask, 3)

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
    # starting in bottom left corner (origin), with points alternating as follows:
    # | - 7 |
    # | 6 - |
    # | ... |
    # | - 1 |
    # | 0 - |
    # center_dist_vert is the vertical distance between two consecutive points [cm]
    # center_dist_horz is the horizontal distance between the two vertical sets of points [cm]
    # NOTE: Distance between consecutive points is 7.62/2 (ie 1.5 inches)
    center_dist_vert = 3.81  # 1.5 inches
    center_dist_horz = 55.88  # 22 inches
    global_pts = np.vstack((np.tile([0, 1], 4) * center_dist_horz, np.array([i * center_dist_vert for i in range(8)]))).transpose()

    # Call homography function
    H = calculate_homography2.calc_H(uv, global_pts)
    print("Calculated homography is:")
    print(H)

    return H


if __name__ == "__main__":
    tracker_init()
    tracker_main()
    tracker_angle()
