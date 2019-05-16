
# opencv_object_tracker.py
# Primary object tracking program, includes both the tracking initilaization (tracker_init())
# and the main tracking loop (tracking_main())

# import the necessary packages
import numpy as np
import time
import cv2
import sys
import imutils
import rospy
import calculate_homography2
import fisheye
import os


# Define the total number of expected clicks for the homography.
TOT_H_CLICKS = 4

# Set desired frame width in pixels
# PIX_W = 1500
# Drew's laptop screen is 1600x900 so the above is too big.
PIX_W = 2400

# Define number of expected homography dots
NUM_PT = 10

# define HSV color ranges for blob detector
lower_blue = np.array([90, 50, 50])
upper_blue = np.array([130, 255, 255])
lower_red1 = np.array([0, 50, 50])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 50, 50])
upper_red2 = np.array([180, 255, 255])
lower_black = np.array([0, 0, 0])
upper_black = np.array([255, 255, 60])


def tracker_init():

    # Setup SimpleBlobDetector parameters and set desired
    # # blob properties
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
    params.filterByColor = True  # True previously
    params.blobColor = 255
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 1200  # NOTE: 1000 for 05-15-2019, NOTE: 750 for 1500W frame; 1500 for 3000W
    # params.maxArea = 2500
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.40  # 0.80 reguired for 3000W frame; 0.85 usable for 1500W

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:
        detector = cv2.SimpleBlobDetector(params)
    else:
        detector = cv2.SimpleBlobDetector_create(params)

    # fisheye calibration
    try:  # check if K, D calibration arrays already exist in the current directory
        K, D = np.load(os.path.dirname(os.path.abspath(__file__)) + '/calibration_data.npy')
        print '[IMPORTING CALIBRATION DATA]'
    except IOError:  # if file dooesn't exist, calibrate using the new images and save them as .npy file
        calibration_data = fisheye.calibrate()
        [K, D] = [calibration_data['K'], calibration_data['D']]
        np.save(os.path.dirname(os.path.abspath(__file__)) + '/calibration_data', np.array([K, D]))

    # grab the reference to the web cam
    # to use PC webcam, change src=0
    # to use connected USB camera, change src=1 or src=2...
    print("\n[INFO] starting video stream...")
    vs = cv2.VideoCapture(1)

    # set webcam resolution
    print "Frame default resolution: (" + str(vs.get(cv2.CAP_PROP_FRAME_WIDTH)) + "; " + str(vs.get(cv2.CAP_PROP_FRAME_HEIGHT)) + ")"
    # vs.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    print "Frame resolution set to: (" + str(vs.get(cv2.CAP_PROP_FRAME_WIDTH)) + "; " + str(vs.get(cv2.CAP_PROP_FRAME_HEIGHT)) + ")"
    print "Frame FPS: " + str(vs.get(cv2.CAP_PROP_FPS))

    # delay to reduce overexposure, initalize keyboard exception
    time.sleep(0.5)
    key = None

    # We can now start tracking
    print('Press <T> in any window to start tracking')

    # loop over frames from the video stream
    while not rospy.is_shutdown():

        # grab the current frame, then handle if we are using a
        # VideoStream or VideoCapture object
        frame = vs.read()[1]

        # undistort fisheye
        dst = fisheye.undistort(K, D, frame)

        # resize the frame and grab the frame dimensions
        dst = imutils.resize(dst, width=PIX_W)
        (Height, W) = dst.shape[:2]

        # blob detection and visual output of keypoints (ie red and blue dots)
        blob_detection_data = blob_detection(detector, dst)
        cv2.imshow('Black Keypoints', blob_detection_data['blim_with_keypoints'])
        # cv2.imshow('Blue Keypoints', blob_detection_data['bim_with_keypoints'])
        # cv2.imshow('Red Keypoints', blob_detection_data['rim_with_keypoints'])
        # cv2.imshow('original', frame)
        # cv2.imshow('undistorted', dst)

        # reset keyboard interrupt, unless the 't' key has been pressed
        key = (cv2.waitKey(1) & 0xFF) if key != ord('t') else key

        # if the "q" key is pressed, quit the program
        if key == ord('q'):

            # release webcam image
            vs.release()
            print('[END OF TRACKING]')

            # close all windows and end program
            cv2.destroyAllWindows()
            sys.exit()

        # if the 't' key is selected, we are going to start tracking
        if key == ord('t'):

            # eight homography dots not detected
            if len(blob_detection_data['blcom']) != NUM_PT:
                # restart whiel loop to keep searching for eight dots
                pass

            # exactly eight dots found
            else:
                # Calculate homography matrix
                H = auto_homography(blob_detection_data)
                # Calculate the x-unit vector for world frame (ie the [1,0] vector for this frame),
                # used to calculate the vertebra rotation.
                # Uses the first homography dot (ie world frame origin) and the second to last dot,
                # which is in line, directly above the first dot
                # Note that the unit vector between these two should nominally be [0,1]
                blcom = blob_detection_data['blcom']
                Q = np.linalg.inv(H)  # calculate Q matrix for pixel- real world conversion
                u1 = np.array(blcom[0, :])  # location of first homography dot (ie world frame origin) in pixel frame
                u2 = np.array(blcom[NUM_PT - 2, :])  # location of dowel pin 2 in pixel frame
                u1h = np.r_[np.transpose([u1]), np.array([[1]])]
                u2h = np.r_[np.transpose([u2]), np.array([[1]])]
                u1true = np.dot(Q, u1h)[0:2, :] / np.dot(Q, u1h)[2, :]  # location of first homography dot (ie world frame origin)
                u2true = np.dot(Q, u2h)[0:2, :] / np.dot(Q, u2h)[2, :]  # location of second to last homography dot (in line with 1st)
                umag = np.linalg.norm(u1true - u2true)  # magnitude of x-unit vector
                uvec = np.flip((abs(u1true - u2true)) / umag)  # x-unit vector for world frame
                # end initialization function and start tracking program
                cv2.destroyAllWindows()
                print('\n[START OF TRACKING]' + '\n' + 'Press <Q> in any window to stop tracking')
                break

    return(detector, vs, Q, uvec, K, D, key)


def tracker_main(detector, K, D, vs, originpix):

    # grab the current frame, then handle if we are using a
    # VideoStream or VideoCapture object
    frame = vs.read()[1]
    # undistort fisheye
    dst = fisheye.undistort(K, D, frame)
    # resize the frame and grab the frame dimensions
    dst = imutils.resize(dst, width=PIX_W)

    # blob detection and visual output of keypoints (ie red and blue dots)
    blob_detection_data = blob_detection(detector, dst)
    # cv2.imshow('Blue Keypoints', blob_detection_data['bim_with_keypoints'])
    # cv2.imshow('Red Keypoints', blob_detection_data['rim_with_keypoints'])

    # show the output frame
    if originpix is not None:
        # print tuple(originpix.reshape(1, -1)[0])
        cv2.circle(dst, tuple(originpix.reshape(1, -1)[0]), 20, (0, 0, 255), -1)
    cv2.imshow('Frame', dst)

    # reset keyboard interrupt key
    key = cv2.waitKey(1) & 0xFF

    return (blob_detection_data, key)


def blob_detection(detector, dst):

    # Convert BGR to HSV
    hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only blue and red colors
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = red_mask1 + red_mask2
    black_mask = cv2.inRange(hsv, lower_black, upper_black)

    # Median blur to smooth edges and output image
    bmedian = cv2.medianBlur(blue_mask, 9)
    rmedian = cv2.medianBlur(red_mask, 13)
    blmedian = cv2.medianBlur(black_mask, 13)  # 2019_04.15 note: 5 works well for smaller homography dots/lower resolution

    # # Otsu's thresholding after Gaussian filtering
    # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # blur = cv2.GaussianBlur(gray_frame, (5, 5), 0)
    # ret3, th3 = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Detect blobs.
    bkeypoints = detector.detect(bmedian)
    rkeypoints = detector.detect(rmedian)
    blkeypoints = detector.detect(blmedian)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    bim_with_keypoints = cv2.drawKeypoints(bmedian, bkeypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    rim_with_keypoints = cv2.drawKeypoints(rmedian, rkeypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    blim_with_keypoints = cv2.drawKeypoints(blmedian, blkeypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Finds center of mass of both red and blue dots in pixel values
    # and converts these float lists into integer numpy arrays
    bcom = np.array([int(i) for i in bkeypoints[0].pt]) if bkeypoints else np.array([0, 0])
    rcom = np.array([int(i) for i in rkeypoints[0].pt]) if rkeypoints else np.array([0, 0])
    blcom = np.array([blkeypoints[i].pt for i in range(int(len(blkeypoints)))]).astype(int)
    # pix_com = np.array([bcom, rcom])
    pix_com = np.array([rcom, bcom])
    # print 'red: ' + str(pix_com[0, :])
    # print 'blue: ' + str(pix_com[1, :])

    return {'pix_com': pix_com, 'bim_with_keypoints': bim_with_keypoints, 'rim_with_keypoints': rim_with_keypoints,
            'blim_with_keypoints': blim_with_keypoints, 'blcom': blcom}


def auto_homography(blob_detection_data):

    print('\n[START OF HOMOGRAPHY CALCULATION]')

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
    global_pts = np.vstack((np.tile([0, 1], NUM_PT / 2) * center_dist_horz, np.array([i * center_dist_vert for i in range(NUM_PT)]))).transpose()

    # Call homography function
    H = calculate_homography2.calc_H(uv, global_pts)
    print("Calculated homography is:")
    print(H)

    return H


if __name__ == "__main__":
    tracker_init()
    tracker_main()
    tracker_angle()
