
# opencv_object_tracker.py
# Primary object tracking program, includes both the tracking initilaization (tracker_init())
# and the main tracking loop (tracking_main())

# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import argparse
import time
import cv2
import sys
import imutils
import math
import rospy
import calculate_homography

# Define the total number of expected clicks for the homography.
TOT_H_CLICKS = 4

def tracker_init():

    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()

    # use for video file
    ap.add_argument("-v", "--video", type=str,
                    help="path to input video file")
    ap.add_argument("-t", "--tracker", type=str, default="kcf",
                    help="OpenCV object tracker type")
    args = vars(ap.parse_args())

    # initialize a dictionary that maps strings to their corresponding
    # OpenCV object tracker implementations
    OPENCV_OBJECT_TRACKERS = {
        "csrt": cv2.TrackerCSRT_create,
        "kcf": cv2.TrackerKCF_create,
        "boosting": cv2.TrackerBoosting_create,
        "mil": cv2.TrackerMIL_create,
        "tld": cv2.TrackerTLD_create,
        "medianflow": cv2.TrackerMedianFlow_create,
        "mosse": cv2.TrackerMOSSE_create
    }

    # initialize the bounding box coordinates of the object we are going
    # to track, and scale object
    trackers = cv2.MultiTracker_create()
    initBB = np.array([], [])
    pix_com = np.zeros((2, 2), dtype=np.float32)
    print pix_com

    # grab the reference to the web cam
    # to use PC webcam, change src=0
    # to use connected USB camera, change src=1 or src=2...
    print("[INFO] starting video stream...")
    vs = VideoStream(src=1).start()
    time.sleep(0.5)

    # initialize the FPS throughput estimator
    fps = FPS().start()

    # Get the homography for this run of the tracker.
    print("[INFO] Now calculating the homography.")
    # We'll store the clicked points for the homography in this array:
    h_pts = []
    # First, create a callback for mouse clicks. (Adapted from EE206A Lab 4)
    def on_mouse_click(event,x,y,flag,param):
        # we'll only capture four clicks.
        if len(h_pts) < TOT_H_CLICKS:
            if(event == cv2.EVENT_LBUTTONUP):
                point = (x,y)
                print "Point Captured: " + str(point)
                h_pts.append(point)

    # Get the positions of the clicks.
    # We want to wait until the user is ready to start.
    # A nifty way to do so is to have them type something into the terminal, then discard what was typed.
    raw_input("Please move out of the frame, then press enter to capture the frame that will be used for the homography.")
    print("Click four times to get the points for the homography.")
    # Next, get the four clicks. Read this frame:
    # grab the current frame, then handle if we are using a
    # VideoStream or VideoCapture object
    frame = vs.read()
    frame = frame[1] if args.get("video", False) else frame
    # show the newly-captured frame
    cv2.imshow("Frame", frame)

    # Tell OpenCV that it should call 'on_mouse_click' when the user
    # clicks the window. This will add clicked points to our list
    cv2.setMouseCallback("Frame", on_mouse_click, param=1)
    # Finally, loop until we've got enough clicks. Just a blocking wait.
    while len(h_pts) < TOT_H_CLICKS:
        if rospy.is_shutdown():
            raise KeyboardInterrupt
        # just block
        cv2.waitKey(10)
    # We should now have four elements in the h_pts array now.
    print("Captured points for the homography:")
    print(h_pts)
    # Convert the Python list of points to a NumPy array of the form
    #   | u1 u2 u3 u4 |
    #   | v1 v2 v3 v4 |
    uv = np.array(h_pts).T

    # Specify the points in the global frame, i.e. the frame of the vertebra.
    # Assume the points go (bottom left, top left, top right, bottom right)
    # and that the origin is at the bottom left.
    global_pts = np.ndarray((4, 2))
    # On 2018-12-5, the blue tape on the test setup was 16 squares of 2cm each,
    # so that's 32 cm along each edge.
    edge = 32;
    # the coordinates are then,
    # for points 0 to 4 in the world frame,
    global_pts[0,:] = [0, 0]
    global_pts[1,:] = [0, edge]
    global_pts[2,:] = [edge, edge]
    global_pts[3,:] = [edge, 0]

    # and can now call the function itself.
    # as of 2018-12-5, uv is 2x4 but global_pts is 4x2. STANDARDIZE THIS.
    H = calculate_homography.calc_H(uv, global_pts)
    print("Calculated homography is:")
    print(H)

    # close the current window before proceeding.
    #cv2.destroyWindow("Frame")

    # Testing: 
    # 1) show a grid of points that should correspond to the grid behind the spine
    #calculate_homography.check_homography(H, vs, args, 16, 16, 2)
    # 2) calculate the distance between two points in the local frame.
    calculate_homography.test_H(H, vs, args)

    # Back to the rest of the script.
    print('Press <S> in the "Frame" window to select ROI of first object')

    # loop over frames from the video stream
    while not rospy.is_shutdown():

        # grab the current frame, then handle if we are using a
        # VideoStream or VideoCapture object
        frame = vs.read()
        frame = frame[1] if args.get("video", False) else frame

        # update the FPS counter
        fps.update()
        fps.stop()

        # resize the frame (so we can process it faster) and grab the
        # frame dimensions
        frame = imutils.resize(frame, width=1000)
        (H, W) = frame.shape[:2]

        # initialize the set of information we'll be displaying on
        # the frame
        info = [("FPS", "{:.2f}".format(fps.fps()))
                ]

        # loop over the info tuples and draw them on our frame
        for (i, (k, v)) in enumerate(info):
            text = "{}: {}".format(k, v)
            cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # show the output frame
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the "q" key is pressed, quit the program
        if key == ord("q"):

            # release webcam image
            vs.stop()
            print('[END OF TRACKING]')

            # close all windows and end program
            cv2.destroyAllWindows()
            sys.exit()

        # if the 's' key is selected, we are going to "select" a bounding
        # box to track
        if key == ord("s"):

            # select the bounding box of the objects we want to track (make
            # sure you press ENTER or SPACE after selecting the ROI)
            box = cv2.selectROI("Frame", frame, fromCenter=False,
                                showCrosshair=True)
            tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
            trackers.add(tracker, frame, box)

            print('Press <S> in the "Frame" window to select ROI of second object')

            box = cv2.selectROI("Frame", frame, fromCenter=False,
                                showCrosshair=True)
            tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
            trackers.add(tracker, frame, box)

            # start OpenCV object tracker using the supplied bounding box
            # coordinates, then start the FPS throughput estimator as well
            # tracker.init(frame, initBB[0])
            fps = FPS().start()

            break

    print('ROI selected' + '\n' + 'Press <T> in the "Frame" window to start tracking')
    while not rospy.is_shutdown():

        # grab the current frame, then handle if we are using a
        # VideoStream or VideoCapture object
        frame = vs.read()
        frame = frame[1] if args.get("video", False) else frame
        key = cv2.waitKey(1) & 0xFF

        # if the `s' key is pressed, break from loop
        if key == ord("t"):
            print('[START OF TRACKING]' + '\n' + 'Press <Q> in the "Frame" window to stop tracking')
            break

    return (trackers, args, initBB, pix_com, vs, fps, key)


def tracker_main(trackers, args, initBB, pix_com, vs, fps):

    # grab the current frame, then handle if we are using a
    # VideoStream or VideoCapture object
    frame = vs.read()
    frame = frame[1] if args.get("video", False) else frame

    # resize the frame (so we can process it faster) and grab the
    # frame dimensions
    frame = imutils.resize(frame, width=1000)
    (H, W) = frame.shape[:2]

    # grab the new bounding box coordinates of the object
    (success, boxes) = trackers.update(frame)

    # check to see if the tracking was a success
    if success:
        for ind in range(len(boxes)):
            (x, y, w, h) = [int(v) for v in boxes[ind, :]]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # calculate COM info for tracked object (pixel
            # coordinates, relative to the upper left corner)
            pix_com[ind, :] = [x + w / 2, y + h / 2]

    # update the FPS counter
    fps.update()
    fps.stop()

    # initialize the set of information we'll be displaying on
    # the frame
    info = [
        ("Tracker", args["tracker"]),
        ("Success", "Yes" if success else "No"),
        ("FPS", "{:.2f}".format(fps.fps())),
    ]

    # loop over the info tuples and draw them on our frame
    for (i, (k, v)) in enumerate(info):
        text = "{}: {}".format(k, v)
        cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # show the output frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    return (pix_com, key)


def tracker_angle(com):
    [x, y] = com[0, :] - com[1, :]
    theta_deg = np.array(math.degrees(math.asin(y / x)), dtype=np.float32)

    return theta_deg


if __name__ == "__main__":
    tracker_init()
    tracker_main()
    tracker_angle()

# 1. integrate ROS node - publish to node
# 2. camera rig
# 3. smoothing filter for data
# 4. Homography transform matrix
