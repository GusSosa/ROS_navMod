
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
    vs = VideoStream(src=0).start()
    time.sleep(0.5)

    # initialize the FPS throughput estimator
    fps = FPS().start()

    print('Press <S> in the "Frame" window to select ROI of first object')

    # loop over frames from the video stream
    while True:

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
    while True:

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
