# Ubuntu 18.04: How to install OpenCVShell

# $ cd ~
# $ workon cv
# $ python
# Python 3.6.5 (default, Apr 1 2018, 05:46:30)
# [GCC 7.3.0] on linux
# Type "help", "copyright", "credits" or "license" for more information.
# >>> import cv2
# >>> cv2.__version__
# '3.4.1'
# >>> quit()

# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import imutils
import argparse
import time
import cv2
import numpy as np


def tracker_init():

    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()

    # use for video file
    ap.add_argument("-v", "--video", type=str,
                    help="path to input video file")
    ap.add_argument("-t", "--tracker", type=str, default="kcf",
                    help="OpenCV object tracker type")
    args = vars(ap.parse_args())

    # extract the OpenCV version info
    (major, minor) = cv2.__version__.split(".")[:2]

    # # if we are using OpenCV 3.2 OR BEFORE, we can use a special factory
    # # function to create our object tracker
    # if int(major) == 3 and int(minor) < 3:
    #     tracker = cv2.Tracker_create(args["tracker"].upper())

    # otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the
    # approrpiate object tracker constructor:
    # else:
    #     # initialize a dictionary that maps strings to their corresponding
    #     # OpenCV object tracker implementations
    OPENCV_OBJECT_TRACKERS = {
        "csrt": cv2.TrackerCSRT_create,
        "kcf": cv2.TrackerKCF_create,
        "boosting": cv2.TrackerBoosting_create,
        "mil": cv2.TrackerMIL_create,
        "tld": cv2.TrackerTLD_create,
        "medianflow": cv2.TrackerMedianFlow_create,
        "mosse": cv2.TrackerMOSSE_create
    }

    #     # grab the appropriate object tracker using our dictionary of
    #     # OpenCV object tracker objects
    #     tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()

    # initialize the bounding box coordinates of the object we are going
    # to track, and scale object
    trackers = cv2.MultiTracker_create()
    initBB = np.array([], [])
    initScale = None
    x_pix_com = np.zeros((2, 2))
    y_pix_com = [0, 0]

    # if a video path was not supplied, grab the reference to the web cam
    # to use PC webcam, change src=0
    # to use connected USB camera, change src=1 or src=2...
    if not args.get("video", False):
        print("[INFO] starting video stream...")
        vs = VideoStream(src=1).start()
        time.sleep(1.0)

    # otherwise, grab a reference to the video file
    else:
        vs = cv2.VideoCapture(args["video"])

    # initialize the FPS throughput estimator
    fps = FPS().start()

    print('Press <S> in the "Frame" window to select ROI of first object')

    # loop over frames from the video stream
    while True:

        # grab the current frame, then handle if we are using a
        # VideoStream or VideoCapture object
        frame = vs.read()
        frame = frame[1] if args.get("video", False) else frame
        r = 1000 / frame.shape[1]
        dim = (1000, int(frame.shape[0] * r))
        frame = imutils.resize(frame, 3000)
        # frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)

        # # check to see if we have reached the end of the stream
        if frame is None:
            break

        # resize the frame (so we can process it faster) and grab the
        # frame dimensions
        # frame = imutils.resize(frame, width=500)
        (H, W) = frame.shape[:2]

        # update the FPS counter
        fps.update()
        fps.stop()

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

        # if the 't' key is selected, we are going to "select" a bounding
        # box to determine object scale, then get user input scale
        if key == ord("t"):
            # select bounding box of object to scale
            initScale = cv2.selectROI("Frame", frame, fromCenter=False,
                                      showCrosshair=True)

        # if the 's' key is selected, we are going to "select" a bounding
        # box to track
        if key == ord("s"):
            # select the bounding box of the object we want to track (make
            # sure you press ENTER or SPACE after selecting the ROI)

            box = cv2.selectROI("Frame", frame, fromCenter=False,
                                showCrosshair=True)
            tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
            trackers.add(tracker, frame, box)
            # initBB[0] = cv2.selectROI("Frame", frame, fromCenter=False,
            #                           showCrosshair=True)

            print('Press <S> in the "Frame" window to select ROI of second object')

            box = cv2.selectROI("Frame", frame, fromCenter=False,
                                showCrosshair=True)
            tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
            trackers.add(tracker, frame, box)

            # initBB[1] = cv2.selectROI("Frame", frame, fromCenter=False,
            #                           showCrosshair=True)
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

    return (trackers, args, initBB, initScale, x_pix_com, y_pix_com, vs, fps, frame, key)


def tracker_main(trackers, args, initBB, initScale, x_pix_com, y_pix_com, vs, fps):

    # grab the current frame, then handle if we are using a
    # VideoStream or VideoCapture object
    frame = vs.read()
    frame = frame[1] if args.get("video", False) else frame

    # resize the frame (so we can process it faster) and grab the
    # frame dimensions
    # frame = imutils.resize(frame, width=500)
    (H, W) = frame.shape[:2]

    # check to see if we are currently tracking an object
    if initBB is not None:
        # grab the new bounding box coordinates of the object
        (success, boxes) = trackers.update(frame)

        # check to see if the tracking was a success
        if success:
            for ind in range(len(boxes)):
                (x, y, w, h) = [int(v) for v in boxes[ind, :]]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # calculate COM info for tracked object (pixel
                # coordinates, relative to the upper left corner)

                x_pix_com[ind, :] = [x + w / 2, y + h / 2]
                #x_pix_com[ind] = (x + w / 2)
                y_pix_com[ind] = (y + h / 2)

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

    pix_com = np.array([x_pix_com], dtype=np.float32)
    # print pix_com[0, :]
    return (pix_com, key)


if __name__ == "__main__":
    tracker_init()
    tracker_main()


# 1. integrate ROS node - publish to node
# 2. camera rig
# 3. smoothing filter for data
# 4. Homography transform matrix
