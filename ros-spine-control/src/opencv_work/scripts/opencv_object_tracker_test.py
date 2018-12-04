#!/usr/bin/python

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
import argparse
import imutils
import time
import cv2
import sys

# main funtion call

def main(self):
    return Initialization()
    track_check = 'y'

    # if a video path was not supplied, start tracking immediately
    while track_check == 'y' or track_check == 'Y':
        main_tracking(ap, args, self)


class initialization:
    def init(self):

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

        # if we are using OpenCV 3.2 OR BEFORE, we can use a special factory
        # function to create our object tracker
        if int(major) == 3 and int(minor) < 3:
            tracker = cv2.Tracker_create(args["tracker"].upper())

        # otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the
        # approrpiate object tracker constructor:
        else:
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

            # grab the appropriate object tracker using our dictionary of
            # OpenCV object tracker objects
            self.tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()

            # initialize the bounding box coordinates of the object we are going
            # to track, and scale object
            self.initBB = None
            self.initScale = None

        # if a video path was not supplied, grab the reference to the web cam
        if not args.get("video", False):
            print('\n' + "[INFO] starting video stream...")
            self.vs = VideoStream(src=0).start()
            time.sleep(1.0)

        # otherwise, grab a reference to the video file
        else:
            selfvs = cv2.VideoCapture(args["video"])

        # initialize the FPS throughput estimator
        self.fps = None
        self.com_output = input('Output COM info? <Y/N>')


def main_tracking(ap, args, self):

    print('\n' + 'Press <S> in the "Frame" window to select ROI')

    # loop over frames from the video stream
    while True:
        # grab the current frame, then handle if we are using a
        # VideoStream or VideoCapture object
        frame = vs.read()
        frame = frame[1] if args.get("video", False) else frame

        # check to see if we have reached the end of the stream
        if frame is None:
            break

        # resize the frame (so we can process it faster) and grab the
        # frame dimensions
        # frame = imutils.resize(frame, width=500)
        (H, W) = frame.shape[:2]

        # check to see if we are currently tracking an object
        if initBB is not None:
            # grab the new bounding box coordinates of the object
            (success, box) = tracker.update(frame)

            # check to see if the tracking was a success
            if success:
                (x, y, w, h) = [int(v) for v in box]
                cv2.rectangle(frame, (x, y), (x + w, y + h),
                              (0, 255, 0), 2)

                # check to see if user wants the center of mass info displayed
                if (com_output == 'Y') or (com_output == 'y'):

                    # calculate and print COM info for tracked object (pixel
                    # coordinates, relative to the upper left corner)
                    x_pix_com = x + w / 2
                    y_pix_com = y + h / 2

                    print('\n' + 'X coordinate COM = ' + str(x_pix_com))
                    print('Y coordinate COM = ' + str(y_pix_com))

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

        # if the 't' key is selected, we are going to "select" a bounding
        # box to determine object scale, then get user input scale
        if key == ord('t'):
            # select bounding box of object to scale
            initScale = cv2.selectROI("Frame", frame, fromCenter=False,
                                      showCrosshair=True)

        # if the 's' key is selected, we are going to "select" a bounding
        # box to track
        if key == ord("s"):
            # select the bounding box of the object we want to track (make
            # sure you press ENTER or SPACE after selecting the ROI)
            initBB = cv2.selectROI("Frame", frame, fromCenter=False,
                                   showCrosshair=True)

            # start OpenCV object tracker using the supplied bounding box
            # coordinates, then start the FPS throughput estimator as well
            tracker.init(frame, initBB)
            fps = FPS().start()

        # if the `q` key was pressed, break from the loop
        elif key == ord("q"):
            break

    # if we are using a webcam, release the pointer
    if not args.get("video", False):
        vs.stop()

    # otherwise, release the file pointer
    else:
        vs.release()

    # close all windows
    cv2.destroyAllWindows()

    track_check = input('Start tracking again?')


if __name__ == '__main__':

    main(self)
