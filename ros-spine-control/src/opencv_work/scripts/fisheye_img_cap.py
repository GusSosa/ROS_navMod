#!/usr/bin/python2.7

# fisheye_img_cap.py
# Program used in conjunction with fisheye.py. This program captures images from the specified camera,
# used for calibration and correction of the fisheye distortion for that camera.

# This program can be used in two modes:
# 1) Single Image Capture: Press <c> key to capture an individual image
# 2) Continuous Image Capture: Press <s> key and the program will capture one image / sec until stopped (<q>)

# NOTE 1: Images captured will be saved in user's current command line location, NOT the program folder
# NOTE 2: By default (ie no extra sys.argv), the program will index images starting with 1,2,...
#   The user can specify a starting index by passing an additional argument to the command line
#   ie "rosrun opencv_work fisheye_img_cap.py 24" would start indexing at 24, so the output images
#   would be titled "frame24.jpg", "frame25.jpg",... etc

import cv2
import rospy
import sys
import time

# time to wait in between auto-shots
DELAY = 1


def img_cap():

    # start video stream with webcam
    vs = cv2.VideoCapture(1)
    ind = 1 if not sys.argv[1] else int(sys.argv[1])

    # set reolution
    print "Frame default resolution: (" + str(vs.get(cv2.CAP_PROP_FRAME_WIDTH)) + "; " + str(vs.get(cv2.CAP_PROP_FRAME_HEIGHT)) + ")"
    vs.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    print "Frame resolution set to: (" + str(vs.get(cv2.CAP_PROP_FRAME_WIDTH)) + "; " + str(vs.get(cv2.CAP_PROP_FRAME_HEIGHT)) + ")"
    print "Frame FPS: " + str(vs.get(cv2.CAP_PROP_FPS))

    print ('Press <c> in the "frame" to capture a single image.' + '\n' +
           'Press <s> in "frame" to start Continuous Shooting (ie one image captured every second).' + '\n' +
           'Press <q> to exit program')

    while not rospy.is_shutdown():

        # current frame
        frame = vs.read()[1]
        cv2.imshow('frame', frame)

        # reset keyboard interrupt
        key = cv2.waitKey(1) & 0xFF

        # if the 'c' key is pressed, capture current frame
        if key == ord("c"):

                        # image name
            name = 'frame%d.jpg' % ind
            cv2.imwrite(name, frame)
            print('Image No. ' + str(ind) + ' Captured')
            ind = ind + 1

        # if the 's' key is pressed, take one image every 1 second
        if key == ord('s'):
            print('Continuous Shooting Activated')

            t = time.time()

            while not rospy.is_shutdown():

                # current frame
                frame = vs.read()[1]
                cv2.imshow('frame', frame)
                # reset keyboard interrupt
                key = cv2.waitKey(1) & 0xFF

                # if the 'q' key is pressed, end program
                if key == ord("q"):

                    # close all windows and end program
                    vs.release()
                    cv2.destroyAllWindows()
                    print('[PROGRAM END]')
                    sys.exit()

                if time.time() - t > DELAY:

                    # image name
                    name = 'frame%d.jpg' % ind
                    cv2.imwrite(name, frame)
                    print('Image No. ' + str(ind) + ' Captured')
                    ind = ind + 1
                    t = time.time()

        # if the 'q' key is pressed, end programe
        if key == ord("q"):

            # close all windows and end program
            vs.release()
            cv2.destroyAllWindows()
            print('[PROGRAM END]')
            sys.exit()


if __name__ == '__main__':
    try:
        img_cap()
    except rospy.ROSInterruptException:
        pass
