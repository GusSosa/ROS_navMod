#!/usr/bin/python2.7

import cv2
import rospy
import sys
import time

# time to wait in between auto-shots
DELAY = 1


def img_cap():

    # start video stream with webcam
    vs = cv2.VideoCapture(1)
    ind = 1

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
