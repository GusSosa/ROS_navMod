;; Auto-generated. Do not edit!


(when (boundp 'opencv_work::SpineState)
  (if (not (find-package "OPENCV_WORK"))
    (make-package "OPENCV_WORK"))
  (shadow 'SpineState (find-package "OPENCV_WORK")))
(unless (find-package "OPENCV_WORK::SPINESTATE")
  (make-package "OPENCV_WORK::SPINESTATE"))

(in-package "ROS")
;;//! \htmlinclude SpineState.msg.html


(defclass opencv_work::SpineState
  :super ros::object
  :slots (_rotation _comy _comx ))

(defmethod opencv_work::SpineState
  (:init
   (&key
    ((:rotation __rotation) 0.0)
    ((:comy __comy) 0.0)
    ((:comx __comx) 0.0)
    )
   (send-super :init)
   (setq _rotation (float __rotation))
   (setq _comy (float __comy))
   (setq _comx (float __comx))
   self)
  (:rotation
   (&optional __rotation)
   (if __rotation (setq _rotation __rotation)) _rotation)
  (:comy
   (&optional __comy)
   (if __comy (setq _comy __comy)) _comy)
  (:comx
   (&optional __comx)
   (if __comx (setq _comx __comx)) _comx)
  (:serialization-length
   ()
   (+
    ;; float64 _rotation
    8
    ;; float64 _comy
    8
    ;; float64 _comx
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _rotation
       (sys::poke _rotation (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _comy
       (sys::poke _comy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _comx
       (sys::poke _comx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _rotation
     (setq _rotation (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _comy
     (setq _comy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _comx
     (setq _comx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get opencv_work::SpineState :md5sum-) "362faa163ba6b21bd1aa0295e7ccf8ab")
(setf (get opencv_work::SpineState :datatype-) "opencv_work/SpineState")
(setf (get opencv_work::SpineState :definition-)
      "float64 rotation
float64 comy
float64 comx

")



(provide :opencv_work/SpineState "362faa163ba6b21bd1aa0295e7ccf8ab")


