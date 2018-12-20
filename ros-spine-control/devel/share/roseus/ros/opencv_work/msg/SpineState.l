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
  :slots (_rotation _com1 _com2 ))

(defmethod opencv_work::SpineState
  (:init
   (&key
    ((:rotation __rotation) 0.0)
    ((:com1 __com1) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:com2 __com2) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _rotation (float __rotation))
   (setq _com1 __com1)
   (setq _com2 __com2)
   self)
  (:rotation
   (&optional __rotation)
   (if __rotation (setq _rotation __rotation)) _rotation)
  (:com1
   (&optional __com1)
   (if __com1 (setq _com1 __com1)) _com1)
  (:com2
   (&optional __com2)
   (if __com2 (setq _com2 __com2)) _com2)
  (:serialization-length
   ()
   (+
    ;; float64 _rotation
    8
    ;; float64[] _com1
    (* 8    (length _com1)) 4
    ;; float64[] _com2
    (* 8    (length _com2)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _rotation
       (sys::poke _rotation (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _com1
     (write-long (length _com1) s)
     (dotimes (i (length _com1))
       (sys::poke (elt _com1 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _com2
     (write-long (length _com2) s)
     (dotimes (i (length _com2))
       (sys::poke (elt _com2 i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _rotation
     (setq _rotation (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _com1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _com1 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _com1 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _com2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _com2 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _com2 i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get opencv_work::SpineState :md5sum-) "0c4ed841ebd6c1ed336e4b4bb0dbbca7")
(setf (get opencv_work::SpineState :datatype-) "opencv_work/SpineState")
(setf (get opencv_work::SpineState :definition-)
      "float64 rotation
float64[] com1
float64[] com2
")



(provide :opencv_work/SpineState "0c4ed841ebd6c1ed336e4b4bb0dbbca7")


