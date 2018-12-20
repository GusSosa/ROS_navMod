;; Auto-generated. Do not edit!


(when (boundp 'spine_controller::InvkinControlCommand)
  (if (not (find-package "SPINE_CONTROLLER"))
    (make-package "SPINE_CONTROLLER"))
  (shadow 'InvkinControlCommand (find-package "SPINE_CONTROLLER")))
(unless (find-package "SPINE_CONTROLLER::INVKINCONTROLCOMMAND")
  (make-package "SPINE_CONTROLLER::INVKINCONTROLCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude InvkinControlCommand.msg.html


(defclass spine_controller::InvkinControlCommand
  :super ros::object
  :slots (_invkin_control _invkin_ref_state ))

(defmethod spine_controller::InvkinControlCommand
  (:init
   (&key
    ((:invkin_control __invkin_control) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:invkin_ref_state __invkin_ref_state) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _invkin_control __invkin_control)
   (setq _invkin_ref_state __invkin_ref_state)
   self)
  (:invkin_control
   (&optional __invkin_control)
   (if __invkin_control (setq _invkin_control __invkin_control)) _invkin_control)
  (:invkin_ref_state
   (&optional __invkin_ref_state)
   (if __invkin_ref_state (setq _invkin_ref_state __invkin_ref_state)) _invkin_ref_state)
  (:serialization-length
   ()
   (+
    ;; float64[] _invkin_control
    (* 8    (length _invkin_control)) 4
    ;; float64[] _invkin_ref_state
    (* 8    (length _invkin_ref_state)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _invkin_control
     (write-long (length _invkin_control) s)
     (dotimes (i (length _invkin_control))
       (sys::poke (elt _invkin_control i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _invkin_ref_state
     (write-long (length _invkin_ref_state) s)
     (dotimes (i (length _invkin_ref_state))
       (sys::poke (elt _invkin_ref_state i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _invkin_control
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _invkin_control (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _invkin_control i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _invkin_ref_state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _invkin_ref_state (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _invkin_ref_state i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get spine_controller::InvkinControlCommand :md5sum-) "ad1887908a0dd9bc527fc66e7b154313")
(setf (get spine_controller::InvkinControlCommand :datatype-) "spine_controller/InvkinControlCommand")
(setf (get spine_controller::InvkinControlCommand :definition-)
      "float64[] invkin_control
float64[] invkin_ref_state
")



(provide :spine_controller/InvkinControlCommand "ad1887908a0dd9bc527fc66e7b154313")


