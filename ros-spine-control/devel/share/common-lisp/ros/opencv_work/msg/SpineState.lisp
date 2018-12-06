; Auto-generated. Do not edit!


(cl:in-package opencv_work-msg)


;//! \htmlinclude SpineState.msg.html

(cl:defclass <SpineState> (roslisp-msg-protocol:ros-message)
  ((rotation
    :reader rotation
    :initarg :rotation
    :type cl:float
    :initform 0.0)
   (com1
    :reader com1
    :initarg :com1
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (com2
    :reader com2
    :initarg :com2
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SpineState (<SpineState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpineState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpineState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name opencv_work-msg:<SpineState> is deprecated: use opencv_work-msg:SpineState instead.")))

(cl:ensure-generic-function 'rotation-val :lambda-list '(m))
(cl:defmethod rotation-val ((m <SpineState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_work-msg:rotation-val is deprecated.  Use opencv_work-msg:rotation instead.")
  (rotation m))

(cl:ensure-generic-function 'com1-val :lambda-list '(m))
(cl:defmethod com1-val ((m <SpineState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_work-msg:com1-val is deprecated.  Use opencv_work-msg:com1 instead.")
  (com1 m))

(cl:ensure-generic-function 'com2-val :lambda-list '(m))
(cl:defmethod com2-val ((m <SpineState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_work-msg:com2-val is deprecated.  Use opencv_work-msg:com2 instead.")
  (com2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpineState>) ostream)
  "Serializes a message object of type '<SpineState>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rotation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'com1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'com1))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'com2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'com2))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpineState>) istream)
  "Deserializes a message object of type '<SpineState>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotation) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'com1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'com1)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'com2) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'com2)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpineState>)))
  "Returns string type for a message object of type '<SpineState>"
  "opencv_work/SpineState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpineState)))
  "Returns string type for a message object of type 'SpineState"
  "opencv_work/SpineState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpineState>)))
  "Returns md5sum for a message object of type '<SpineState>"
  "0c4ed841ebd6c1ed336e4b4bb0dbbca7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpineState)))
  "Returns md5sum for a message object of type 'SpineState"
  "0c4ed841ebd6c1ed336e4b4bb0dbbca7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpineState>)))
  "Returns full string definition for message of type '<SpineState>"
  (cl:format cl:nil "float64 rotation~%float64[] com1~%float64[] com2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpineState)))
  "Returns full string definition for message of type 'SpineState"
  (cl:format cl:nil "float64 rotation~%float64[] com1~%float64[] com2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpineState>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'com1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'com2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpineState>))
  "Converts a ROS message object to a list"
  (cl:list 'SpineState
    (cl:cons ':rotation (rotation msg))
    (cl:cons ':com1 (com1 msg))
    (cl:cons ':com2 (com2 msg))
))
