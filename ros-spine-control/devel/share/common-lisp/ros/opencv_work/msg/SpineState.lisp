; Auto-generated. Do not edit!


(cl:in-package opencv_work-msg)


;//! \htmlinclude SpineState.msg.html

(cl:defclass <SpineState> (roslisp-msg-protocol:ros-message)
  ((rotation
    :reader rotation
    :initarg :rotation
    :type cl:float
    :initform 0.0)
   (comy
    :reader comy
    :initarg :comy
    :type cl:float
    :initform 0.0)
   (comx
    :reader comx
    :initarg :comx
    :type cl:float
    :initform 0.0))
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

(cl:ensure-generic-function 'comy-val :lambda-list '(m))
(cl:defmethod comy-val ((m <SpineState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_work-msg:comy-val is deprecated.  Use opencv_work-msg:comy instead.")
  (comy m))

(cl:ensure-generic-function 'comx-val :lambda-list '(m))
(cl:defmethod comx-val ((m <SpineState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader opencv_work-msg:comx-val is deprecated.  Use opencv_work-msg:comx instead.")
  (comx m))
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
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'comy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'comx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'comy) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'comx) (roslisp-utils:decode-double-float-bits bits)))
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
  "362faa163ba6b21bd1aa0295e7ccf8ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpineState)))
  "Returns md5sum for a message object of type 'SpineState"
  "362faa163ba6b21bd1aa0295e7ccf8ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpineState>)))
  "Returns full string definition for message of type '<SpineState>"
  (cl:format cl:nil "float64 rotation~%float64 comy~%float64 comx~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpineState)))
  "Returns full string definition for message of type 'SpineState"
  (cl:format cl:nil "float64 rotation~%float64 comy~%float64 comx~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpineState>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpineState>))
  "Converts a ROS message object to a list"
  (cl:list 'SpineState
    (cl:cons ':rotation (rotation msg))
    (cl:cons ':comy (comy msg))
    (cl:cons ':comx (comx msg))
))
