; Auto-generated. Do not edit!


(cl:in-package spine_controller-msg)


;//! \htmlinclude InvkinControlCommand.msg.html

(cl:defclass <InvkinControlCommand> (roslisp-msg-protocol:ros-message)
  ((invkin_control
    :reader invkin_control
    :initarg :invkin_control
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (invkin_ref_state
    :reader invkin_ref_state
    :initarg :invkin_ref_state
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass InvkinControlCommand (<InvkinControlCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InvkinControlCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InvkinControlCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spine_controller-msg:<InvkinControlCommand> is deprecated: use spine_controller-msg:InvkinControlCommand instead.")))

(cl:ensure-generic-function 'invkin_control-val :lambda-list '(m))
(cl:defmethod invkin_control-val ((m <InvkinControlCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spine_controller-msg:invkin_control-val is deprecated.  Use spine_controller-msg:invkin_control instead.")
  (invkin_control m))

(cl:ensure-generic-function 'invkin_ref_state-val :lambda-list '(m))
(cl:defmethod invkin_ref_state-val ((m <InvkinControlCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spine_controller-msg:invkin_ref_state-val is deprecated.  Use spine_controller-msg:invkin_ref_state instead.")
  (invkin_ref_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InvkinControlCommand>) ostream)
  "Serializes a message object of type '<InvkinControlCommand>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'invkin_control))))
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
   (cl:slot-value msg 'invkin_control))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'invkin_ref_state))))
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
   (cl:slot-value msg 'invkin_ref_state))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InvkinControlCommand>) istream)
  "Deserializes a message object of type '<InvkinControlCommand>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'invkin_control) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'invkin_control)))
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
  (cl:setf (cl:slot-value msg 'invkin_ref_state) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'invkin_ref_state)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InvkinControlCommand>)))
  "Returns string type for a message object of type '<InvkinControlCommand>"
  "spine_controller/InvkinControlCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InvkinControlCommand)))
  "Returns string type for a message object of type 'InvkinControlCommand"
  "spine_controller/InvkinControlCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InvkinControlCommand>)))
  "Returns md5sum for a message object of type '<InvkinControlCommand>"
  "ad1887908a0dd9bc527fc66e7b154313")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InvkinControlCommand)))
  "Returns md5sum for a message object of type 'InvkinControlCommand"
  "ad1887908a0dd9bc527fc66e7b154313")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InvkinControlCommand>)))
  "Returns full string definition for message of type '<InvkinControlCommand>"
  (cl:format cl:nil "float64[] invkin_control~%float64[] invkin_ref_state~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InvkinControlCommand)))
  "Returns full string definition for message of type 'InvkinControlCommand"
  (cl:format cl:nil "float64[] invkin_control~%float64[] invkin_ref_state~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InvkinControlCommand>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'invkin_control) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'invkin_ref_state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InvkinControlCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'InvkinControlCommand
    (cl:cons ':invkin_control (invkin_control msg))
    (cl:cons ':invkin_ref_state (invkin_ref_state msg))
))
