; Auto-generated. Do not edit!


(cl:in-package my_teleop-msg)


;//! \htmlinclude Axis.msg.html

(cl:defclass <Axis> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass Axis (<Axis>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Axis>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Axis)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_teleop-msg:<Axis> is deprecated: use my_teleop-msg:Axis instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Axis>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_teleop-msg:type-val is deprecated.  Use my_teleop-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Axis>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_teleop-msg:value-val is deprecated.  Use my_teleop-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Axis>) ostream)
  "Serializes a message object of type '<Axis>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Axis>) istream)
  "Deserializes a message object of type '<Axis>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Axis>)))
  "Returns string type for a message object of type '<Axis>"
  "my_teleop/Axis")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Axis)))
  "Returns string type for a message object of type 'Axis"
  "my_teleop/Axis")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Axis>)))
  "Returns md5sum for a message object of type '<Axis>"
  "ff848a94bb7e24a6adffc2343d23e191")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Axis)))
  "Returns md5sum for a message object of type 'Axis"
  "ff848a94bb7e24a6adffc2343d23e191")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Axis>)))
  "Returns full string definition for message of type '<Axis>"
  (cl:format cl:nil "int32 type~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Axis)))
  "Returns full string definition for message of type 'Axis"
  (cl:format cl:nil "int32 type~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Axis>))
  (cl:+ 0
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Axis>))
  "Converts a ROS message object to a list"
  (cl:list 'Axis
    (cl:cons ':type (type msg))
    (cl:cons ':value (value msg))
))
