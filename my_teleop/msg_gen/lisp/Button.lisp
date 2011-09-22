; Auto-generated. Do not edit!


(cl:in-package my_teleop-msg)


;//! \htmlinclude Button.msg.html

(cl:defclass <Button> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass Button (<Button>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Button>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Button)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_teleop-msg:<Button> is deprecated: use my_teleop-msg:Button instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Button>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_teleop-msg:type-val is deprecated.  Use my_teleop-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Button>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_teleop-msg:value-val is deprecated.  Use my_teleop-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Button>) ostream)
  "Serializes a message object of type '<Button>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Button>) istream)
  "Deserializes a message object of type '<Button>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Button>)))
  "Returns string type for a message object of type '<Button>"
  "my_teleop/Button")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Button)))
  "Returns string type for a message object of type 'Button"
  "my_teleop/Button")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Button>)))
  "Returns md5sum for a message object of type '<Button>"
  "21c2f05c8d96e487152613b53f09057b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Button)))
  "Returns md5sum for a message object of type 'Button"
  "21c2f05c8d96e487152613b53f09057b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Button>)))
  "Returns full string definition for message of type '<Button>"
  (cl:format cl:nil "int32 type~%int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Button)))
  "Returns full string definition for message of type 'Button"
  (cl:format cl:nil "int32 type~%int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Button>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Button>))
  "Converts a ROS message object to a list"
  (cl:list 'Button
    (cl:cons ':type (type msg))
    (cl:cons ':value (value msg))
))
