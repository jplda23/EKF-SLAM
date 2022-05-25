; Auto-generated. Do not edit!


(cl:in-package monarch_msgs-msg)


;//! \htmlinclude HeadControlSemantic.msg.html

(cl:defclass <HeadControlSemantic> (roslisp-msg-protocol:ros-message)
  ((cardinal_direction
    :reader cardinal_direction
    :initarg :cardinal_direction
    :type cl:string
    :initform "")
   (speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass HeadControlSemantic (<HeadControlSemantic>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HeadControlSemantic>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HeadControlSemantic)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name monarch_msgs-msg:<HeadControlSemantic> is deprecated: use monarch_msgs-msg:HeadControlSemantic instead.")))

(cl:ensure-generic-function 'cardinal_direction-val :lambda-list '(m))
(cl:defmethod cardinal_direction-val ((m <HeadControlSemantic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monarch_msgs-msg:cardinal_direction-val is deprecated.  Use monarch_msgs-msg:cardinal_direction instead.")
  (cardinal_direction m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <HeadControlSemantic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monarch_msgs-msg:speed-val is deprecated.  Use monarch_msgs-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<HeadControlSemantic>)))
    "Constants for message type '<HeadControlSemantic>"
  '((:NORMAL . 0)
    (:SLOW . 1)
    (:FAST . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'HeadControlSemantic)))
    "Constants for message type 'HeadControlSemantic"
  '((:NORMAL . 0)
    (:SLOW . 1)
    (:FAST . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HeadControlSemantic>) ostream)
  "Serializes a message object of type '<HeadControlSemantic>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cardinal_direction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cardinal_direction))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HeadControlSemantic>) istream)
  "Deserializes a message object of type '<HeadControlSemantic>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cardinal_direction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cardinal_direction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HeadControlSemantic>)))
  "Returns string type for a message object of type '<HeadControlSemantic>"
  "monarch_msgs/HeadControlSemantic")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HeadControlSemantic)))
  "Returns string type for a message object of type 'HeadControlSemantic"
  "monarch_msgs/HeadControlSemantic")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HeadControlSemantic>)))
  "Returns md5sum for a message object of type '<HeadControlSemantic>"
  "a8f7a062da2d04ec1d87e1a59f64f6af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HeadControlSemantic)))
  "Returns md5sum for a message object of type 'HeadControlSemantic"
  "a8f7a062da2d04ec1d87e1a59f64f6af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HeadControlSemantic>)))
  "Returns full string definition for message of type '<HeadControlSemantic>"
  (cl:format cl:nil "# If you add more, please do not break the current definitions~%uint8 NORMAL=0~%uint8 SLOW=1~%uint8 FAST=2~%~%# Example for NW, accepts:~%# {NW, NORTHEAST}~%string cardinal_direction~%uint8 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HeadControlSemantic)))
  "Returns full string definition for message of type 'HeadControlSemantic"
  (cl:format cl:nil "# If you add more, please do not break the current definitions~%uint8 NORMAL=0~%uint8 SLOW=1~%uint8 FAST=2~%~%# Example for NW, accepts:~%# {NW, NORTHEAST}~%string cardinal_direction~%uint8 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HeadControlSemantic>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'cardinal_direction))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HeadControlSemantic>))
  "Converts a ROS message object to a list"
  (cl:list 'HeadControlSemantic
    (cl:cons ':cardinal_direction (cardinal_direction msg))
    (cl:cons ':speed (speed msg))
))
