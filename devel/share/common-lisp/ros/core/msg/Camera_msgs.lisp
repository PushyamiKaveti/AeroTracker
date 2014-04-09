; Auto-generated. Do not edit!


(cl:in-package core-msg)


;//! \htmlinclude Camera_msgs.msg.html

(cl:defclass <Camera_msgs> (roslisp-msg-protocol:ros-message)
  ((center_x
    :reader center_x
    :initarg :center_x
    :type cl:integer
    :initform 0)
   (center_y
    :reader center_y
    :initarg :center_y
    :type cl:integer
    :initform 0)
   (centroid_x
    :reader centroid_x
    :initarg :centroid_x
    :type cl:integer
    :initform 0)
   (centroid_y
    :reader centroid_y
    :initarg :centroid_y
    :type cl:integer
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:real
    :initform 0))
)

(cl:defclass Camera_msgs (<Camera_msgs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Camera_msgs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Camera_msgs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name core-msg:<Camera_msgs> is deprecated: use core-msg:Camera_msgs instead.")))

(cl:ensure-generic-function 'center_x-val :lambda-list '(m))
(cl:defmethod center_x-val ((m <Camera_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader core-msg:center_x-val is deprecated.  Use core-msg:center_x instead.")
  (center_x m))

(cl:ensure-generic-function 'center_y-val :lambda-list '(m))
(cl:defmethod center_y-val ((m <Camera_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader core-msg:center_y-val is deprecated.  Use core-msg:center_y instead.")
  (center_y m))

(cl:ensure-generic-function 'centroid_x-val :lambda-list '(m))
(cl:defmethod centroid_x-val ((m <Camera_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader core-msg:centroid_x-val is deprecated.  Use core-msg:centroid_x instead.")
  (centroid_x m))

(cl:ensure-generic-function 'centroid_y-val :lambda-list '(m))
(cl:defmethod centroid_y-val ((m <Camera_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader core-msg:centroid_y-val is deprecated.  Use core-msg:centroid_y instead.")
  (centroid_y m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <Camera_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader core-msg:time-val is deprecated.  Use core-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Camera_msgs>) ostream)
  "Serializes a message object of type '<Camera_msgs>"
  (cl:let* ((signed (cl:slot-value msg 'center_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'center_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'centroid_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'centroid_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time) (cl:floor (cl:slot-value msg 'time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Camera_msgs>) istream)
  "Deserializes a message object of type '<Camera_msgs>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'center_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'center_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'centroid_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'centroid_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Camera_msgs>)))
  "Returns string type for a message object of type '<Camera_msgs>"
  "core/Camera_msgs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Camera_msgs)))
  "Returns string type for a message object of type 'Camera_msgs"
  "core/Camera_msgs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Camera_msgs>)))
  "Returns md5sum for a message object of type '<Camera_msgs>"
  "8e94230b3f39704075111718d57a6c87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Camera_msgs)))
  "Returns md5sum for a message object of type 'Camera_msgs"
  "8e94230b3f39704075111718d57a6c87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Camera_msgs>)))
  "Returns full string definition for message of type '<Camera_msgs>"
  (cl:format cl:nil "int32 center_x~%int32 center_y~%int32 centroid_x~%int32 centroid_y~%time time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Camera_msgs)))
  "Returns full string definition for message of type 'Camera_msgs"
  (cl:format cl:nil "int32 center_x~%int32 center_y~%int32 centroid_x~%int32 centroid_y~%time time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Camera_msgs>))
  (cl:+ 0
     4
     4
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Camera_msgs>))
  "Converts a ROS message object to a list"
  (cl:list 'Camera_msgs
    (cl:cons ':center_x (center_x msg))
    (cl:cons ':center_y (center_y msg))
    (cl:cons ':centroid_x (centroid_x msg))
    (cl:cons ':centroid_y (centroid_y msg))
    (cl:cons ':time (time msg))
))
