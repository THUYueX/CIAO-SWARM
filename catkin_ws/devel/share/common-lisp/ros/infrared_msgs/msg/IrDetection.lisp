; Auto-generated. Do not edit!


(cl:in-package infrared_msgs-msg)


;//! \htmlinclude IrDetection.msg.html

(cl:defclass <IrDetection> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (emitter_id
    :reader emitter_id
    :initarg :emitter_id
    :type cl:string
    :initform "")
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (strength
    :reader strength
    :initarg :strength
    :type cl:float
    :initform 0.0)
   (direction
    :reader direction
    :initarg :direction
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (signal_to_noise
    :reader signal_to_noise
    :initarg :signal_to_noise
    :type cl:float
    :initform 0.0)
   (line_of_sight
    :reader line_of_sight
    :initarg :line_of_sight
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass IrDetection (<IrDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IrDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IrDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name infrared_msgs-msg:<IrDetection> is deprecated: use infrared_msgs-msg:IrDetection instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <IrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader infrared_msgs-msg:header-val is deprecated.  Use infrared_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'emitter_id-val :lambda-list '(m))
(cl:defmethod emitter_id-val ((m <IrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader infrared_msgs-msg:emitter_id-val is deprecated.  Use infrared_msgs-msg:emitter_id instead.")
  (emitter_id m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <IrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader infrared_msgs-msg:distance-val is deprecated.  Use infrared_msgs-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'strength-val :lambda-list '(m))
(cl:defmethod strength-val ((m <IrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader infrared_msgs-msg:strength-val is deprecated.  Use infrared_msgs-msg:strength instead.")
  (strength m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <IrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader infrared_msgs-msg:direction-val is deprecated.  Use infrared_msgs-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'signal_to_noise-val :lambda-list '(m))
(cl:defmethod signal_to_noise-val ((m <IrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader infrared_msgs-msg:signal_to_noise-val is deprecated.  Use infrared_msgs-msg:signal_to_noise instead.")
  (signal_to_noise m))

(cl:ensure-generic-function 'line_of_sight-val :lambda-list '(m))
(cl:defmethod line_of_sight-val ((m <IrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader infrared_msgs-msg:line_of_sight-val is deprecated.  Use infrared_msgs-msg:line_of_sight instead.")
  (line_of_sight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IrDetection>) ostream)
  "Serializes a message object of type '<IrDetection>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'emitter_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'emitter_id))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'strength))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'direction) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'signal_to_noise))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'line_of_sight) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IrDetection>) istream)
  "Deserializes a message object of type '<IrDetection>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'emitter_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'emitter_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'strength) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'direction) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'signal_to_noise) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'line_of_sight) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IrDetection>)))
  "Returns string type for a message object of type '<IrDetection>"
  "infrared_msgs/IrDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IrDetection)))
  "Returns string type for a message object of type 'IrDetection"
  "infrared_msgs/IrDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IrDetection>)))
  "Returns md5sum for a message object of type '<IrDetection>"
  "d8765f5c28a65d3ab4e34a4adb77d9b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IrDetection)))
  "Returns md5sum for a message object of type 'IrDetection"
  "d8765f5c28a65d3ab4e34a4adb77d9b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IrDetection>)))
  "Returns full string definition for message of type '<IrDetection>"
  (cl:format cl:nil "Header header~%string emitter_id~%float64 distance~%float64 strength~%geometry_msgs/Vector3 direction~%float64 signal_to_noise~%bool line_of_sight~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IrDetection)))
  "Returns full string definition for message of type 'IrDetection"
  (cl:format cl:nil "Header header~%string emitter_id~%float64 distance~%float64 strength~%geometry_msgs/Vector3 direction~%float64 signal_to_noise~%bool line_of_sight~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IrDetection>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'emitter_id))
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'direction))
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IrDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'IrDetection
    (cl:cons ':header (header msg))
    (cl:cons ':emitter_id (emitter_id msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':strength (strength msg))
    (cl:cons ':direction (direction msg))
    (cl:cons ':signal_to_noise (signal_to_noise msg))
    (cl:cons ':line_of_sight (line_of_sight msg))
))
