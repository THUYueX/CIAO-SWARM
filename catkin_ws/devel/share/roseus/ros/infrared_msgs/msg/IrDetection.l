;; Auto-generated. Do not edit!


(when (boundp 'infrared_msgs::IrDetection)
  (if (not (find-package "INFRARED_MSGS"))
    (make-package "INFRARED_MSGS"))
  (shadow 'IrDetection (find-package "INFRARED_MSGS")))
(unless (find-package "INFRARED_MSGS::IRDETECTION")
  (make-package "INFRARED_MSGS::IRDETECTION"))

(in-package "ROS")
;;//! \htmlinclude IrDetection.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass infrared_msgs::IrDetection
  :super ros::object
  :slots (_header _emitter_id _distance _strength _direction _signal_to_noise _line_of_sight ))

(defmethod infrared_msgs::IrDetection
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:emitter_id __emitter_id) "")
    ((:distance __distance) 0.0)
    ((:strength __strength) 0.0)
    ((:direction __direction) (instance geometry_msgs::Vector3 :init))
    ((:signal_to_noise __signal_to_noise) 0.0)
    ((:line_of_sight __line_of_sight) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _emitter_id (string __emitter_id))
   (setq _distance (float __distance))
   (setq _strength (float __strength))
   (setq _direction __direction)
   (setq _signal_to_noise (float __signal_to_noise))
   (setq _line_of_sight __line_of_sight)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:emitter_id
   (&optional __emitter_id)
   (if __emitter_id (setq _emitter_id __emitter_id)) _emitter_id)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:strength
   (&optional __strength)
   (if __strength (setq _strength __strength)) _strength)
  (:direction
   (&rest __direction)
   (if (keywordp (car __direction))
       (send* _direction __direction)
     (progn
       (if __direction (setq _direction (car __direction)))
       _direction)))
  (:signal_to_noise
   (&optional __signal_to_noise)
   (if __signal_to_noise (setq _signal_to_noise __signal_to_noise)) _signal_to_noise)
  (:line_of_sight
   (&optional __line_of_sight)
   (if __line_of_sight (setq _line_of_sight __line_of_sight)) _line_of_sight)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _emitter_id
    4 (length _emitter_id)
    ;; float64 _distance
    8
    ;; float64 _strength
    8
    ;; geometry_msgs/Vector3 _direction
    (send _direction :serialization-length)
    ;; float64 _signal_to_noise
    8
    ;; bool _line_of_sight
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _emitter_id
       (write-long (length _emitter_id) s) (princ _emitter_id s)
     ;; float64 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _strength
       (sys::poke _strength (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Vector3 _direction
       (send _direction :serialize s)
     ;; float64 _signal_to_noise
       (sys::poke _signal_to_noise (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _line_of_sight
       (if _line_of_sight (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _emitter_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _emitter_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _distance
     (setq _distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _strength
     (setq _strength (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Vector3 _direction
     (send _direction :deserialize buf ptr-) (incf ptr- (send _direction :serialization-length))
   ;; float64 _signal_to_noise
     (setq _signal_to_noise (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _line_of_sight
     (setq _line_of_sight (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get infrared_msgs::IrDetection :md5sum-) "d8765f5c28a65d3ab4e34a4adb77d9b1")
(setf (get infrared_msgs::IrDetection :datatype-) "infrared_msgs/IrDetection")
(setf (get infrared_msgs::IrDetection :definition-)
      "Header header
string emitter_id
float64 distance
float64 strength
geometry_msgs/Vector3 direction
float64 signal_to_noise
bool line_of_sight

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :infrared_msgs/IrDetection "d8765f5c28a65d3ab4e34a4adb77d9b1")


