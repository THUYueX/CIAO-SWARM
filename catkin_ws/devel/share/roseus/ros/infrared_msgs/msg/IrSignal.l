;; Auto-generated. Do not edit!


(when (boundp 'infrared_msgs::IrSignal)
  (if (not (find-package "INFRARED_MSGS"))
    (make-package "INFRARED_MSGS"))
  (shadow 'IrSignal (find-package "INFRARED_MSGS")))
(unless (find-package "INFRARED_MSGS::IRSIGNAL")
  (make-package "INFRARED_MSGS::IRSIGNAL"))

(in-package "ROS")
;;//! \htmlinclude IrSignal.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass infrared_msgs::IrSignal
  :super ros::object
  :slots (_header _emitter_id _position _direction _range _strength _frequency _modulation _wavelength ))

(defmethod infrared_msgs::IrSignal
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:emitter_id __emitter_id) "")
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:direction __direction) (instance geometry_msgs::Vector3 :init))
    ((:range __range) 0.0)
    ((:strength __strength) 0.0)
    ((:frequency __frequency) 0.0)
    ((:modulation __modulation) "")
    ((:wavelength __wavelength) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _emitter_id (string __emitter_id))
   (setq _position __position)
   (setq _direction __direction)
   (setq _range (float __range))
   (setq _strength (float __strength))
   (setq _frequency (float __frequency))
   (setq _modulation (string __modulation))
   (setq _wavelength (float __wavelength))
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
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:direction
   (&rest __direction)
   (if (keywordp (car __direction))
       (send* _direction __direction)
     (progn
       (if __direction (setq _direction (car __direction)))
       _direction)))
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:strength
   (&optional __strength)
   (if __strength (setq _strength __strength)) _strength)
  (:frequency
   (&optional __frequency)
   (if __frequency (setq _frequency __frequency)) _frequency)
  (:modulation
   (&optional __modulation)
   (if __modulation (setq _modulation __modulation)) _modulation)
  (:wavelength
   (&optional __wavelength)
   (if __wavelength (setq _wavelength __wavelength)) _wavelength)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _emitter_id
    4 (length _emitter_id)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _direction
    (send _direction :serialization-length)
    ;; float64 _range
    8
    ;; float64 _strength
    8
    ;; float64 _frequency
    8
    ;; string _modulation
    4 (length _modulation)
    ;; float64 _wavelength
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _emitter_id
       (write-long (length _emitter_id) s) (princ _emitter_id s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _direction
       (send _direction :serialize s)
     ;; float64 _range
       (sys::poke _range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _strength
       (sys::poke _strength (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _frequency
       (sys::poke _frequency (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _modulation
       (write-long (length _modulation) s) (princ _modulation s)
     ;; float64 _wavelength
       (sys::poke _wavelength (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _emitter_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _emitter_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _direction
     (send _direction :deserialize buf ptr-) (incf ptr- (send _direction :serialization-length))
   ;; float64 _range
     (setq _range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _strength
     (setq _strength (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _frequency
     (setq _frequency (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _modulation
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _modulation (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _wavelength
     (setq _wavelength (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get infrared_msgs::IrSignal :md5sum-) "2b33114226fe9c26f156b572ed403903")
(setf (get infrared_msgs::IrSignal :datatype-) "infrared_msgs/IrSignal")
(setf (get infrared_msgs::IrSignal :definition-)
      "Header header
string emitter_id
geometry_msgs/Point position
geometry_msgs/Vector3 direction
float64 range
float64 strength
float64 frequency
string modulation
float64 wavelength

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

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



(provide :infrared_msgs/IrSignal "2b33114226fe9c26f156b572ed403903")


