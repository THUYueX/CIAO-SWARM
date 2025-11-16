// Auto-generated. Do not edit!

// (in-package infrared_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class IrDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.emitter_id = null;
      this.distance = null;
      this.strength = null;
      this.direction = null;
      this.signal_to_noise = null;
      this.line_of_sight = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('emitter_id')) {
        this.emitter_id = initObj.emitter_id
      }
      else {
        this.emitter_id = '';
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
      if (initObj.hasOwnProperty('strength')) {
        this.strength = initObj.strength
      }
      else {
        this.strength = 0.0;
      }
      if (initObj.hasOwnProperty('direction')) {
        this.direction = initObj.direction
      }
      else {
        this.direction = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('signal_to_noise')) {
        this.signal_to_noise = initObj.signal_to_noise
      }
      else {
        this.signal_to_noise = 0.0;
      }
      if (initObj.hasOwnProperty('line_of_sight')) {
        this.line_of_sight = initObj.line_of_sight
      }
      else {
        this.line_of_sight = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IrDetection
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [emitter_id]
    bufferOffset = _serializer.string(obj.emitter_id, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float64(obj.distance, buffer, bufferOffset);
    // Serialize message field [strength]
    bufferOffset = _serializer.float64(obj.strength, buffer, bufferOffset);
    // Serialize message field [direction]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.direction, buffer, bufferOffset);
    // Serialize message field [signal_to_noise]
    bufferOffset = _serializer.float64(obj.signal_to_noise, buffer, bufferOffset);
    // Serialize message field [line_of_sight]
    bufferOffset = _serializer.bool(obj.line_of_sight, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IrDetection
    let len;
    let data = new IrDetection(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [emitter_id]
    data.emitter_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [strength]
    data.strength = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [direction]
    data.direction = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [signal_to_noise]
    data.signal_to_noise = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [line_of_sight]
    data.line_of_sight = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.emitter_id.length;
    return length + 53;
  }

  static datatype() {
    // Returns string type for a message object
    return 'infrared_msgs/IrDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8765f5c28a65d3ab4e34a4adb77d9b1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IrDetection(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.emitter_id !== undefined) {
      resolved.emitter_id = msg.emitter_id;
    }
    else {
      resolved.emitter_id = ''
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    if (msg.strength !== undefined) {
      resolved.strength = msg.strength;
    }
    else {
      resolved.strength = 0.0
    }

    if (msg.direction !== undefined) {
      resolved.direction = geometry_msgs.msg.Vector3.Resolve(msg.direction)
    }
    else {
      resolved.direction = new geometry_msgs.msg.Vector3()
    }

    if (msg.signal_to_noise !== undefined) {
      resolved.signal_to_noise = msg.signal_to_noise;
    }
    else {
      resolved.signal_to_noise = 0.0
    }

    if (msg.line_of_sight !== undefined) {
      resolved.line_of_sight = msg.line_of_sight;
    }
    else {
      resolved.line_of_sight = false
    }

    return resolved;
    }
};

module.exports = IrDetection;
