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

class IrSignal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.emitter_id = null;
      this.position = null;
      this.direction = null;
      this.range = null;
      this.strength = null;
      this.frequency = null;
      this.modulation = null;
      this.wavelength = null;
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
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('direction')) {
        this.direction = initObj.direction
      }
      else {
        this.direction = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('range')) {
        this.range = initObj.range
      }
      else {
        this.range = 0.0;
      }
      if (initObj.hasOwnProperty('strength')) {
        this.strength = initObj.strength
      }
      else {
        this.strength = 0.0;
      }
      if (initObj.hasOwnProperty('frequency')) {
        this.frequency = initObj.frequency
      }
      else {
        this.frequency = 0.0;
      }
      if (initObj.hasOwnProperty('modulation')) {
        this.modulation = initObj.modulation
      }
      else {
        this.modulation = '';
      }
      if (initObj.hasOwnProperty('wavelength')) {
        this.wavelength = initObj.wavelength
      }
      else {
        this.wavelength = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IrSignal
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [emitter_id]
    bufferOffset = _serializer.string(obj.emitter_id, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [direction]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.direction, buffer, bufferOffset);
    // Serialize message field [range]
    bufferOffset = _serializer.float64(obj.range, buffer, bufferOffset);
    // Serialize message field [strength]
    bufferOffset = _serializer.float64(obj.strength, buffer, bufferOffset);
    // Serialize message field [frequency]
    bufferOffset = _serializer.float64(obj.frequency, buffer, bufferOffset);
    // Serialize message field [modulation]
    bufferOffset = _serializer.string(obj.modulation, buffer, bufferOffset);
    // Serialize message field [wavelength]
    bufferOffset = _serializer.float64(obj.wavelength, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IrSignal
    let len;
    let data = new IrSignal(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [emitter_id]
    data.emitter_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [direction]
    data.direction = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [range]
    data.range = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [strength]
    data.strength = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [frequency]
    data.frequency = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [modulation]
    data.modulation = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [wavelength]
    data.wavelength = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.emitter_id.length;
    length += object.modulation.length;
    return length + 88;
  }

  static datatype() {
    // Returns string type for a message object
    return 'infrared_msgs/IrSignal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2b33114226fe9c26f156b572ed403903';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IrSignal(null);
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

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.direction !== undefined) {
      resolved.direction = geometry_msgs.msg.Vector3.Resolve(msg.direction)
    }
    else {
      resolved.direction = new geometry_msgs.msg.Vector3()
    }

    if (msg.range !== undefined) {
      resolved.range = msg.range;
    }
    else {
      resolved.range = 0.0
    }

    if (msg.strength !== undefined) {
      resolved.strength = msg.strength;
    }
    else {
      resolved.strength = 0.0
    }

    if (msg.frequency !== undefined) {
      resolved.frequency = msg.frequency;
    }
    else {
      resolved.frequency = 0.0
    }

    if (msg.modulation !== undefined) {
      resolved.modulation = msg.modulation;
    }
    else {
      resolved.modulation = ''
    }

    if (msg.wavelength !== undefined) {
      resolved.wavelength = msg.wavelength;
    }
    else {
      resolved.wavelength = 0.0
    }

    return resolved;
    }
};

module.exports = IrSignal;
