// Auto-generated. Do not edit!

// (in-package monarch_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class HeadControlSemantic {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cardinal_direction = null;
      this.speed = null;
    }
    else {
      if (initObj.hasOwnProperty('cardinal_direction')) {
        this.cardinal_direction = initObj.cardinal_direction
      }
      else {
        this.cardinal_direction = '';
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HeadControlSemantic
    // Serialize message field [cardinal_direction]
    bufferOffset = _serializer.string(obj.cardinal_direction, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.uint8(obj.speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HeadControlSemantic
    let len;
    let data = new HeadControlSemantic(null);
    // Deserialize message field [cardinal_direction]
    data.cardinal_direction = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.cardinal_direction.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'monarch_msgs/HeadControlSemantic';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8f7a062da2d04ec1d87e1a59f64f6af';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # If you add more, please do not break the current definitions
    uint8 NORMAL=0
    uint8 SLOW=1
    uint8 FAST=2
    
    # Example for NW, accepts:
    # {NW, NORTHEAST}
    string cardinal_direction
    uint8 speed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HeadControlSemantic(null);
    if (msg.cardinal_direction !== undefined) {
      resolved.cardinal_direction = msg.cardinal_direction;
    }
    else {
      resolved.cardinal_direction = ''
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0
    }

    return resolved;
    }
};

// Constants for message
HeadControlSemantic.Constants = {
  NORMAL: 0,
  SLOW: 1,
  FAST: 2,
}

module.exports = HeadControlSemantic;
