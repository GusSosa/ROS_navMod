// Auto-generated. Do not edit!

// (in-package opencv_work.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SpineState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rotation = null;
      this.com1 = null;
      this.com2 = null;
    }
    else {
      if (initObj.hasOwnProperty('rotation')) {
        this.rotation = initObj.rotation
      }
      else {
        this.rotation = 0.0;
      }
      if (initObj.hasOwnProperty('com1')) {
        this.com1 = initObj.com1
      }
      else {
        this.com1 = [];
      }
      if (initObj.hasOwnProperty('com2')) {
        this.com2 = initObj.com2
      }
      else {
        this.com2 = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SpineState
    // Serialize message field [rotation]
    bufferOffset = _serializer.float64(obj.rotation, buffer, bufferOffset);
    // Serialize message field [com1]
    bufferOffset = _arraySerializer.float64(obj.com1, buffer, bufferOffset, null);
    // Serialize message field [com2]
    bufferOffset = _arraySerializer.float64(obj.com2, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SpineState
    let len;
    let data = new SpineState(null);
    // Deserialize message field [rotation]
    data.rotation = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [com1]
    data.com1 = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [com2]
    data.com2 = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.com1.length;
    length += 8 * object.com2.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_work/SpineState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0c4ed841ebd6c1ed336e4b4bb0dbbca7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 rotation
    float64[] com1
    float64[] com2
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SpineState(null);
    if (msg.rotation !== undefined) {
      resolved.rotation = msg.rotation;
    }
    else {
      resolved.rotation = 0.0
    }

    if (msg.com1 !== undefined) {
      resolved.com1 = msg.com1;
    }
    else {
      resolved.com1 = []
    }

    if (msg.com2 !== undefined) {
      resolved.com2 = msg.com2;
    }
    else {
      resolved.com2 = []
    }

    return resolved;
    }
};

module.exports = SpineState;
