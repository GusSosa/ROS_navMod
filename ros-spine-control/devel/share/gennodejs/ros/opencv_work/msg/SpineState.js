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
      this.comy = null;
      this.comx = null;
    }
    else {
      if (initObj.hasOwnProperty('rotation')) {
        this.rotation = initObj.rotation
      }
      else {
        this.rotation = 0.0;
      }
      if (initObj.hasOwnProperty('comy')) {
        this.comy = initObj.comy
      }
      else {
        this.comy = 0.0;
      }
      if (initObj.hasOwnProperty('comx')) {
        this.comx = initObj.comx
      }
      else {
        this.comx = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SpineState
    // Serialize message field [rotation]
    bufferOffset = _serializer.float64(obj.rotation, buffer, bufferOffset);
    // Serialize message field [comy]
    bufferOffset = _serializer.float64(obj.comy, buffer, bufferOffset);
    // Serialize message field [comx]
    bufferOffset = _serializer.float64(obj.comx, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SpineState
    let len;
    let data = new SpineState(null);
    // Deserialize message field [rotation]
    data.rotation = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [comy]
    data.comy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [comx]
    data.comx = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencv_work/SpineState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '362faa163ba6b21bd1aa0295e7ccf8ab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 rotation
    float64 comy
    float64 comx
    
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

    if (msg.comy !== undefined) {
      resolved.comy = msg.comy;
    }
    else {
      resolved.comy = 0.0
    }

    if (msg.comx !== undefined) {
      resolved.comx = msg.comx;
    }
    else {
      resolved.comx = 0.0
    }

    return resolved;
    }
};

module.exports = SpineState;
