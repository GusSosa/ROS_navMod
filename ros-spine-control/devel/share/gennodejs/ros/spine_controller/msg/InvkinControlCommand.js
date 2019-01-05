// Auto-generated. Do not edit!

// (in-package spine_controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class InvkinControlCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.invkin_control = null;
      this.invkin_ref_state = null;
    }
    else {
      if (initObj.hasOwnProperty('invkin_control')) {
        this.invkin_control = initObj.invkin_control
      }
      else {
        this.invkin_control = [];
      }
      if (initObj.hasOwnProperty('invkin_ref_state')) {
        this.invkin_ref_state = initObj.invkin_ref_state
      }
      else {
        this.invkin_ref_state = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InvkinControlCommand
    // Serialize message field [invkin_control]
    bufferOffset = _arraySerializer.float64(obj.invkin_control, buffer, bufferOffset, null);
    // Serialize message field [invkin_ref_state]
    bufferOffset = _arraySerializer.float64(obj.invkin_ref_state, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InvkinControlCommand
    let len;
    let data = new InvkinControlCommand(null);
    // Deserialize message field [invkin_control]
    data.invkin_control = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [invkin_ref_state]
    data.invkin_ref_state = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.invkin_control.length;
    length += 8 * object.invkin_ref_state.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'spine_controller/InvkinControlCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ad1887908a0dd9bc527fc66e7b154313';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] invkin_control
    float64[] invkin_ref_state
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InvkinControlCommand(null);
    if (msg.invkin_control !== undefined) {
      resolved.invkin_control = msg.invkin_control;
    }
    else {
      resolved.invkin_control = []
    }

    if (msg.invkin_ref_state !== undefined) {
      resolved.invkin_ref_state = msg.invkin_ref_state;
    }
    else {
      resolved.invkin_ref_state = []
    }

    return resolved;
    }
};

module.exports = InvkinControlCommand;
