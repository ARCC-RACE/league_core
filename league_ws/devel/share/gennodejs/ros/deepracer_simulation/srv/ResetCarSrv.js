// Auto-generated. Do not edit!

// (in-package deepracer_simulation.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ResetCarSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ndist = null;
      this.next_index = null;
    }
    else {
      if (initObj.hasOwnProperty('ndist')) {
        this.ndist = initObj.ndist
      }
      else {
        this.ndist = 0.0;
      }
      if (initObj.hasOwnProperty('next_index')) {
        this.next_index = initObj.next_index
      }
      else {
        this.next_index = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetCarSrvRequest
    // Serialize message field [ndist]
    bufferOffset = _serializer.float32(obj.ndist, buffer, bufferOffset);
    // Serialize message field [next_index]
    bufferOffset = _serializer.int32(obj.next_index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetCarSrvRequest
    let len;
    let data = new ResetCarSrvRequest(null);
    // Deserialize message field [ndist]
    data.ndist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [next_index]
    data.next_index = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_simulation/ResetCarSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eff28ed986b58db32d8bdead6e257efe';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 ndist
    int32 next_index
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetCarSrvRequest(null);
    if (msg.ndist !== undefined) {
      resolved.ndist = msg.ndist;
    }
    else {
      resolved.ndist = 0.0
    }

    if (msg.next_index !== undefined) {
      resolved.next_index = msg.next_index;
    }
    else {
      resolved.next_index = 0
    }

    return resolved;
    }
};

class ResetCarSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetCarSrvResponse
    // Serialize message field [success]
    bufferOffset = _serializer.int32(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetCarSrvResponse
    let len;
    let data = new ResetCarSrvResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_simulation/ResetCarSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3c2bcf2ff0894cb3058b1bf4c8c4175a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetCarSrvResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: ResetCarSrvRequest,
  Response: ResetCarSrvResponse,
  md5sum() { return '51459ac9873b166b6314083bf1771edb'; },
  datatype() { return 'deepracer_simulation/ResetCarSrv'; }
};
