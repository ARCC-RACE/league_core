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

class GetWaypointSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetWaypointSrvRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetWaypointSrvRequest
    let len;
    let data = new GetWaypointSrvRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_simulation/GetWaypointSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetWaypointSrvRequest(null);
    return resolved;
    }
};

class GetWaypointSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.row = null;
      this.col = null;
      this.waypoints = null;
    }
    else {
      if (initObj.hasOwnProperty('row')) {
        this.row = initObj.row
      }
      else {
        this.row = 0;
      }
      if (initObj.hasOwnProperty('col')) {
        this.col = initObj.col
      }
      else {
        this.col = 0;
      }
      if (initObj.hasOwnProperty('waypoints')) {
        this.waypoints = initObj.waypoints
      }
      else {
        this.waypoints = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetWaypointSrvResponse
    // Serialize message field [row]
    bufferOffset = _serializer.int32(obj.row, buffer, bufferOffset);
    // Serialize message field [col]
    bufferOffset = _serializer.int32(obj.col, buffer, bufferOffset);
    // Serialize message field [waypoints]
    bufferOffset = _arraySerializer.float64(obj.waypoints, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetWaypointSrvResponse
    let len;
    let data = new GetWaypointSrvResponse(null);
    // Deserialize message field [row]
    data.row = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [col]
    data.col = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [waypoints]
    data.waypoints = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.waypoints.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_simulation/GetWaypointSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dd1c3c0f312afb554365a4b5e8f07a10';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 row
    int32 col
    float64[] waypoints
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetWaypointSrvResponse(null);
    if (msg.row !== undefined) {
      resolved.row = msg.row;
    }
    else {
      resolved.row = 0
    }

    if (msg.col !== undefined) {
      resolved.col = msg.col;
    }
    else {
      resolved.col = 0
    }

    if (msg.waypoints !== undefined) {
      resolved.waypoints = msg.waypoints;
    }
    else {
      resolved.waypoints = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GetWaypointSrvRequest,
  Response: GetWaypointSrvResponse,
  md5sum() { return 'dd1c3c0f312afb554365a4b5e8f07a10'; },
  datatype() { return 'deepracer_simulation/GetWaypointSrv'; }
};
