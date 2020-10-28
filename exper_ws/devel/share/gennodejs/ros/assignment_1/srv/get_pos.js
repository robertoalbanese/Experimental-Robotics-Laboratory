// Auto-generated. Do not edit!

// (in-package assignment_1.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class get_posRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.minx = null;
      this.maxx = null;
      this.miny = null;
      this.maxy = null;
    }
    else {
      if (initObj.hasOwnProperty('minx')) {
        this.minx = initObj.minx
      }
      else {
        this.minx = 0;
      }
      if (initObj.hasOwnProperty('maxx')) {
        this.maxx = initObj.maxx
      }
      else {
        this.maxx = 0;
      }
      if (initObj.hasOwnProperty('miny')) {
        this.miny = initObj.miny
      }
      else {
        this.miny = 0;
      }
      if (initObj.hasOwnProperty('maxy')) {
        this.maxy = initObj.maxy
      }
      else {
        this.maxy = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type get_posRequest
    // Serialize message field [minx]
    bufferOffset = _serializer.int64(obj.minx, buffer, bufferOffset);
    // Serialize message field [maxx]
    bufferOffset = _serializer.int64(obj.maxx, buffer, bufferOffset);
    // Serialize message field [miny]
    bufferOffset = _serializer.int64(obj.miny, buffer, bufferOffset);
    // Serialize message field [maxy]
    bufferOffset = _serializer.int64(obj.maxy, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type get_posRequest
    let len;
    let data = new get_posRequest(null);
    // Deserialize message field [minx]
    data.minx = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [maxx]
    data.maxx = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [miny]
    data.miny = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [maxy]
    data.maxy = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a service object
    return 'assignment_1/get_posRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '997f9bce9b979590fcbd1f50d2d0fe9c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 minx
    int64 maxx
    int64 miny
    int64 maxy
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new get_posRequest(null);
    if (msg.minx !== undefined) {
      resolved.minx = msg.minx;
    }
    else {
      resolved.minx = 0
    }

    if (msg.maxx !== undefined) {
      resolved.maxx = msg.maxx;
    }
    else {
      resolved.maxx = 0
    }

    if (msg.miny !== undefined) {
      resolved.miny = msg.miny;
    }
    else {
      resolved.miny = 0
    }

    if (msg.maxy !== undefined) {
      resolved.maxy = msg.maxy;
    }
    else {
      resolved.maxy = 0
    }

    return resolved;
    }
};

class get_posResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type get_posResponse
    // Serialize message field [x]
    bufferOffset = _serializer.int64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.int64(obj.y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type get_posResponse
    let len;
    let data = new get_posResponse(null);
    // Deserialize message field [x]
    data.x = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'assignment_1/get_posResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3b834ede922a0fff22c43585c533b49f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 x
    int64 y
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new get_posResponse(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: get_posRequest,
  Response: get_posResponse,
  md5sum() { return '689b3c9d7a6b02bd3b9a51ce8fe45a3b'; },
  datatype() { return 'assignment_1/get_pos'; }
};
