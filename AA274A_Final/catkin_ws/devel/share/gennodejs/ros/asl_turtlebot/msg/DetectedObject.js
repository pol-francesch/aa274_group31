// Auto-generated. Do not edit!

// (in-package asl_turtlebot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DetectedObject {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.name = null;
      this.confidence = null;
      this.distance = null;
      this.thetaleft = null;
      this.thetaright = null;
      this.corners = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
      if (initObj.hasOwnProperty('thetaleft')) {
        this.thetaleft = initObj.thetaleft
      }
      else {
        this.thetaleft = 0.0;
      }
      if (initObj.hasOwnProperty('thetaright')) {
        this.thetaright = initObj.thetaright
      }
      else {
        this.thetaright = 0.0;
      }
      if (initObj.hasOwnProperty('corners')) {
        this.corners = initObj.corners
      }
      else {
        this.corners = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectedObject
    // Serialize message field [id]
    bufferOffset = _serializer.uint32(obj.id, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float64(obj.confidence, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float64(obj.distance, buffer, bufferOffset);
    // Serialize message field [thetaleft]
    bufferOffset = _serializer.float64(obj.thetaleft, buffer, bufferOffset);
    // Serialize message field [thetaright]
    bufferOffset = _serializer.float64(obj.thetaright, buffer, bufferOffset);
    // Serialize message field [corners]
    bufferOffset = _arraySerializer.float64(obj.corners, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectedObject
    let len;
    let data = new DetectedObject(null);
    // Deserialize message field [id]
    data.id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [thetaleft]
    data.thetaleft = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [thetaright]
    data.thetaright = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [corners]
    data.corners = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += 8 * object.corners.length;
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'asl_turtlebot/DetectedObject';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2650e98dd260a5b8590eced757507d05';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 id
    string name
    float64 confidence
    float64 distance
    float64 thetaleft
    float64 thetaright
    float64[] corners
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DetectedObject(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    if (msg.thetaleft !== undefined) {
      resolved.thetaleft = msg.thetaleft;
    }
    else {
      resolved.thetaleft = 0.0
    }

    if (msg.thetaright !== undefined) {
      resolved.thetaright = msg.thetaright;
    }
    else {
      resolved.thetaright = 0.0
    }

    if (msg.corners !== undefined) {
      resolved.corners = msg.corners;
    }
    else {
      resolved.corners = []
    }

    return resolved;
    }
};

module.exports = DetectedObject;
