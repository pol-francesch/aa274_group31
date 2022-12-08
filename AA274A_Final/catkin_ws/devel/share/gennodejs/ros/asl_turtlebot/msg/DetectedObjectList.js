// Auto-generated. Do not edit!

// (in-package asl_turtlebot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DetectedObject = require('./DetectedObject.js');

//-----------------------------------------------------------

class DetectedObjectList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.objects = null;
      this.ob_msgs = null;
    }
    else {
      if (initObj.hasOwnProperty('objects')) {
        this.objects = initObj.objects
      }
      else {
        this.objects = [];
      }
      if (initObj.hasOwnProperty('ob_msgs')) {
        this.ob_msgs = initObj.ob_msgs
      }
      else {
        this.ob_msgs = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectedObjectList
    // Serialize message field [objects]
    bufferOffset = _arraySerializer.string(obj.objects, buffer, bufferOffset, null);
    // Serialize message field [ob_msgs]
    // Serialize the length for message field [ob_msgs]
    bufferOffset = _serializer.uint32(obj.ob_msgs.length, buffer, bufferOffset);
    obj.ob_msgs.forEach((val) => {
      bufferOffset = DetectedObject.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectedObjectList
    let len;
    let data = new DetectedObjectList(null);
    // Deserialize message field [objects]
    data.objects = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [ob_msgs]
    // Deserialize array length for message field [ob_msgs]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.ob_msgs = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.ob_msgs[i] = DetectedObject.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.objects.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.ob_msgs.forEach((val) => {
      length += DetectedObject.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'asl_turtlebot/DetectedObjectList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b55ad56e2dd8e9c68837a2cd0b12032a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] objects
    DetectedObject[] ob_msgs
    ================================================================================
    MSG: asl_turtlebot/DetectedObject
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
    const resolved = new DetectedObjectList(null);
    if (msg.objects !== undefined) {
      resolved.objects = msg.objects;
    }
    else {
      resolved.objects = []
    }

    if (msg.ob_msgs !== undefined) {
      resolved.ob_msgs = new Array(msg.ob_msgs.length);
      for (let i = 0; i < resolved.ob_msgs.length; ++i) {
        resolved.ob_msgs[i] = DetectedObject.Resolve(msg.ob_msgs[i]);
      }
    }
    else {
      resolved.ob_msgs = []
    }

    return resolved;
    }
};

module.exports = DetectedObjectList;
