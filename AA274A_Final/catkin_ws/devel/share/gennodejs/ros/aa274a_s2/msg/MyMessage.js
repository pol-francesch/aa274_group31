// Auto-generated. Do not edit!

// (in-package aa274a_s2.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MyMessage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mic = null;
      this.s = null;
      this.f = null;
      this.c = null;
      this.idx = null;
    }
    else {
      if (initObj.hasOwnProperty('mic')) {
        this.mic = initObj.mic
      }
      else {
        this.mic = false;
      }
      if (initObj.hasOwnProperty('s')) {
        this.s = initObj.s
      }
      else {
        this.s = '';
      }
      if (initObj.hasOwnProperty('f')) {
        this.f = initObj.f
      }
      else {
        this.f = 0.0;
      }
      if (initObj.hasOwnProperty('c')) {
        this.c = initObj.c
      }
      else {
        this.c = 0;
      }
      if (initObj.hasOwnProperty('idx')) {
        this.idx = initObj.idx
      }
      else {
        this.idx = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MyMessage
    // Serialize message field [mic]
    bufferOffset = _serializer.bool(obj.mic, buffer, bufferOffset);
    // Serialize message field [s]
    bufferOffset = _serializer.string(obj.s, buffer, bufferOffset);
    // Serialize message field [f]
    bufferOffset = _serializer.float64(obj.f, buffer, bufferOffset);
    // Serialize message field [c]
    bufferOffset = _serializer.char(obj.c, buffer, bufferOffset);
    // Serialize message field [idx]
    bufferOffset = _serializer.int64(obj.idx, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MyMessage
    let len;
    let data = new MyMessage(null);
    // Deserialize message field [mic]
    data.mic = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [s]
    data.s = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [f]
    data.f = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [c]
    data.c = _deserializer.char(buffer, bufferOffset);
    // Deserialize message field [idx]
    data.idx = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.s);
    return length + 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aa274a_s2/MyMessage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5c584acd7d9101176cedf5e6e01732f5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool mic
    string s
    float64 f
    char c
    int64 idx
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MyMessage(null);
    if (msg.mic !== undefined) {
      resolved.mic = msg.mic;
    }
    else {
      resolved.mic = false
    }

    if (msg.s !== undefined) {
      resolved.s = msg.s;
    }
    else {
      resolved.s = ''
    }

    if (msg.f !== undefined) {
      resolved.f = msg.f;
    }
    else {
      resolved.f = 0.0
    }

    if (msg.c !== undefined) {
      resolved.c = msg.c;
    }
    else {
      resolved.c = 0
    }

    if (msg.idx !== undefined) {
      resolved.idx = msg.idx;
    }
    else {
      resolved.idx = 0
    }

    return resolved;
    }
};

module.exports = MyMessage;
