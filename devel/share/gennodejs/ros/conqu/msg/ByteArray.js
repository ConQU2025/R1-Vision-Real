// Auto-generated. Do not edit!

// (in-package conqu.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ByteArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = new Array(10).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ByteArray
    // Check that the constant length array field [data] has the right length
    if (obj.data.length !== 10) {
      throw new Error('Unable to serialize array field data - length must be 10')
    }
    // Serialize message field [data]
    bufferOffset = _arraySerializer.int8(obj.data, buffer, bufferOffset, 10);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ByteArray
    let len;
    let data = new ByteArray(null);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.int8(buffer, bufferOffset, 10)
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'conqu/ByteArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '56792dc2ab08b5a97efe471984a66192';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8[10] data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ByteArray(null);
    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = new Array(10).fill(0)
    }

    return resolved;
    }
};

module.exports = ByteArray;
