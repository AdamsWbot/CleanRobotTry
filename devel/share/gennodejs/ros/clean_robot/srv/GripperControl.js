// Auto-generated. Do not edit!

// (in-package clean_robot.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GripperControlRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gripper_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('gripper_angle')) {
        this.gripper_angle = initObj.gripper_angle
      }
      else {
        this.gripper_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperControlRequest
    // Serialize message field [gripper_angle]
    bufferOffset = _serializer.float32(obj.gripper_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperControlRequest
    let len;
    let data = new GripperControlRequest(null);
    // Deserialize message field [gripper_angle]
    data.gripper_angle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'clean_robot/GripperControlRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc962a3df0a5f2cbbb32dc805bc415f5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #请求：爪部目标开合角度
    float32 gripper_angle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperControlRequest(null);
    if (msg.gripper_angle !== undefined) {
      resolved.gripper_angle = msg.gripper_angle;
    }
    else {
      resolved.gripper_angle = 0.0
    }

    return resolved;
    }
};

class GripperControlResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
      this.current_angle = null;
      this.info = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = false;
      }
      if (initObj.hasOwnProperty('current_angle')) {
        this.current_angle = initObj.current_angle
      }
      else {
        this.current_angle = 0.0;
      }
      if (initObj.hasOwnProperty('info')) {
        this.info = initObj.info
      }
      else {
        this.info = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperControlResponse
    // Serialize message field [result]
    bufferOffset = _serializer.bool(obj.result, buffer, bufferOffset);
    // Serialize message field [current_angle]
    bufferOffset = _serializer.float32(obj.current_angle, buffer, bufferOffset);
    // Serialize message field [info]
    bufferOffset = _serializer.string(obj.info, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperControlResponse
    let len;
    let data = new GripperControlResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [current_angle]
    data.current_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [info]
    data.info = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.info);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'clean_robot/GripperControlResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '97b215a2a03506f046ae385e68943d66';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #响应：执行结果+当前实际角度+结果说明
    bool result                 #ture=执行成功，false=执行失败
    float32 current_angle       #成功为目标角度，失败为上一帧有效角度
    string info                 #结果说明（角度合法，执行成功/执行失败—角度超出范围/角度变化过小）
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperControlResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = false
    }

    if (msg.current_angle !== undefined) {
      resolved.current_angle = msg.current_angle;
    }
    else {
      resolved.current_angle = 0.0
    }

    if (msg.info !== undefined) {
      resolved.info = msg.info;
    }
    else {
      resolved.info = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: GripperControlRequest,
  Response: GripperControlResponse,
  md5sum() { return '55a3885cd6a0cb48fd314894a776c441'; },
  datatype() { return 'clean_robot/GripperControl'; }
};
