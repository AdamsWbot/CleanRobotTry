; Auto-generated. Do not edit!


(cl:in-package clean_robot-srv)


;//! \htmlinclude GripperControl-request.msg.html

(cl:defclass <GripperControl-request> (roslisp-msg-protocol:ros-message)
  ((gripper_angle
    :reader gripper_angle
    :initarg :gripper_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass GripperControl-request (<GripperControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clean_robot-srv:<GripperControl-request> is deprecated: use clean_robot-srv:GripperControl-request instead.")))

(cl:ensure-generic-function 'gripper_angle-val :lambda-list '(m))
(cl:defmethod gripper_angle-val ((m <GripperControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clean_robot-srv:gripper_angle-val is deprecated.  Use clean_robot-srv:gripper_angle instead.")
  (gripper_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperControl-request>) ostream)
  "Serializes a message object of type '<GripperControl-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gripper_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperControl-request>) istream)
  "Deserializes a message object of type '<GripperControl-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gripper_angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperControl-request>)))
  "Returns string type for a service object of type '<GripperControl-request>"
  "clean_robot/GripperControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperControl-request)))
  "Returns string type for a service object of type 'GripperControl-request"
  "clean_robot/GripperControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperControl-request>)))
  "Returns md5sum for a message object of type '<GripperControl-request>"
  "55a3885cd6a0cb48fd314894a776c441")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperControl-request)))
  "Returns md5sum for a message object of type 'GripperControl-request"
  "55a3885cd6a0cb48fd314894a776c441")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperControl-request>)))
  "Returns full string definition for message of type '<GripperControl-request>"
  (cl:format cl:nil "#请求：爪部目标开合角度~%float32 gripper_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperControl-request)))
  "Returns full string definition for message of type 'GripperControl-request"
  (cl:format cl:nil "#请求：爪部目标开合角度~%float32 gripper_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperControl-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperControl-request
    (cl:cons ':gripper_angle (gripper_angle msg))
))
;//! \htmlinclude GripperControl-response.msg.html

(cl:defclass <GripperControl-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil)
   (current_angle
    :reader current_angle
    :initarg :current_angle
    :type cl:float
    :initform 0.0)
   (info
    :reader info
    :initarg :info
    :type cl:string
    :initform ""))
)

(cl:defclass GripperControl-response (<GripperControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clean_robot-srv:<GripperControl-response> is deprecated: use clean_robot-srv:GripperControl-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <GripperControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clean_robot-srv:result-val is deprecated.  Use clean_robot-srv:result instead.")
  (result m))

(cl:ensure-generic-function 'current_angle-val :lambda-list '(m))
(cl:defmethod current_angle-val ((m <GripperControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clean_robot-srv:current_angle-val is deprecated.  Use clean_robot-srv:current_angle instead.")
  (current_angle m))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <GripperControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clean_robot-srv:info-val is deprecated.  Use clean_robot-srv:info instead.")
  (info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperControl-response>) ostream)
  "Serializes a message object of type '<GripperControl-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'info))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperControl-response>) istream)
  "Deserializes a message object of type '<GripperControl-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'info) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'info) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperControl-response>)))
  "Returns string type for a service object of type '<GripperControl-response>"
  "clean_robot/GripperControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperControl-response)))
  "Returns string type for a service object of type 'GripperControl-response"
  "clean_robot/GripperControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperControl-response>)))
  "Returns md5sum for a message object of type '<GripperControl-response>"
  "55a3885cd6a0cb48fd314894a776c441")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperControl-response)))
  "Returns md5sum for a message object of type 'GripperControl-response"
  "55a3885cd6a0cb48fd314894a776c441")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperControl-response>)))
  "Returns full string definition for message of type '<GripperControl-response>"
  (cl:format cl:nil "#响应：执行结果+当前实际角度+结果说明~%bool result                 #ture=执行成功，false=执行失败~%float32 current_angle       #成功为目标角度，失败为上一帧有效角度~%string info                 #结果说明（角度合法，执行成功/执行失败—角度超出范围/角度变化过小）~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperControl-response)))
  "Returns full string definition for message of type 'GripperControl-response"
  (cl:format cl:nil "#响应：执行结果+当前实际角度+结果说明~%bool result                 #ture=执行成功，false=执行失败~%float32 current_angle       #成功为目标角度，失败为上一帧有效角度~%string info                 #结果说明（角度合法，执行成功/执行失败—角度超出范围/角度变化过小）~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperControl-response>))
  (cl:+ 0
     1
     4
     4 (cl:length (cl:slot-value msg 'info))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperControl-response
    (cl:cons ':result (result msg))
    (cl:cons ':current_angle (current_angle msg))
    (cl:cons ':info (info msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GripperControl)))
  'GripperControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GripperControl)))
  'GripperControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperControl)))
  "Returns string type for a service object of type '<GripperControl>"
  "clean_robot/GripperControl")