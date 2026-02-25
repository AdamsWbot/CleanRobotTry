;; Auto-generated. Do not edit!


(when (boundp 'clean_robot::GripperControl)
  (if (not (find-package "CLEAN_ROBOT"))
    (make-package "CLEAN_ROBOT"))
  (shadow 'GripperControl (find-package "CLEAN_ROBOT")))
(unless (find-package "CLEAN_ROBOT::GRIPPERCONTROL")
  (make-package "CLEAN_ROBOT::GRIPPERCONTROL"))
(unless (find-package "CLEAN_ROBOT::GRIPPERCONTROLREQUEST")
  (make-package "CLEAN_ROBOT::GRIPPERCONTROLREQUEST"))
(unless (find-package "CLEAN_ROBOT::GRIPPERCONTROLRESPONSE")
  (make-package "CLEAN_ROBOT::GRIPPERCONTROLRESPONSE"))

(in-package "ROS")





(defclass clean_robot::GripperControlRequest
  :super ros::object
  :slots (_gripper_angle ))

(defmethod clean_robot::GripperControlRequest
  (:init
   (&key
    ((:gripper_angle __gripper_angle) 0.0)
    )
   (send-super :init)
   (setq _gripper_angle (float __gripper_angle))
   self)
  (:gripper_angle
   (&optional __gripper_angle)
   (if __gripper_angle (setq _gripper_angle __gripper_angle)) _gripper_angle)
  (:serialization-length
   ()
   (+
    ;; float32 _gripper_angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _gripper_angle
       (sys::poke _gripper_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _gripper_angle
     (setq _gripper_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass clean_robot::GripperControlResponse
  :super ros::object
  :slots (_result _current_angle _info ))

(defmethod clean_robot::GripperControlResponse
  (:init
   (&key
    ((:result __result) nil)
    ((:current_angle __current_angle) 0.0)
    ((:info __info) "")
    )
   (send-super :init)
   (setq _result __result)
   (setq _current_angle (float __current_angle))
   (setq _info (string __info))
   self)
  (:result
   (&optional (__result :null))
   (if (not (eq __result :null)) (setq _result __result)) _result)
  (:current_angle
   (&optional __current_angle)
   (if __current_angle (setq _current_angle __current_angle)) _current_angle)
  (:info
   (&optional __info)
   (if __info (setq _info __info)) _info)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ;; float32 _current_angle
    4
    ;; string _info
    4 (length _info)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;; float32 _current_angle
       (sys::poke _current_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _info
       (write-long (length _info) s) (princ _info s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _current_angle
     (setq _current_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _info
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _info (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass clean_robot::GripperControl
  :super ros::object
  :slots ())

(setf (get clean_robot::GripperControl :md5sum-) "55a3885cd6a0cb48fd314894a776c441")
(setf (get clean_robot::GripperControl :datatype-) "clean_robot/GripperControl")
(setf (get clean_robot::GripperControl :request) clean_robot::GripperControlRequest)
(setf (get clean_robot::GripperControl :response) clean_robot::GripperControlResponse)

(defmethod clean_robot::GripperControlRequest
  (:response () (instance clean_robot::GripperControlResponse :init)))

(setf (get clean_robot::GripperControlRequest :md5sum-) "55a3885cd6a0cb48fd314894a776c441")
(setf (get clean_robot::GripperControlRequest :datatype-) "clean_robot/GripperControlRequest")
(setf (get clean_robot::GripperControlRequest :definition-)
      "#请求：爪部目标开合角度
float32 gripper_angle
---
#响应：执行结果+当前实际角度+结果说明
bool result                 #ture=执行成功，false=执行失败
float32 current_angle       #成功为目标角度，失败为上一帧有效角度
string info                 #结果说明（角度合法，执行成功/执行失败—角度超出范围/角度变化过小）
")

(setf (get clean_robot::GripperControlResponse :md5sum-) "55a3885cd6a0cb48fd314894a776c441")
(setf (get clean_robot::GripperControlResponse :datatype-) "clean_robot/GripperControlResponse")
(setf (get clean_robot::GripperControlResponse :definition-)
      "#请求：爪部目标开合角度
float32 gripper_angle
---
#响应：执行结果+当前实际角度+结果说明
bool result                 #ture=执行成功，false=执行失败
float32 current_angle       #成功为目标角度，失败为上一帧有效角度
string info                 #结果说明（角度合法，执行成功/执行失败—角度超出范围/角度变化过小）
")



(provide :clean_robot/GripperControl "55a3885cd6a0cb48fd314894a776c441")


