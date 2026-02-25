
(cl:in-package :asdf)

(defsystem "clean_robot-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GripperControl" :depends-on ("_package_GripperControl"))
    (:file "_package_GripperControl" :depends-on ("_package"))
  ))