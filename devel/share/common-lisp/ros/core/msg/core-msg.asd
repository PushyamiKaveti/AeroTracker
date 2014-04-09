
(cl:in-package :asdf)

(defsystem "core-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Mavlink" :depends-on ("_package_Mavlink"))
    (:file "_package_Mavlink" :depends-on ("_package"))
    (:file "Camera_msgs" :depends-on ("_package_Camera_msgs"))
    (:file "_package_Camera_msgs" :depends-on ("_package"))
  ))