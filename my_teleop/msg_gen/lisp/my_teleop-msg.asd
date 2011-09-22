
(cl:in-package :asdf)

(defsystem "my_teleop-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Button" :depends-on ("_package_Button"))
    (:file "_package_Button" :depends-on ("_package"))
    (:file "Axis" :depends-on ("_package_Axis"))
    (:file "_package_Axis" :depends-on ("_package"))
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
  ))