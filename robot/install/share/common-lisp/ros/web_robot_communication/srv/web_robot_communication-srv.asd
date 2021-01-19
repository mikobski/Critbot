
(cl:in-package :asdf)

(defsystem "web_robot_communication-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ModeChanges" :depends-on ("_package_ModeChanges"))
    (:file "_package_ModeChanges" :depends-on ("_package"))
  ))