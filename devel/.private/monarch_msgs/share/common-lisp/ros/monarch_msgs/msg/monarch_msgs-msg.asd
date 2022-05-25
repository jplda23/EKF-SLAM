
(cl:in-package :asdf)

(defsystem "monarch_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "HeadControlSemantic" :depends-on ("_package_HeadControlSemantic"))
    (:file "_package_HeadControlSemantic" :depends-on ("_package"))
  ))