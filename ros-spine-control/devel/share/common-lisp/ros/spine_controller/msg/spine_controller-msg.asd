
(cl:in-package :asdf)

(defsystem "spine_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "InvkinControlCommand" :depends-on ("_package_InvkinControlCommand"))
    (:file "_package_InvkinControlCommand" :depends-on ("_package"))
  ))