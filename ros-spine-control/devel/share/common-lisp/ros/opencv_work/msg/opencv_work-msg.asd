
(cl:in-package :asdf)

(defsystem "opencv_work-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SpineState" :depends-on ("_package_SpineState"))
    (:file "_package_SpineState" :depends-on ("_package"))
  ))