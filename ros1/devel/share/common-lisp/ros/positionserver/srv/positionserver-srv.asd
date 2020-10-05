
(cl:in-package :asdf)

(defsystem "positionserver-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "newPosition" :depends-on ("_package_newPosition"))
    (:file "_package_newPosition" :depends-on ("_package"))
  ))