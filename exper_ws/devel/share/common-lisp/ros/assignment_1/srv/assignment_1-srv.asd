
(cl:in-package :asdf)

(defsystem "assignment_1-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "get_pos" :depends-on ("_package_get_pos"))
    (:file "_package_get_pos" :depends-on ("_package"))
  ))