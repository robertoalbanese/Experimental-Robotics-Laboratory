
(cl:in-package :asdf)

(defsystem "ex1_server-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "rand_pos" :depends-on ("_package_rand_pos"))
    (:file "_package_rand_pos" :depends-on ("_package"))
  ))