;; Auto-generated. Do not edit!


(when (boundp 'ex1_server::rand_pos)
  (if (not (find-package "EX1_SERVER"))
    (make-package "EX1_SERVER"))
  (shadow 'rand_pos (find-package "EX1_SERVER")))
(unless (find-package "EX1_SERVER::RAND_POS")
  (make-package "EX1_SERVER::RAND_POS"))
(unless (find-package "EX1_SERVER::RAND_POSREQUEST")
  (make-package "EX1_SERVER::RAND_POSREQUEST"))
(unless (find-package "EX1_SERVER::RAND_POSRESPONSE")
  (make-package "EX1_SERVER::RAND_POSRESPONSE"))

(in-package "ROS")





(defclass ex1_server::rand_posRequest
  :super ros::object
  :slots ())

(defmethod ex1_server::rand_posRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass ex1_server::rand_posResponse
  :super ros::object
  :slots (_x _y ))

(defmethod ex1_server::rand_posResponse
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ex1_server::rand_pos
  :super ros::object
  :slots ())

(setf (get ex1_server::rand_pos :md5sum-) "ff8d7d66dd3e4b731ef14a45d38888b6")
(setf (get ex1_server::rand_pos :datatype-) "ex1_server/rand_pos")
(setf (get ex1_server::rand_pos :request) ex1_server::rand_posRequest)
(setf (get ex1_server::rand_pos :response) ex1_server::rand_posResponse)

(defmethod ex1_server::rand_posRequest
  (:response () (instance ex1_server::rand_posResponse :init)))

(setf (get ex1_server::rand_posRequest :md5sum-) "ff8d7d66dd3e4b731ef14a45d38888b6")
(setf (get ex1_server::rand_posRequest :datatype-) "ex1_server/rand_posRequest")
(setf (get ex1_server::rand_posRequest :definition-)
      "---
float32 x
float32 y
")

(setf (get ex1_server::rand_posResponse :md5sum-) "ff8d7d66dd3e4b731ef14a45d38888b6")
(setf (get ex1_server::rand_posResponse :datatype-) "ex1_server/rand_posResponse")
(setf (get ex1_server::rand_posResponse :definition-)
      "---
float32 x
float32 y
")



(provide :ex1_server/rand_pos "ff8d7d66dd3e4b731ef14a45d38888b6")


