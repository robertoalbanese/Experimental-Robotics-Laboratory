;; Auto-generated. Do not edit!


(when (boundp 'positionserver::newPosition)
  (if (not (find-package "POSITIONSERVER"))
    (make-package "POSITIONSERVER"))
  (shadow 'newPosition (find-package "POSITIONSERVER")))
(unless (find-package "POSITIONSERVER::NEWPOSITION")
  (make-package "POSITIONSERVER::NEWPOSITION"))
(unless (find-package "POSITIONSERVER::NEWPOSITIONREQUEST")
  (make-package "POSITIONSERVER::NEWPOSITIONREQUEST"))
(unless (find-package "POSITIONSERVER::NEWPOSITIONRESPONSE")
  (make-package "POSITIONSERVER::NEWPOSITIONRESPONSE"))

(in-package "ROS")





(defclass positionserver::newPositionRequest
  :super ros::object
  :slots ())

(defmethod positionserver::newPositionRequest
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

(defclass positionserver::newPositionResponse
  :super ros::object
  :slots (_x _y ))

(defmethod positionserver::newPositionResponse
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

(defclass positionserver::newPosition
  :super ros::object
  :slots ())

(setf (get positionserver::newPosition :md5sum-) "ff8d7d66dd3e4b731ef14a45d38888b6")
(setf (get positionserver::newPosition :datatype-) "positionserver/newPosition")
(setf (get positionserver::newPosition :request) positionserver::newPositionRequest)
(setf (get positionserver::newPosition :response) positionserver::newPositionResponse)

(defmethod positionserver::newPositionRequest
  (:response () (instance positionserver::newPositionResponse :init)))

(setf (get positionserver::newPositionRequest :md5sum-) "ff8d7d66dd3e4b731ef14a45d38888b6")
(setf (get positionserver::newPositionRequest :datatype-) "positionserver/newPositionRequest")
(setf (get positionserver::newPositionRequest :definition-)
      "---
float32 x
float32 y
")

(setf (get positionserver::newPositionResponse :md5sum-) "ff8d7d66dd3e4b731ef14a45d38888b6")
(setf (get positionserver::newPositionResponse :datatype-) "positionserver/newPositionResponse")
(setf (get positionserver::newPositionResponse :definition-)
      "---
float32 x
float32 y
")



(provide :positionserver/newPosition "ff8d7d66dd3e4b731ef14a45d38888b6")


