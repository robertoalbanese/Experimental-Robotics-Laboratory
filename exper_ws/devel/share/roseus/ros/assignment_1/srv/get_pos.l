;; Auto-generated. Do not edit!


(when (boundp 'assignment_1::get_pos)
  (if (not (find-package "ASSIGNMENT_1"))
    (make-package "ASSIGNMENT_1"))
  (shadow 'get_pos (find-package "ASSIGNMENT_1")))
(unless (find-package "ASSIGNMENT_1::GET_POS")
  (make-package "ASSIGNMENT_1::GET_POS"))
(unless (find-package "ASSIGNMENT_1::GET_POSREQUEST")
  (make-package "ASSIGNMENT_1::GET_POSREQUEST"))
(unless (find-package "ASSIGNMENT_1::GET_POSRESPONSE")
  (make-package "ASSIGNMENT_1::GET_POSRESPONSE"))

(in-package "ROS")





(defclass assignment_1::get_posRequest
  :super ros::object
  :slots (_minx _maxx _miny _maxy ))

(defmethod assignment_1::get_posRequest
  (:init
   (&key
    ((:minx __minx) 0)
    ((:maxx __maxx) 0)
    ((:miny __miny) 0)
    ((:maxy __maxy) 0)
    )
   (send-super :init)
   (setq _minx (round __minx))
   (setq _maxx (round __maxx))
   (setq _miny (round __miny))
   (setq _maxy (round __maxy))
   self)
  (:minx
   (&optional __minx)
   (if __minx (setq _minx __minx)) _minx)
  (:maxx
   (&optional __maxx)
   (if __maxx (setq _maxx __maxx)) _maxx)
  (:miny
   (&optional __miny)
   (if __miny (setq _miny __miny)) _miny)
  (:maxy
   (&optional __maxy)
   (if __maxy (setq _maxy __maxy)) _maxy)
  (:serialization-length
   ()
   (+
    ;; int64 _minx
    8
    ;; int64 _maxx
    8
    ;; int64 _miny
    8
    ;; int64 _maxy
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _minx
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _minx (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _minx) (= (length (_minx . bv)) 2)) ;; bignum
              (write-long (ash (elt (_minx . bv) 0) 0) s)
              (write-long (ash (elt (_minx . bv) 1) -1) s))
             ((and (class _minx) (= (length (_minx . bv)) 1)) ;; big1
              (write-long (elt (_minx . bv) 0) s)
              (write-long (if (>= _minx 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _minx s)(write-long (if (>= _minx 0) 0 #xffffffff) s)))
     ;; int64 _maxx
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _maxx (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _maxx) (= (length (_maxx . bv)) 2)) ;; bignum
              (write-long (ash (elt (_maxx . bv) 0) 0) s)
              (write-long (ash (elt (_maxx . bv) 1) -1) s))
             ((and (class _maxx) (= (length (_maxx . bv)) 1)) ;; big1
              (write-long (elt (_maxx . bv) 0) s)
              (write-long (if (>= _maxx 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _maxx s)(write-long (if (>= _maxx 0) 0 #xffffffff) s)))
     ;; int64 _miny
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _miny (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _miny) (= (length (_miny . bv)) 2)) ;; bignum
              (write-long (ash (elt (_miny . bv) 0) 0) s)
              (write-long (ash (elt (_miny . bv) 1) -1) s))
             ((and (class _miny) (= (length (_miny . bv)) 1)) ;; big1
              (write-long (elt (_miny . bv) 0) s)
              (write-long (if (>= _miny 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _miny s)(write-long (if (>= _miny 0) 0 #xffffffff) s)))
     ;; int64 _maxy
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _maxy (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _maxy) (= (length (_maxy . bv)) 2)) ;; bignum
              (write-long (ash (elt (_maxy . bv) 0) 0) s)
              (write-long (ash (elt (_maxy . bv) 1) -1) s))
             ((and (class _maxy) (= (length (_maxy . bv)) 1)) ;; big1
              (write-long (elt (_maxy . bv) 0) s)
              (write-long (if (>= _maxy 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _maxy s)(write-long (if (>= _maxy 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _minx
#+(or :alpha :irix6 :x86_64)
      (setf _minx (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _minx (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _maxx
#+(or :alpha :irix6 :x86_64)
      (setf _maxx (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _maxx (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _miny
#+(or :alpha :irix6 :x86_64)
      (setf _miny (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _miny (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _maxy
#+(or :alpha :irix6 :x86_64)
      (setf _maxy (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _maxy (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass assignment_1::get_posResponse
  :super ros::object
  :slots (_x _y ))

(defmethod assignment_1::get_posResponse
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
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
    ;; int64 _x
    8
    ;; int64 _y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _x
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _x (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _x) (= (length (_x . bv)) 2)) ;; bignum
              (write-long (ash (elt (_x . bv) 0) 0) s)
              (write-long (ash (elt (_x . bv) 1) -1) s))
             ((and (class _x) (= (length (_x . bv)) 1)) ;; big1
              (write-long (elt (_x . bv) 0) s)
              (write-long (if (>= _x 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _x s)(write-long (if (>= _x 0) 0 #xffffffff) s)))
     ;; int64 _y
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _y (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _y) (= (length (_y . bv)) 2)) ;; bignum
              (write-long (ash (elt (_y . bv) 0) 0) s)
              (write-long (ash (elt (_y . bv) 1) -1) s))
             ((and (class _y) (= (length (_y . bv)) 1)) ;; big1
              (write-long (elt (_y . bv) 0) s)
              (write-long (if (>= _y 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _y s)(write-long (if (>= _y 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _x
#+(or :alpha :irix6 :x86_64)
      (setf _x (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _x (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _y
#+(or :alpha :irix6 :x86_64)
      (setf _y (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _y (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass assignment_1::get_pos
  :super ros::object
  :slots ())

(setf (get assignment_1::get_pos :md5sum-) "689b3c9d7a6b02bd3b9a51ce8fe45a3b")
(setf (get assignment_1::get_pos :datatype-) "assignment_1/get_pos")
(setf (get assignment_1::get_pos :request) assignment_1::get_posRequest)
(setf (get assignment_1::get_pos :response) assignment_1::get_posResponse)

(defmethod assignment_1::get_posRequest
  (:response () (instance assignment_1::get_posResponse :init)))

(setf (get assignment_1::get_posRequest :md5sum-) "689b3c9d7a6b02bd3b9a51ce8fe45a3b")
(setf (get assignment_1::get_posRequest :datatype-) "assignment_1/get_posRequest")
(setf (get assignment_1::get_posRequest :definition-)
      "int64 minx
int64 maxx
int64 miny
int64 maxy
---
int64 x
int64 y

")

(setf (get assignment_1::get_posResponse :md5sum-) "689b3c9d7a6b02bd3b9a51ce8fe45a3b")
(setf (get assignment_1::get_posResponse :datatype-) "assignment_1/get_posResponse")
(setf (get assignment_1::get_posResponse :definition-)
      "int64 minx
int64 maxx
int64 miny
int64 maxy
---
int64 x
int64 y

")



(provide :assignment_1/get_pos "689b3c9d7a6b02bd3b9a51ce8fe45a3b")


