; Auto-generated. Do not edit!


(cl:in-package assignment_1-srv)


;//! \htmlinclude get_pos-request.msg.html

(cl:defclass <get_pos-request> (roslisp-msg-protocol:ros-message)
  ((minx
    :reader minx
    :initarg :minx
    :type cl:integer
    :initform 0)
   (maxx
    :reader maxx
    :initarg :maxx
    :type cl:integer
    :initform 0)
   (miny
    :reader miny
    :initarg :miny
    :type cl:integer
    :initform 0)
   (maxy
    :reader maxy
    :initarg :maxy
    :type cl:integer
    :initform 0))
)

(cl:defclass get_pos-request (<get_pos-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_pos-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_pos-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment_1-srv:<get_pos-request> is deprecated: use assignment_1-srv:get_pos-request instead.")))

(cl:ensure-generic-function 'minx-val :lambda-list '(m))
(cl:defmethod minx-val ((m <get_pos-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment_1-srv:minx-val is deprecated.  Use assignment_1-srv:minx instead.")
  (minx m))

(cl:ensure-generic-function 'maxx-val :lambda-list '(m))
(cl:defmethod maxx-val ((m <get_pos-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment_1-srv:maxx-val is deprecated.  Use assignment_1-srv:maxx instead.")
  (maxx m))

(cl:ensure-generic-function 'miny-val :lambda-list '(m))
(cl:defmethod miny-val ((m <get_pos-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment_1-srv:miny-val is deprecated.  Use assignment_1-srv:miny instead.")
  (miny m))

(cl:ensure-generic-function 'maxy-val :lambda-list '(m))
(cl:defmethod maxy-val ((m <get_pos-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment_1-srv:maxy-val is deprecated.  Use assignment_1-srv:maxy instead.")
  (maxy m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_pos-request>) ostream)
  "Serializes a message object of type '<get_pos-request>"
  (cl:let* ((signed (cl:slot-value msg 'minx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'maxx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'miny)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'maxy)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_pos-request>) istream)
  "Deserializes a message object of type '<get_pos-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'minx) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'maxx) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'miny) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'maxy) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_pos-request>)))
  "Returns string type for a service object of type '<get_pos-request>"
  "assignment_1/get_posRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_pos-request)))
  "Returns string type for a service object of type 'get_pos-request"
  "assignment_1/get_posRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_pos-request>)))
  "Returns md5sum for a message object of type '<get_pos-request>"
  "689b3c9d7a6b02bd3b9a51ce8fe45a3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_pos-request)))
  "Returns md5sum for a message object of type 'get_pos-request"
  "689b3c9d7a6b02bd3b9a51ce8fe45a3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_pos-request>)))
  "Returns full string definition for message of type '<get_pos-request>"
  (cl:format cl:nil "int64 minx~%int64 maxx~%int64 miny~%int64 maxy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_pos-request)))
  "Returns full string definition for message of type 'get_pos-request"
  (cl:format cl:nil "int64 minx~%int64 maxx~%int64 miny~%int64 maxy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_pos-request>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_pos-request>))
  "Converts a ROS message object to a list"
  (cl:list 'get_pos-request
    (cl:cons ':minx (minx msg))
    (cl:cons ':maxx (maxx msg))
    (cl:cons ':miny (miny msg))
    (cl:cons ':maxy (maxy msg))
))
;//! \htmlinclude get_pos-response.msg.html

(cl:defclass <get_pos-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0))
)

(cl:defclass get_pos-response (<get_pos-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_pos-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_pos-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name assignment_1-srv:<get_pos-response> is deprecated: use assignment_1-srv:get_pos-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <get_pos-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment_1-srv:x-val is deprecated.  Use assignment_1-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <get_pos-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader assignment_1-srv:y-val is deprecated.  Use assignment_1-srv:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_pos-response>) ostream)
  "Serializes a message object of type '<get_pos-response>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_pos-response>) istream)
  "Deserializes a message object of type '<get_pos-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_pos-response>)))
  "Returns string type for a service object of type '<get_pos-response>"
  "assignment_1/get_posResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_pos-response)))
  "Returns string type for a service object of type 'get_pos-response"
  "assignment_1/get_posResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_pos-response>)))
  "Returns md5sum for a message object of type '<get_pos-response>"
  "689b3c9d7a6b02bd3b9a51ce8fe45a3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_pos-response)))
  "Returns md5sum for a message object of type 'get_pos-response"
  "689b3c9d7a6b02bd3b9a51ce8fe45a3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_pos-response>)))
  "Returns full string definition for message of type '<get_pos-response>"
  (cl:format cl:nil "int64 x~%int64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_pos-response)))
  "Returns full string definition for message of type 'get_pos-response"
  (cl:format cl:nil "int64 x~%int64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_pos-response>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_pos-response>))
  "Converts a ROS message object to a list"
  (cl:list 'get_pos-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'get_pos)))
  'get_pos-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'get_pos)))
  'get_pos-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_pos)))
  "Returns string type for a service object of type '<get_pos>"
  "assignment_1/get_pos")