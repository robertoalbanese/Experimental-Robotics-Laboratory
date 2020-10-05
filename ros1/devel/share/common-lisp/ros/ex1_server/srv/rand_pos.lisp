; Auto-generated. Do not edit!


(cl:in-package ex1_server-srv)


;//! \htmlinclude rand_pos-request.msg.html

(cl:defclass <rand_pos-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass rand_pos-request (<rand_pos-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rand_pos-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rand_pos-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ex1_server-srv:<rand_pos-request> is deprecated: use ex1_server-srv:rand_pos-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rand_pos-request>) ostream)
  "Serializes a message object of type '<rand_pos-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rand_pos-request>) istream)
  "Deserializes a message object of type '<rand_pos-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rand_pos-request>)))
  "Returns string type for a service object of type '<rand_pos-request>"
  "ex1_server/rand_posRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rand_pos-request)))
  "Returns string type for a service object of type 'rand_pos-request"
  "ex1_server/rand_posRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rand_pos-request>)))
  "Returns md5sum for a message object of type '<rand_pos-request>"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rand_pos-request)))
  "Returns md5sum for a message object of type 'rand_pos-request"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rand_pos-request>)))
  "Returns full string definition for message of type '<rand_pos-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rand_pos-request)))
  "Returns full string definition for message of type 'rand_pos-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rand_pos-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rand_pos-request>))
  "Converts a ROS message object to a list"
  (cl:list 'rand_pos-request
))
;//! \htmlinclude rand_pos-response.msg.html

(cl:defclass <rand_pos-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass rand_pos-response (<rand_pos-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rand_pos-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rand_pos-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ex1_server-srv:<rand_pos-response> is deprecated: use ex1_server-srv:rand_pos-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <rand_pos-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ex1_server-srv:x-val is deprecated.  Use ex1_server-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <rand_pos-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ex1_server-srv:y-val is deprecated.  Use ex1_server-srv:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rand_pos-response>) ostream)
  "Serializes a message object of type '<rand_pos-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rand_pos-response>) istream)
  "Deserializes a message object of type '<rand_pos-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rand_pos-response>)))
  "Returns string type for a service object of type '<rand_pos-response>"
  "ex1_server/rand_posResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rand_pos-response)))
  "Returns string type for a service object of type 'rand_pos-response"
  "ex1_server/rand_posResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rand_pos-response>)))
  "Returns md5sum for a message object of type '<rand_pos-response>"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rand_pos-response)))
  "Returns md5sum for a message object of type 'rand_pos-response"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rand_pos-response>)))
  "Returns full string definition for message of type '<rand_pos-response>"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rand_pos-response)))
  "Returns full string definition for message of type 'rand_pos-response"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rand_pos-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rand_pos-response>))
  "Converts a ROS message object to a list"
  (cl:list 'rand_pos-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'rand_pos)))
  'rand_pos-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'rand_pos)))
  'rand_pos-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rand_pos)))
  "Returns string type for a service object of type '<rand_pos>"
  "ex1_server/rand_pos")