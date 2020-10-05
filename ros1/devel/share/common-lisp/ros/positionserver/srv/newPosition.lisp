; Auto-generated. Do not edit!


(cl:in-package positionserver-srv)


;//! \htmlinclude newPosition-request.msg.html

(cl:defclass <newPosition-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass newPosition-request (<newPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <newPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'newPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name positionserver-srv:<newPosition-request> is deprecated: use positionserver-srv:newPosition-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <newPosition-request>) ostream)
  "Serializes a message object of type '<newPosition-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <newPosition-request>) istream)
  "Deserializes a message object of type '<newPosition-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<newPosition-request>)))
  "Returns string type for a service object of type '<newPosition-request>"
  "positionserver/newPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'newPosition-request)))
  "Returns string type for a service object of type 'newPosition-request"
  "positionserver/newPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<newPosition-request>)))
  "Returns md5sum for a message object of type '<newPosition-request>"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'newPosition-request)))
  "Returns md5sum for a message object of type 'newPosition-request"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<newPosition-request>)))
  "Returns full string definition for message of type '<newPosition-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'newPosition-request)))
  "Returns full string definition for message of type 'newPosition-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <newPosition-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <newPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'newPosition-request
))
;//! \htmlinclude newPosition-response.msg.html

(cl:defclass <newPosition-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass newPosition-response (<newPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <newPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'newPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name positionserver-srv:<newPosition-response> is deprecated: use positionserver-srv:newPosition-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <newPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader positionserver-srv:x-val is deprecated.  Use positionserver-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <newPosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader positionserver-srv:y-val is deprecated.  Use positionserver-srv:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <newPosition-response>) ostream)
  "Serializes a message object of type '<newPosition-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <newPosition-response>) istream)
  "Deserializes a message object of type '<newPosition-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<newPosition-response>)))
  "Returns string type for a service object of type '<newPosition-response>"
  "positionserver/newPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'newPosition-response)))
  "Returns string type for a service object of type 'newPosition-response"
  "positionserver/newPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<newPosition-response>)))
  "Returns md5sum for a message object of type '<newPosition-response>"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'newPosition-response)))
  "Returns md5sum for a message object of type 'newPosition-response"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<newPosition-response>)))
  "Returns full string definition for message of type '<newPosition-response>"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'newPosition-response)))
  "Returns full string definition for message of type 'newPosition-response"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <newPosition-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <newPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'newPosition-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'newPosition)))
  'newPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'newPosition)))
  'newPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'newPosition)))
  "Returns string type for a service object of type '<newPosition>"
  "positionserver/newPosition")