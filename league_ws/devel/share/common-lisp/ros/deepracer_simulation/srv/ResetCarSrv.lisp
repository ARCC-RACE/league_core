; Auto-generated. Do not edit!


(cl:in-package deepracer_simulation-srv)


;//! \htmlinclude ResetCarSrv-request.msg.html

(cl:defclass <ResetCarSrv-request> (roslisp-msg-protocol:ros-message)
  ((ndist
    :reader ndist
    :initarg :ndist
    :type cl:float
    :initform 0.0)
   (next_index
    :reader next_index
    :initarg :next_index
    :type cl:integer
    :initform 0))
)

(cl:defclass ResetCarSrv-request (<ResetCarSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetCarSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetCarSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_simulation-srv:<ResetCarSrv-request> is deprecated: use deepracer_simulation-srv:ResetCarSrv-request instead.")))

(cl:ensure-generic-function 'ndist-val :lambda-list '(m))
(cl:defmethod ndist-val ((m <ResetCarSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_simulation-srv:ndist-val is deprecated.  Use deepracer_simulation-srv:ndist instead.")
  (ndist m))

(cl:ensure-generic-function 'next_index-val :lambda-list '(m))
(cl:defmethod next_index-val ((m <ResetCarSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_simulation-srv:next_index-val is deprecated.  Use deepracer_simulation-srv:next_index instead.")
  (next_index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetCarSrv-request>) ostream)
  "Serializes a message object of type '<ResetCarSrv-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ndist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'next_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetCarSrv-request>) istream)
  "Deserializes a message object of type '<ResetCarSrv-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ndist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'next_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetCarSrv-request>)))
  "Returns string type for a service object of type '<ResetCarSrv-request>"
  "deepracer_simulation/ResetCarSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetCarSrv-request)))
  "Returns string type for a service object of type 'ResetCarSrv-request"
  "deepracer_simulation/ResetCarSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetCarSrv-request>)))
  "Returns md5sum for a message object of type '<ResetCarSrv-request>"
  "51459ac9873b166b6314083bf1771edb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetCarSrv-request)))
  "Returns md5sum for a message object of type 'ResetCarSrv-request"
  "51459ac9873b166b6314083bf1771edb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetCarSrv-request>)))
  "Returns full string definition for message of type '<ResetCarSrv-request>"
  (cl:format cl:nil "float32 ndist~%int32 next_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetCarSrv-request)))
  "Returns full string definition for message of type 'ResetCarSrv-request"
  (cl:format cl:nil "float32 ndist~%int32 next_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetCarSrv-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetCarSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetCarSrv-request
    (cl:cons ':ndist (ndist msg))
    (cl:cons ':next_index (next_index msg))
))
;//! \htmlinclude ResetCarSrv-response.msg.html

(cl:defclass <ResetCarSrv-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0))
)

(cl:defclass ResetCarSrv-response (<ResetCarSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetCarSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetCarSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_simulation-srv:<ResetCarSrv-response> is deprecated: use deepracer_simulation-srv:ResetCarSrv-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ResetCarSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_simulation-srv:success-val is deprecated.  Use deepracer_simulation-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetCarSrv-response>) ostream)
  "Serializes a message object of type '<ResetCarSrv-response>"
  (cl:let* ((signed (cl:slot-value msg 'success)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetCarSrv-response>) istream)
  "Deserializes a message object of type '<ResetCarSrv-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'success) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetCarSrv-response>)))
  "Returns string type for a service object of type '<ResetCarSrv-response>"
  "deepracer_simulation/ResetCarSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetCarSrv-response)))
  "Returns string type for a service object of type 'ResetCarSrv-response"
  "deepracer_simulation/ResetCarSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetCarSrv-response>)))
  "Returns md5sum for a message object of type '<ResetCarSrv-response>"
  "51459ac9873b166b6314083bf1771edb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetCarSrv-response)))
  "Returns md5sum for a message object of type 'ResetCarSrv-response"
  "51459ac9873b166b6314083bf1771edb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetCarSrv-response>)))
  "Returns full string definition for message of type '<ResetCarSrv-response>"
  (cl:format cl:nil "int32 success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetCarSrv-response)))
  "Returns full string definition for message of type 'ResetCarSrv-response"
  (cl:format cl:nil "int32 success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetCarSrv-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetCarSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetCarSrv-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetCarSrv)))
  'ResetCarSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetCarSrv)))
  'ResetCarSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetCarSrv)))
  "Returns string type for a service object of type '<ResetCarSrv>"
  "deepracer_simulation/ResetCarSrv")