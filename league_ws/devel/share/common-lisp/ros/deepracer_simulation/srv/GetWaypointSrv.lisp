; Auto-generated. Do not edit!


(cl:in-package deepracer_simulation-srv)


;//! \htmlinclude GetWaypointSrv-request.msg.html

(cl:defclass <GetWaypointSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetWaypointSrv-request (<GetWaypointSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWaypointSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWaypointSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_simulation-srv:<GetWaypointSrv-request> is deprecated: use deepracer_simulation-srv:GetWaypointSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWaypointSrv-request>) ostream)
  "Serializes a message object of type '<GetWaypointSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWaypointSrv-request>) istream)
  "Deserializes a message object of type '<GetWaypointSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWaypointSrv-request>)))
  "Returns string type for a service object of type '<GetWaypointSrv-request>"
  "deepracer_simulation/GetWaypointSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWaypointSrv-request)))
  "Returns string type for a service object of type 'GetWaypointSrv-request"
  "deepracer_simulation/GetWaypointSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWaypointSrv-request>)))
  "Returns md5sum for a message object of type '<GetWaypointSrv-request>"
  "dd1c3c0f312afb554365a4b5e8f07a10")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWaypointSrv-request)))
  "Returns md5sum for a message object of type 'GetWaypointSrv-request"
  "dd1c3c0f312afb554365a4b5e8f07a10")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWaypointSrv-request>)))
  "Returns full string definition for message of type '<GetWaypointSrv-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWaypointSrv-request)))
  "Returns full string definition for message of type 'GetWaypointSrv-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWaypointSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWaypointSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWaypointSrv-request
))
;//! \htmlinclude GetWaypointSrv-response.msg.html

(cl:defclass <GetWaypointSrv-response> (roslisp-msg-protocol:ros-message)
  ((row
    :reader row
    :initarg :row
    :type cl:integer
    :initform 0)
   (col
    :reader col
    :initarg :col
    :type cl:integer
    :initform 0)
   (waypoints
    :reader waypoints
    :initarg :waypoints
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GetWaypointSrv-response (<GetWaypointSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetWaypointSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetWaypointSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name deepracer_simulation-srv:<GetWaypointSrv-response> is deprecated: use deepracer_simulation-srv:GetWaypointSrv-response instead.")))

(cl:ensure-generic-function 'row-val :lambda-list '(m))
(cl:defmethod row-val ((m <GetWaypointSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_simulation-srv:row-val is deprecated.  Use deepracer_simulation-srv:row instead.")
  (row m))

(cl:ensure-generic-function 'col-val :lambda-list '(m))
(cl:defmethod col-val ((m <GetWaypointSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_simulation-srv:col-val is deprecated.  Use deepracer_simulation-srv:col instead.")
  (col m))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <GetWaypointSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader deepracer_simulation-srv:waypoints-val is deprecated.  Use deepracer_simulation-srv:waypoints instead.")
  (waypoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetWaypointSrv-response>) ostream)
  "Serializes a message object of type '<GetWaypointSrv-response>"
  (cl:let* ((signed (cl:slot-value msg 'row)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'col)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'waypoints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetWaypointSrv-response>) istream)
  "Deserializes a message object of type '<GetWaypointSrv-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'row) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'col) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'waypoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'waypoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetWaypointSrv-response>)))
  "Returns string type for a service object of type '<GetWaypointSrv-response>"
  "deepracer_simulation/GetWaypointSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWaypointSrv-response)))
  "Returns string type for a service object of type 'GetWaypointSrv-response"
  "deepracer_simulation/GetWaypointSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetWaypointSrv-response>)))
  "Returns md5sum for a message object of type '<GetWaypointSrv-response>"
  "dd1c3c0f312afb554365a4b5e8f07a10")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetWaypointSrv-response)))
  "Returns md5sum for a message object of type 'GetWaypointSrv-response"
  "dd1c3c0f312afb554365a4b5e8f07a10")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetWaypointSrv-response>)))
  "Returns full string definition for message of type '<GetWaypointSrv-response>"
  (cl:format cl:nil "int32 row~%int32 col~%float64[] waypoints~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetWaypointSrv-response)))
  "Returns full string definition for message of type 'GetWaypointSrv-response"
  (cl:format cl:nil "int32 row~%int32 col~%float64[] waypoints~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetWaypointSrv-response>))
  (cl:+ 0
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetWaypointSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetWaypointSrv-response
    (cl:cons ':row (row msg))
    (cl:cons ':col (col msg))
    (cl:cons ':waypoints (waypoints msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetWaypointSrv)))
  'GetWaypointSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetWaypointSrv)))
  'GetWaypointSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetWaypointSrv)))
  "Returns string type for a service object of type '<GetWaypointSrv>"
  "deepracer_simulation/GetWaypointSrv")