; Auto-generated. Do not edit!


(cl:in-package web_robot_communication-srv)


;//! \htmlinclude ModeChanges-request.msg.html

(cl:defclass <ModeChanges-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:string
    :initform ""))
)

(cl:defclass ModeChanges-request (<ModeChanges-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModeChanges-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModeChanges-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name web_robot_communication-srv:<ModeChanges-request> is deprecated: use web_robot_communication-srv:ModeChanges-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <ModeChanges-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader web_robot_communication-srv:mode-val is deprecated.  Use web_robot_communication-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModeChanges-request>) ostream)
  "Serializes a message object of type '<ModeChanges-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModeChanges-request>) istream)
  "Deserializes a message object of type '<ModeChanges-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModeChanges-request>)))
  "Returns string type for a service object of type '<ModeChanges-request>"
  "web_robot_communication/ModeChangesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModeChanges-request)))
  "Returns string type for a service object of type 'ModeChanges-request"
  "web_robot_communication/ModeChangesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModeChanges-request>)))
  "Returns md5sum for a message object of type '<ModeChanges-request>"
  "4f7a1d450bc2c8872a872f31f56ded53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModeChanges-request)))
  "Returns md5sum for a message object of type 'ModeChanges-request"
  "4f7a1d450bc2c8872a872f31f56ded53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModeChanges-request>)))
  "Returns full string definition for message of type '<ModeChanges-request>"
  (cl:format cl:nil "string mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModeChanges-request)))
  "Returns full string definition for message of type 'ModeChanges-request"
  (cl:format cl:nil "string mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModeChanges-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'mode))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModeChanges-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ModeChanges-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude ModeChanges-response.msg.html

(cl:defclass <ModeChanges-response> (roslisp-msg-protocol:ros-message)
  ((message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass ModeChanges-response (<ModeChanges-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModeChanges-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModeChanges-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name web_robot_communication-srv:<ModeChanges-response> is deprecated: use web_robot_communication-srv:ModeChanges-response instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ModeChanges-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader web_robot_communication-srv:message-val is deprecated.  Use web_robot_communication-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModeChanges-response>) ostream)
  "Serializes a message object of type '<ModeChanges-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModeChanges-response>) istream)
  "Deserializes a message object of type '<ModeChanges-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModeChanges-response>)))
  "Returns string type for a service object of type '<ModeChanges-response>"
  "web_robot_communication/ModeChangesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModeChanges-response)))
  "Returns string type for a service object of type 'ModeChanges-response"
  "web_robot_communication/ModeChangesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModeChanges-response>)))
  "Returns md5sum for a message object of type '<ModeChanges-response>"
  "4f7a1d450bc2c8872a872f31f56ded53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModeChanges-response)))
  "Returns md5sum for a message object of type 'ModeChanges-response"
  "4f7a1d450bc2c8872a872f31f56ded53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModeChanges-response>)))
  "Returns full string definition for message of type '<ModeChanges-response>"
  (cl:format cl:nil "string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModeChanges-response)))
  "Returns full string definition for message of type 'ModeChanges-response"
  (cl:format cl:nil "string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModeChanges-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModeChanges-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ModeChanges-response
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ModeChanges)))
  'ModeChanges-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ModeChanges)))
  'ModeChanges-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModeChanges)))
  "Returns string type for a service object of type '<ModeChanges>"
  "web_robot_communication/ModeChanges")