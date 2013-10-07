; Auto-generated. Do not edit!


(cl:in-package CodyLegoBot-msg)


;//! \htmlinclude commandsend.msg.html

(cl:defclass <commandsend> (roslisp-msg-protocol:ros-message)
  ((cds
    :reader cds
    :initarg :cds
    :type cl:string
    :initform ""))
)

(cl:defclass commandsend (<commandsend>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <commandsend>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'commandsend)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name CodyLegoBot-msg:<commandsend> is deprecated: use CodyLegoBot-msg:commandsend instead.")))

(cl:ensure-generic-function 'cds-val :lambda-list '(m))
(cl:defmethod cds-val ((m <commandsend>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader CodyLegoBot-msg:cds-val is deprecated.  Use CodyLegoBot-msg:cds instead.")
  (cds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <commandsend>) ostream)
  "Serializes a message object of type '<commandsend>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cds))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <commandsend>) istream)
  "Deserializes a message object of type '<commandsend>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cds) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cds) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<commandsend>)))
  "Returns string type for a message object of type '<commandsend>"
  "CodyLegoBot/commandsend")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'commandsend)))
  "Returns string type for a message object of type 'commandsend"
  "CodyLegoBot/commandsend")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<commandsend>)))
  "Returns md5sum for a message object of type '<commandsend>"
  "0753234295f3099569ee5ddb4e7f2d72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'commandsend)))
  "Returns md5sum for a message object of type 'commandsend"
  "0753234295f3099569ee5ddb4e7f2d72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<commandsend>)))
  "Returns full string definition for message of type '<commandsend>"
  (cl:format cl:nil "string cds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'commandsend)))
  "Returns full string definition for message of type 'commandsend"
  (cl:format cl:nil "string cds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <commandsend>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'cds))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <commandsend>))
  "Converts a ROS message object to a list"
  (cl:list 'commandsend
    (cl:cons ':cds (cds msg))
))
