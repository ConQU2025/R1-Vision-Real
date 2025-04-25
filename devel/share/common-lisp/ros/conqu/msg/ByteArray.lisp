; Auto-generated. Do not edit!


(cl:in-package conqu-msg)


;//! \htmlinclude ByteArray.msg.html

(cl:defclass <ByteArray> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 10 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass ByteArray (<ByteArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ByteArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ByteArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name conqu-msg:<ByteArray> is deprecated: use conqu-msg:ByteArray instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ByteArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader conqu-msg:data-val is deprecated.  Use conqu-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ByteArray>) ostream)
  "Serializes a message object of type '<ByteArray>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ByteArray>) istream)
  "Deserializes a message object of type '<ByteArray>"
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 10))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 10)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ByteArray>)))
  "Returns string type for a message object of type '<ByteArray>"
  "conqu/ByteArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ByteArray)))
  "Returns string type for a message object of type 'ByteArray"
  "conqu/ByteArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ByteArray>)))
  "Returns md5sum for a message object of type '<ByteArray>"
  "56792dc2ab08b5a97efe471984a66192")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ByteArray)))
  "Returns md5sum for a message object of type 'ByteArray"
  "56792dc2ab08b5a97efe471984a66192")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ByteArray>)))
  "Returns full string definition for message of type '<ByteArray>"
  (cl:format cl:nil "int8[10] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ByteArray)))
  "Returns full string definition for message of type 'ByteArray"
  (cl:format cl:nil "int8[10] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ByteArray>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ByteArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ByteArray
    (cl:cons ':data (data msg))
))
