; Auto-generated. Do not edit!


(cl:in-package asl_turtlebot-msg)


;//! \htmlinclude DetectedObjectList.msg.html

(cl:defclass <DetectedObjectList> (roslisp-msg-protocol:ros-message)
  ((objects
    :reader objects
    :initarg :objects
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (ob_msgs
    :reader ob_msgs
    :initarg :ob_msgs
    :type (cl:vector asl_turtlebot-msg:DetectedObject)
   :initform (cl:make-array 0 :element-type 'asl_turtlebot-msg:DetectedObject :initial-element (cl:make-instance 'asl_turtlebot-msg:DetectedObject))))
)

(cl:defclass DetectedObjectList (<DetectedObjectList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectedObjectList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectedObjectList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name asl_turtlebot-msg:<DetectedObjectList> is deprecated: use asl_turtlebot-msg:DetectedObjectList instead.")))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <DetectedObjectList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader asl_turtlebot-msg:objects-val is deprecated.  Use asl_turtlebot-msg:objects instead.")
  (objects m))

(cl:ensure-generic-function 'ob_msgs-val :lambda-list '(m))
(cl:defmethod ob_msgs-val ((m <DetectedObjectList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader asl_turtlebot-msg:ob_msgs-val is deprecated.  Use asl_turtlebot-msg:ob_msgs instead.")
  (ob_msgs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectedObjectList>) ostream)
  "Serializes a message object of type '<DetectedObjectList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'objects))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ob_msgs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'ob_msgs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectedObjectList>) istream)
  "Deserializes a message object of type '<DetectedObjectList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ob_msgs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ob_msgs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'asl_turtlebot-msg:DetectedObject))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectedObjectList>)))
  "Returns string type for a message object of type '<DetectedObjectList>"
  "asl_turtlebot/DetectedObjectList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectedObjectList)))
  "Returns string type for a message object of type 'DetectedObjectList"
  "asl_turtlebot/DetectedObjectList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectedObjectList>)))
  "Returns md5sum for a message object of type '<DetectedObjectList>"
  "b55ad56e2dd8e9c68837a2cd0b12032a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectedObjectList)))
  "Returns md5sum for a message object of type 'DetectedObjectList"
  "b55ad56e2dd8e9c68837a2cd0b12032a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectedObjectList>)))
  "Returns full string definition for message of type '<DetectedObjectList>"
  (cl:format cl:nil "string[] objects~%DetectedObject[] ob_msgs~%================================================================================~%MSG: asl_turtlebot/DetectedObject~%uint32 id~%string name~%float64 confidence~%float64 distance~%float64 thetaleft~%float64 thetaright~%float64[] corners~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectedObjectList)))
  "Returns full string definition for message of type 'DetectedObjectList"
  (cl:format cl:nil "string[] objects~%DetectedObject[] ob_msgs~%================================================================================~%MSG: asl_turtlebot/DetectedObject~%uint32 id~%string name~%float64 confidence~%float64 distance~%float64 thetaleft~%float64 thetaright~%float64[] corners~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectedObjectList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ob_msgs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectedObjectList>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectedObjectList
    (cl:cons ':objects (objects msg))
    (cl:cons ':ob_msgs (ob_msgs msg))
))
