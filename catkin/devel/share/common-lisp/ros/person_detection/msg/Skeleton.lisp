; Auto-generated. Do not edit!


(cl:in-package person_detection-msg)


;//! \htmlinclude Skeleton.msg.html

(cl:defclass <Skeleton> (roslisp-msg-protocol:ros-message)
  ((keypoints
    :reader keypoints
    :initarg :keypoints
    :type (cl:vector person_detection-msg:Keypoint)
   :initform (cl:make-array 0 :element-type 'person_detection-msg:Keypoint :initial-element (cl:make-instance 'person_detection-msg:Keypoint))))
)

(cl:defclass Skeleton (<Skeleton>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Skeleton>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Skeleton)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name person_detection-msg:<Skeleton> is deprecated: use person_detection-msg:Skeleton instead.")))

(cl:ensure-generic-function 'keypoints-val :lambda-list '(m))
(cl:defmethod keypoints-val ((m <Skeleton>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader person_detection-msg:keypoints-val is deprecated.  Use person_detection-msg:keypoints instead.")
  (keypoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Skeleton>) ostream)
  "Serializes a message object of type '<Skeleton>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'keypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'keypoints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Skeleton>) istream)
  "Deserializes a message object of type '<Skeleton>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'keypoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'keypoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'person_detection-msg:Keypoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Skeleton>)))
  "Returns string type for a message object of type '<Skeleton>"
  "person_detection/Skeleton")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Skeleton)))
  "Returns string type for a message object of type 'Skeleton"
  "person_detection/Skeleton")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Skeleton>)))
  "Returns md5sum for a message object of type '<Skeleton>"
  "be627c8ae26f21bd9a2b064739c579b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Skeleton)))
  "Returns md5sum for a message object of type 'Skeleton"
  "be627c8ae26f21bd9a2b064739c579b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Skeleton>)))
  "Returns full string definition for message of type '<Skeleton>"
  (cl:format cl:nil "Keypoint[] keypoints~%================================================================================~%MSG: person_detection/Keypoint~%string name~%int16 x~%int16 y~%float32 accuracy~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Skeleton)))
  "Returns full string definition for message of type 'Skeleton"
  (cl:format cl:nil "Keypoint[] keypoints~%================================================================================~%MSG: person_detection/Keypoint~%string name~%int16 x~%int16 y~%float32 accuracy~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Skeleton>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'keypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Skeleton>))
  "Converts a ROS message object to a list"
  (cl:list 'Skeleton
    (cl:cons ':keypoints (keypoints msg))
))
