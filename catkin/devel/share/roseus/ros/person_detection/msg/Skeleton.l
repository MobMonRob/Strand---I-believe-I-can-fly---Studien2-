;; Auto-generated. Do not edit!


(when (boundp 'person_detection::Skeleton)
  (if (not (find-package "PERSON_DETECTION"))
    (make-package "PERSON_DETECTION"))
  (shadow 'Skeleton (find-package "PERSON_DETECTION")))
(unless (find-package "PERSON_DETECTION::SKELETON")
  (make-package "PERSON_DETECTION::SKELETON"))

(in-package "ROS")
;;//! \htmlinclude Skeleton.msg.html


(defclass person_detection::Skeleton
  :super ros::object
  :slots (_keypoints ))

(defmethod person_detection::Skeleton
  (:init
   (&key
    ((:keypoints __keypoints) (let (r) (dotimes (i 0) (push (instance person_detection::Keypoint :init) r)) r))
    )
   (send-super :init)
   (setq _keypoints __keypoints)
   self)
  (:keypoints
   (&rest __keypoints)
   (if (keywordp (car __keypoints))
       (send* _keypoints __keypoints)
     (progn
       (if __keypoints (setq _keypoints (car __keypoints)))
       _keypoints)))
  (:serialization-length
   ()
   (+
    ;; person_detection/Keypoint[] _keypoints
    (apply #'+ (send-all _keypoints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; person_detection/Keypoint[] _keypoints
     (write-long (length _keypoints) s)
     (dolist (elem _keypoints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; person_detection/Keypoint[] _keypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _keypoints (let (r) (dotimes (i n) (push (instance person_detection::Keypoint :init) r)) r))
     (dolist (elem- _keypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get person_detection::Skeleton :md5sum-) "be627c8ae26f21bd9a2b064739c579b3")
(setf (get person_detection::Skeleton :datatype-) "person_detection/Skeleton")
(setf (get person_detection::Skeleton :definition-)
      "Keypoint[] keypoints
================================================================================
MSG: person_detection/Keypoint
string name
int16 x
int16 y
float32 accuracy
")



(provide :person_detection/Skeleton "be627c8ae26f21bd9a2b064739c579b3")


