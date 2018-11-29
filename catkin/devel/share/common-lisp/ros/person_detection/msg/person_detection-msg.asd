
(cl:in-package :asdf)

(defsystem "person_detection-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Keypoint" :depends-on ("_package_Keypoint"))
    (:file "_package_Keypoint" :depends-on ("_package"))
    (:file "Skeleton" :depends-on ("_package_Skeleton"))
    (:file "_package_Skeleton" :depends-on ("_package"))
  ))