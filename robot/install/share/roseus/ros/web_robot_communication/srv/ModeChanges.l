;; Auto-generated. Do not edit!


(when (boundp 'web_robot_communication::ModeChanges)
  (if (not (find-package "WEB_ROBOT_COMMUNICATION"))
    (make-package "WEB_ROBOT_COMMUNICATION"))
  (shadow 'ModeChanges (find-package "WEB_ROBOT_COMMUNICATION")))
(unless (find-package "WEB_ROBOT_COMMUNICATION::MODECHANGES")
  (make-package "WEB_ROBOT_COMMUNICATION::MODECHANGES"))
(unless (find-package "WEB_ROBOT_COMMUNICATION::MODECHANGESREQUEST")
  (make-package "WEB_ROBOT_COMMUNICATION::MODECHANGESREQUEST"))
(unless (find-package "WEB_ROBOT_COMMUNICATION::MODECHANGESRESPONSE")
  (make-package "WEB_ROBOT_COMMUNICATION::MODECHANGESRESPONSE"))

(in-package "ROS")





(defclass web_robot_communication::ModeChangesRequest
  :super ros::object
  :slots (_mode ))

(defmethod web_robot_communication::ModeChangesRequest
  (:init
   (&key
    ((:mode __mode) "")
    )
   (send-super :init)
   (setq _mode (string __mode))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; string _mode
    4 (length _mode)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _mode
       (write-long (length _mode) s) (princ _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass web_robot_communication::ModeChangesResponse
  :super ros::object
  :slots (_message ))

(defmethod web_robot_communication::ModeChangesResponse
  (:init
   (&key
    ((:message __message) "")
    )
   (send-super :init)
   (setq _message (string __message))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass web_robot_communication::ModeChanges
  :super ros::object
  :slots ())

(setf (get web_robot_communication::ModeChanges :md5sum-) "4f7a1d450bc2c8872a872f31f56ded53")
(setf (get web_robot_communication::ModeChanges :datatype-) "web_robot_communication/ModeChanges")
(setf (get web_robot_communication::ModeChanges :request) web_robot_communication::ModeChangesRequest)
(setf (get web_robot_communication::ModeChanges :response) web_robot_communication::ModeChangesResponse)

(defmethod web_robot_communication::ModeChangesRequest
  (:response () (instance web_robot_communication::ModeChangesResponse :init)))

(setf (get web_robot_communication::ModeChangesRequest :md5sum-) "4f7a1d450bc2c8872a872f31f56ded53")
(setf (get web_robot_communication::ModeChangesRequest :datatype-) "web_robot_communication/ModeChangesRequest")
(setf (get web_robot_communication::ModeChangesRequest :definition-)
      "string mode
---
string message
")

(setf (get web_robot_communication::ModeChangesResponse :md5sum-) "4f7a1d450bc2c8872a872f31f56ded53")
(setf (get web_robot_communication::ModeChangesResponse :datatype-) "web_robot_communication/ModeChangesResponse")
(setf (get web_robot_communication::ModeChangesResponse :definition-)
      "string mode
---
string message
")



(provide :web_robot_communication/ModeChanges "4f7a1d450bc2c8872a872f31f56ded53")


