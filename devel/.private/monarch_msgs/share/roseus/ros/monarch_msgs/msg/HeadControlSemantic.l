;; Auto-generated. Do not edit!


(when (boundp 'monarch_msgs::HeadControlSemantic)
  (if (not (find-package "MONARCH_MSGS"))
    (make-package "MONARCH_MSGS"))
  (shadow 'HeadControlSemantic (find-package "MONARCH_MSGS")))
(unless (find-package "MONARCH_MSGS::HEADCONTROLSEMANTIC")
  (make-package "MONARCH_MSGS::HEADCONTROLSEMANTIC"))

(in-package "ROS")
;;//! \htmlinclude HeadControlSemantic.msg.html


(intern "*NORMAL*" (find-package "MONARCH_MSGS::HEADCONTROLSEMANTIC"))
(shadow '*NORMAL* (find-package "MONARCH_MSGS::HEADCONTROLSEMANTIC"))
(defconstant monarch_msgs::HeadControlSemantic::*NORMAL* 0)
(intern "*SLOW*" (find-package "MONARCH_MSGS::HEADCONTROLSEMANTIC"))
(shadow '*SLOW* (find-package "MONARCH_MSGS::HEADCONTROLSEMANTIC"))
(defconstant monarch_msgs::HeadControlSemantic::*SLOW* 1)
(intern "*FAST*" (find-package "MONARCH_MSGS::HEADCONTROLSEMANTIC"))
(shadow '*FAST* (find-package "MONARCH_MSGS::HEADCONTROLSEMANTIC"))
(defconstant monarch_msgs::HeadControlSemantic::*FAST* 2)
(defclass monarch_msgs::HeadControlSemantic
  :super ros::object
  :slots (_cardinal_direction _speed ))

(defmethod monarch_msgs::HeadControlSemantic
  (:init
   (&key
    ((:cardinal_direction __cardinal_direction) "")
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _cardinal_direction (string __cardinal_direction))
   (setq _speed (round __speed))
   self)
  (:cardinal_direction
   (&optional __cardinal_direction)
   (if __cardinal_direction (setq _cardinal_direction __cardinal_direction)) _cardinal_direction)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; string _cardinal_direction
    4 (length _cardinal_direction)
    ;; uint8 _speed
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _cardinal_direction
       (write-long (length _cardinal_direction) s) (princ _cardinal_direction s)
     ;; uint8 _speed
       (write-byte _speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _cardinal_direction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cardinal_direction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _speed
     (setq _speed (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get monarch_msgs::HeadControlSemantic :md5sum-) "a8f7a062da2d04ec1d87e1a59f64f6af")
(setf (get monarch_msgs::HeadControlSemantic :datatype-) "monarch_msgs/HeadControlSemantic")
(setf (get monarch_msgs::HeadControlSemantic :definition-)
      "# If you add more, please do not break the current definitions
uint8 NORMAL=0
uint8 SLOW=1
uint8 FAST=2

# Example for NW, accepts:
# {NW, NORTHEAST}
string cardinal_direction
uint8 speed

")



(provide :monarch_msgs/HeadControlSemantic "a8f7a062da2d04ec1d87e1a59f64f6af")


