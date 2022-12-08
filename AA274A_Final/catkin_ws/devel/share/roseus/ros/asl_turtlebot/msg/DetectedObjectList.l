;; Auto-generated. Do not edit!


(when (boundp 'asl_turtlebot::DetectedObjectList)
  (if (not (find-package "ASL_TURTLEBOT"))
    (make-package "ASL_TURTLEBOT"))
  (shadow 'DetectedObjectList (find-package "ASL_TURTLEBOT")))
(unless (find-package "ASL_TURTLEBOT::DETECTEDOBJECTLIST")
  (make-package "ASL_TURTLEBOT::DETECTEDOBJECTLIST"))

(in-package "ROS")
;;//! \htmlinclude DetectedObjectList.msg.html


(defclass asl_turtlebot::DetectedObjectList
  :super ros::object
  :slots (_objects _ob_msgs ))

(defmethod asl_turtlebot::DetectedObjectList
  (:init
   (&key
    ((:objects __objects) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:ob_msgs __ob_msgs) ())
    )
   (send-super :init)
   (setq _objects __objects)
   (setq _ob_msgs __ob_msgs)
   self)
  (:objects
   (&optional __objects)
   (if __objects (setq _objects __objects)) _objects)
  (:ob_msgs
   (&rest __ob_msgs)
   (if (keywordp (car __ob_msgs))
       (send* _ob_msgs __ob_msgs)
     (progn
       (if __ob_msgs (setq _ob_msgs (car __ob_msgs)))
       _ob_msgs)))
  (:serialization-length
   ()
   (+
    ;; string[] _objects
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _objects)) 4
    ;; asl_turtlebot/DetectedObject[] _ob_msgs
    (apply #'+ (send-all _ob_msgs :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
       (write-long (length elem) s) (princ elem s)
       )
     ;; asl_turtlebot/DetectedObject[] _ob_msgs
     (write-long (length _ob_msgs) s)
     (dolist (elem _ob_msgs)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _objects i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; asl_turtlebot/DetectedObject[] _ob_msgs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ob_msgs (let (r) (dotimes (i n) (push (instance asl_turtlebot::DetectedObject :init) r)) r))
     (dolist (elem- _ob_msgs)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get asl_turtlebot::DetectedObjectList :md5sum-) "b55ad56e2dd8e9c68837a2cd0b12032a")
(setf (get asl_turtlebot::DetectedObjectList :datatype-) "asl_turtlebot/DetectedObjectList")
(setf (get asl_turtlebot::DetectedObjectList :definition-)
      "string[] objects
DetectedObject[] ob_msgs
================================================================================
MSG: asl_turtlebot/DetectedObject
uint32 id
string name
float64 confidence
float64 distance
float64 thetaleft
float64 thetaright
float64[] corners

")



(provide :asl_turtlebot/DetectedObjectList "b55ad56e2dd8e9c68837a2cd0b12032a")


