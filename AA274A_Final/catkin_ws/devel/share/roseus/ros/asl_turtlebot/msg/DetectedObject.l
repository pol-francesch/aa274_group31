;; Auto-generated. Do not edit!


(when (boundp 'asl_turtlebot::DetectedObject)
  (if (not (find-package "ASL_TURTLEBOT"))
    (make-package "ASL_TURTLEBOT"))
  (shadow 'DetectedObject (find-package "ASL_TURTLEBOT")))
(unless (find-package "ASL_TURTLEBOT::DETECTEDOBJECT")
  (make-package "ASL_TURTLEBOT::DETECTEDOBJECT"))

(in-package "ROS")
;;//! \htmlinclude DetectedObject.msg.html


(defclass asl_turtlebot::DetectedObject
  :super ros::object
  :slots (_id _name _confidence _distance _thetaleft _thetaright _corners ))

(defmethod asl_turtlebot::DetectedObject
  (:init
   (&key
    ((:id __id) 0)
    ((:name __name) "")
    ((:confidence __confidence) 0.0)
    ((:distance __distance) 0.0)
    ((:thetaleft __thetaleft) 0.0)
    ((:thetaright __thetaright) 0.0)
    ((:corners __corners) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _name (string __name))
   (setq _confidence (float __confidence))
   (setq _distance (float __distance))
   (setq _thetaleft (float __thetaleft))
   (setq _thetaright (float __thetaright))
   (setq _corners __corners)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:thetaleft
   (&optional __thetaleft)
   (if __thetaleft (setq _thetaleft __thetaleft)) _thetaleft)
  (:thetaright
   (&optional __thetaright)
   (if __thetaright (setq _thetaright __thetaright)) _thetaright)
  (:corners
   (&optional __corners)
   (if __corners (setq _corners __corners)) _corners)
  (:serialization-length
   ()
   (+
    ;; uint32 _id
    4
    ;; string _name
    4 (length _name)
    ;; float64 _confidence
    8
    ;; float64 _distance
    8
    ;; float64 _thetaleft
    8
    ;; float64 _thetaright
    8
    ;; float64[] _corners
    (* 8    (length _corners)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _id
       (write-long _id s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _thetaleft
       (sys::poke _thetaleft (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _thetaright
       (sys::poke _thetaright (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _corners
     (write-long (length _corners) s)
     (dotimes (i (length _corners))
       (sys::poke (elt _corners i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _confidence
     (setq _confidence (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _distance
     (setq _distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _thetaleft
     (setq _thetaleft (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _thetaright
     (setq _thetaright (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _corners
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _corners (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _corners i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get asl_turtlebot::DetectedObject :md5sum-) "2650e98dd260a5b8590eced757507d05")
(setf (get asl_turtlebot::DetectedObject :datatype-) "asl_turtlebot/DetectedObject")
(setf (get asl_turtlebot::DetectedObject :definition-)
      "uint32 id
string name
float64 confidence
float64 distance
float64 thetaleft
float64 thetaright
float64[] corners

")



(provide :asl_turtlebot/DetectedObject "2650e98dd260a5b8590eced757507d05")


