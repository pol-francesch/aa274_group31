;; Auto-generated. Do not edit!


(when (boundp 'aa274a_s2::MyMessage)
  (if (not (find-package "AA274A_S2"))
    (make-package "AA274A_S2"))
  (shadow 'MyMessage (find-package "AA274A_S2")))
(unless (find-package "AA274A_S2::MYMESSAGE")
  (make-package "AA274A_S2::MYMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude MyMessage.msg.html


(defclass aa274a_s2::MyMessage
  :super ros::object
  :slots (_mic _s _f _c _idx ))

(defmethod aa274a_s2::MyMessage
  (:init
   (&key
    ((:mic __mic) nil)
    ((:s __s) "")
    ((:f __f) 0.0)
    ((:c __c) 0)
    ((:idx __idx) 0)
    )
   (send-super :init)
   (setq _mic __mic)
   (setq _s (string __s))
   (setq _f (float __f))
   (setq _c (round __c))
   (setq _idx (round __idx))
   self)
  (:mic
   (&optional (__mic :null))
   (if (not (eq __mic :null)) (setq _mic __mic)) _mic)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:f
   (&optional __f)
   (if __f (setq _f __f)) _f)
  (:c
   (&optional __c)
   (if __c (setq _c __c)) _c)
  (:idx
   (&optional __idx)
   (if __idx (setq _idx __idx)) _idx)
  (:serialization-length
   ()
   (+
    ;; bool _mic
    1
    ;; string _s
    4 (length _s)
    ;; float64 _f
    8
    ;; char _c
    1
    ;; int64 _idx
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _mic
       (if _mic (write-byte -1 s) (write-byte 0 s))
     ;; string _s
       (write-long (length _s) s) (princ _s s)
     ;; float64 _f
       (sys::poke _f (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; char _c
       (write-byte _c s)
     ;; int64 _idx
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _idx (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _idx) (= (length (_idx . bv)) 2)) ;; bignum
              (write-long (ash (elt (_idx . bv) 0) 0) s)
              (write-long (ash (elt (_idx . bv) 1) -1) s))
             ((and (class _idx) (= (length (_idx . bv)) 1)) ;; big1
              (write-long (elt (_idx . bv) 0) s)
              (write-long (if (>= _idx 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _idx s)(write-long (if (>= _idx 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _mic
     (setq _mic (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _s
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _s (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _f
     (setq _f (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; char _c
     (setq _c (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int64 _idx
#+(or :alpha :irix6 :x86_64)
      (setf _idx (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _idx (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get aa274a_s2::MyMessage :md5sum-) "5c584acd7d9101176cedf5e6e01732f5")
(setf (get aa274a_s2::MyMessage :datatype-) "aa274a_s2/MyMessage")
(setf (get aa274a_s2::MyMessage :definition-)
      "bool mic
string s
float64 f
char c
int64 idx

")



(provide :aa274a_s2/MyMessage "5c584acd7d9101176cedf5e6e01732f5")


