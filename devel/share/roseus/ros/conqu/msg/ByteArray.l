;; Auto-generated. Do not edit!


(when (boundp 'conqu::ByteArray)
  (if (not (find-package "CONQU"))
    (make-package "CONQU"))
  (shadow 'ByteArray (find-package "CONQU")))
(unless (find-package "CONQU::BYTEARRAY")
  (make-package "CONQU::BYTEARRAY"))

(in-package "ROS")
;;//! \htmlinclude ByteArray.msg.html


(defclass conqu::ByteArray
  :super ros::object
  :slots (_data ))

(defmethod conqu::ByteArray
  (:init
   (&key
    ((:data __data) (make-array 10 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int8[10] _data
    (* 1    10)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8[10] _data
     (dotimes (i 10)
       (write-byte (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8[10] _data
   (dotimes (i (length _data))
     (setf (elt _data i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _data i) 127) (setf (elt _data i) (- (elt _data i) 256)))
     )
   ;;
   self)
  )

(setf (get conqu::ByteArray :md5sum-) "56792dc2ab08b5a97efe471984a66192")
(setf (get conqu::ByteArray :datatype-) "conqu/ByteArray")
(setf (get conqu::ByteArray :definition-)
      "int8[10] data
")



(provide :conqu/ByteArray "56792dc2ab08b5a97efe471984a66192")


