; Auto-generated. Do not edit!


(in-package pmad-msg)


;//! \htmlinclude Status.msg.html

(defclass <Status> (ros-message)
  ((analog_0
    :reader analog_0-val
    :initarg :analog_0
    :type integer
    :initform 0)
   (analog_1
    :reader analog_1-val
    :initarg :analog_1
    :type integer
    :initform 0)
   (analog_2
    :reader analog_2-val
    :initarg :analog_2
    :type integer
    :initform 0)
   (analog_3
    :reader analog_3-val
    :initarg :analog_3
    :type integer
    :initform 0)
   (digital_4
    :reader digital_4-val
    :initarg :digital_4
    :type integer
    :initform 0)
   (digital_5
    :reader digital_5-val
    :initarg :digital_5
    :type integer
    :initform 0)
   (digital_6
    :reader digital_6-val
    :initarg :digital_6
    :type integer
    :initform 0)
   (digital_7
    :reader digital_7-val
    :initarg :digital_7
    :type integer
    :initform 0)
   (pmad_command_count
    :reader pmad_command_count-val
    :initarg :pmad_command_count
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <Status>) ostream)
  "Serializes a message object of type '<Status>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'analog_0)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'analog_0)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'analog_0)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'analog_0)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'analog_1)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'analog_1)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'analog_1)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'analog_1)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'analog_2)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'analog_2)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'analog_2)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'analog_2)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'analog_3)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'analog_3)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'analog_3)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'analog_3)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'digital_4)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'digital_4)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'digital_4)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'digital_4)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'digital_5)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'digital_5)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'digital_5)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'digital_5)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'digital_6)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'digital_6)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'digital_6)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'digital_6)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'digital_7)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'digital_7)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'digital_7)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'digital_7)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'pmad_command_count)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'pmad_command_count)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'pmad_command_count)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'pmad_command_count)) ostream)
)
(defmethod deserialize ((msg <Status>) istream)
  "Deserializes a message object of type '<Status>"
  (setf (ldb (byte 8 0) (slot-value msg 'analog_0)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'analog_0)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'analog_0)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'analog_0)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'analog_1)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'analog_1)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'analog_1)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'analog_1)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'analog_2)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'analog_2)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'analog_2)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'analog_2)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'analog_3)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'analog_3)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'analog_3)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'analog_3)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'digital_4)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'digital_4)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'digital_4)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'digital_4)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'digital_5)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'digital_5)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'digital_5)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'digital_5)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'digital_6)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'digital_6)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'digital_6)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'digital_6)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'digital_7)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'digital_7)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'digital_7)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'digital_7)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'pmad_command_count)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'pmad_command_count)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'pmad_command_count)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'pmad_command_count)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Status>)))
  "Returns string type for a message object of type '<Status>"
  "pmad/Status")
(defmethod md5sum ((type (eql '<Status>)))
  "Returns md5sum for a message object of type '<Status>"
  "7d1b2debb1384c23a9ee7bf93637df60")
(defmethod message-definition ((type (eql '<Status>)))
  "Returns full string definition for message of type '<Status>"
  (format nil "int32 analog_0~%int32 analog_1~%int32 analog_2~%int32 analog_3~%int32 digital_4~%int32 digital_5~%int32 digital_6~%int32 digital_7~%int32 pmad_command_count~%~%"))
(defmethod serialization-length ((msg <Status>))
  (+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <Status>))
  "Converts a ROS message object to a list"
  (list '<Status>
    (cons ':analog_0 (analog_0-val msg))
    (cons ':analog_1 (analog_1-val msg))
    (cons ':analog_2 (analog_2-val msg))
    (cons ':analog_3 (analog_3-val msg))
    (cons ':digital_4 (digital_4-val msg))
    (cons ':digital_5 (digital_5-val msg))
    (cons ':digital_6 (digital_6-val msg))
    (cons ':digital_7 (digital_7-val msg))
    (cons ':pmad_command_count (pmad_command_count-val msg))
))
