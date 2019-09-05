;; Auto-generated. Do not edit!


(when (boundp 'deepracer_simulation::ResetCarSrv)
  (if (not (find-package "DEEPRACER_SIMULATION"))
    (make-package "DEEPRACER_SIMULATION"))
  (shadow 'ResetCarSrv (find-package "DEEPRACER_SIMULATION")))
(unless (find-package "DEEPRACER_SIMULATION::RESETCARSRV")
  (make-package "DEEPRACER_SIMULATION::RESETCARSRV"))
(unless (find-package "DEEPRACER_SIMULATION::RESETCARSRVREQUEST")
  (make-package "DEEPRACER_SIMULATION::RESETCARSRVREQUEST"))
(unless (find-package "DEEPRACER_SIMULATION::RESETCARSRVRESPONSE")
  (make-package "DEEPRACER_SIMULATION::RESETCARSRVRESPONSE"))

(in-package "ROS")





(defclass deepracer_simulation::ResetCarSrvRequest
  :super ros::object
  :slots (_ndist _next_index ))

(defmethod deepracer_simulation::ResetCarSrvRequest
  (:init
   (&key
    ((:ndist __ndist) 0.0)
    ((:next_index __next_index) 0)
    )
   (send-super :init)
   (setq _ndist (float __ndist))
   (setq _next_index (round __next_index))
   self)
  (:ndist
   (&optional __ndist)
   (if __ndist (setq _ndist __ndist)) _ndist)
  (:next_index
   (&optional __next_index)
   (if __next_index (setq _next_index __next_index)) _next_index)
  (:serialization-length
   ()
   (+
    ;; float32 _ndist
    4
    ;; int32 _next_index
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _ndist
       (sys::poke _ndist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _next_index
       (write-long _next_index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _ndist
     (setq _ndist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _next_index
     (setq _next_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass deepracer_simulation::ResetCarSrvResponse
  :super ros::object
  :slots (_success ))

(defmethod deepracer_simulation::ResetCarSrvResponse
  (:init
   (&key
    ((:success __success) 0)
    )
   (send-super :init)
   (setq _success (round __success))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int32 _success
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _success
       (write-long _success s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _success
     (setq _success (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass deepracer_simulation::ResetCarSrv
  :super ros::object
  :slots ())

(setf (get deepracer_simulation::ResetCarSrv :md5sum-) "51459ac9873b166b6314083bf1771edb")
(setf (get deepracer_simulation::ResetCarSrv :datatype-) "deepracer_simulation/ResetCarSrv")
(setf (get deepracer_simulation::ResetCarSrv :request) deepracer_simulation::ResetCarSrvRequest)
(setf (get deepracer_simulation::ResetCarSrv :response) deepracer_simulation::ResetCarSrvResponse)

(defmethod deepracer_simulation::ResetCarSrvRequest
  (:response () (instance deepracer_simulation::ResetCarSrvResponse :init)))

(setf (get deepracer_simulation::ResetCarSrvRequest :md5sum-) "51459ac9873b166b6314083bf1771edb")
(setf (get deepracer_simulation::ResetCarSrvRequest :datatype-) "deepracer_simulation/ResetCarSrvRequest")
(setf (get deepracer_simulation::ResetCarSrvRequest :definition-)
      "float32 ndist
int32 next_index
---
int32 success
")

(setf (get deepracer_simulation::ResetCarSrvResponse :md5sum-) "51459ac9873b166b6314083bf1771edb")
(setf (get deepracer_simulation::ResetCarSrvResponse :datatype-) "deepracer_simulation/ResetCarSrvResponse")
(setf (get deepracer_simulation::ResetCarSrvResponse :definition-)
      "float32 ndist
int32 next_index
---
int32 success
")



(provide :deepracer_simulation/ResetCarSrv "51459ac9873b166b6314083bf1771edb")


