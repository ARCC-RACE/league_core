
(cl:in-package :asdf)

(defsystem "deepracer_simulation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetWaypointSrv" :depends-on ("_package_GetWaypointSrv"))
    (:file "_package_GetWaypointSrv" :depends-on ("_package"))
    (:file "ResetCarSrv" :depends-on ("_package_ResetCarSrv"))
    (:file "_package_ResetCarSrv" :depends-on ("_package"))
  ))