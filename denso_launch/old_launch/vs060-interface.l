#!/usr/bin/env roseus

(require :vs060 "package://open_controllers/launch/vs060.l")
(require :pr2-interface "package://pr2eus/robot-interface.l")

(defclass vs060-interface
  :super robot-interface
  :slots ())
(defmethod vs060-interface
  (:default-controller
   ()
   (list
    (list
     (cons :controller-action "arm_controller/joint_trajectory_action")
     (cons :controller-state "arm_controller/state")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name))))))
  )

(defun vs060-init ()
  (setq *ri* (instance vs060-interface :init :robot (vs060A1_AV6_NNN_NNN)))
  (setq *vs060* (vs060A1_AV6_NNN_NNN))
  ;;(if (not (boundp '*irtviewer*)) (make-irtviewer))
  ;;(objects (list *vs060*))
  ;;(send *irtviewer* :draw-objects)
  *ri*
  )
;;(send *ri* :angle-vector #f(-10 -20 -30 -40  50 -60) 3000)(unix:sleep 2)
;;(send *ri* :wait-interpolation)
