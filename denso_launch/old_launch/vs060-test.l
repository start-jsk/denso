#!/usr/bin/env roseus

(load "package://open_controllers/launch/vs060.l")
(load "package://pr2eus/robot-interface.l")
(ros::roseus "vs060_test")

(setq *vs060* (VS060A1_AV6_NNN_NNN))
;(if (not (boundp '*irtviewer*)) (make-irtviewer))
;(objects (list *vs060*))

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

(setq *ri* (instance vs060-interface :init :robot *vs060*))
;; (while t (send *ri* :angle-vector #f(-10 -20  30 -40 -50 -60) 3000)(unix:sleep 2)
;; (send *ri* :angle-vector #f(-10 -20 -30  40 -50 -60) 3000)(unix:sleep 2)
;; (send *ri* :angle-vector #f(-10 -20 -30 -40  50 -60) 3000)(unix:sleep 2)

;; ;(send *irtviewer* :draw-objects)

;; (send *ri* :wait-interpolation)



;; )

;; generate random joint angle

(setq *random-state* (coerce (unix::gettimeofday) integer-vector))
(let ((av (coerce (mapcar #'(lambda (j)
                          (let* ((max-angle (1- (send j :max-angle)))
                                 (min-angle (1+ (send j :min-angle)))
                                 (range (floor (- max-angle min-angle))))
                            (let* ((random-ang (random range *random-state*)))
                              (+ random-ang min-angle))))
                      (send *vs060* :joint-list)) float-vector)))
  (print av)
  (send *ri* :angle-vector av 500)
  (unix::sleep 1)
)
(ros::exit)
