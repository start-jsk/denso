#!/usr/bin/env roseus

(load "package://open_controllers/launch/vs060.l")
(load "package://pr2eus/robot-interface.l")
(ros::roseus "vs060_test")
(ros::load-ros-manifest "denso_pendant_publisher")
(ros::load-ros-manifest "sensor_msgs")

(setq *current-joint-states* (float-vector 0 0 0 0 0 0))
(setq *df* 10)


(ros::subscribe "/denso_pendant_publisher/status"
		denso_pendant_publisher::PendantStatus
		#'(lambda (msg)
		    (when (send msg :key_manual)
		    (let ((target-av (send *ri* :state :potentio-vector)))
		      (if (send msg :button_j1_plus)
			  (setf (elt target-av 0) (+ (elt target-av 0) *df*)))
		      (if (send msg :button_j1_minus)
			  (setf (elt target-av 0) (- (elt target-av 0) *df*)))
		      (if (send msg :button_j2_plus)
			  (setf (elt target-av 1) (+ (elt target-av 1) *df*)))
		      (if (send msg :button_j2_minus)
			  (setf (elt target-av 1) (- (elt target-av 1) *df*)))
		      (if (send msg :button_j3_plus)
			  (setf (elt target-av 2) (+ (elt target-av 2) *df*)))
		      (if (send msg :button_j3_minus)
			  (setf (elt target-av 2) (- (elt target-av 2) *df*)))
		      (if (send msg :button_j4_plus)
			  (setf (elt target-av 3) (+ (elt target-av 3) *df*)))
		      (if (send msg :button_j4_minus)
			  (setf (elt target-av 3) (- (elt target-av 3) *df*)))
		      (if (send msg :button_j5_plus)
			  (setf (elt target-av 4) (+ (elt target-av 4) *df*)))
		      (if (send msg :button_j5_minus)
			  (setf (elt target-av 4) (- (elt target-av 4) *df*)))
		      (if (send msg :button_j6_plus)
			  (setf (elt target-av 5) (+ (elt target-av 5) *df*)))
		      (if (send msg :button_j6_minus)
			  (setf (elt target-av 5) (- (elt target-av 5) *df*)))
		      (when (or (send msg :button_j1_plus)
			      (send msg :button_j1_minus)
			      (send msg :button_j2_plus)
			      (send msg :button_j2_minus)
			      (send msg :button_j3_plus)
			      (send msg :button_j3_minus)
			      (send msg :button_j4_plus)
			      (send msg :button_j4_minus)
			      (send msg :button_j5_plus)
			      (send msg :button_j5_minus)
			      (send msg :button_j6_plus)
			      (send msg :button_j6_minus))
			(send *ri* :angle-vector target-av 100)
			)
		      )
		    )))

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
;;(send *irtviewer* :draw-objects)

(ros::spin)
