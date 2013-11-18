#!/usr/bin/env roseus

;;(load "package://denso_launch/vs060-interface.l")
(ros::load-ros-manifest "control_msgs")
(load "package://denso_launch/vs060-interface.l")
(load "package://pr2eus/robot-interface.l")
(ros::roseus "vs060_test")
(setq *vs060* (vs060A1_AV6_NNN_NNN))
(setq *ri* (instance vs060-interface :init :robot *vs060*))

;; -6.0
(do-until-key ;;while (ros::ok) 
  (let ((av1 (float-vector -60 -30 40 30 30 30))
        (av2 (float-vector 150 0 0 0 0 0))
        ;;(tm 50000)
        ;;(tm 7000)
        ;;(tm 3000);; 
        (tm 1000);; not working
        )
    (send *ri* :angle-vector av1 tm)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector av2 tm)
    (send *ri* :wait-interpolation)
    )

  (unless (ros::ok) (return-from nil nil))
  (ros::sleep 1)
  )
