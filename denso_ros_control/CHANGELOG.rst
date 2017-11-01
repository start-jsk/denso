^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package denso_ros_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.2 (2017-11-01)
------------------
* Fix kinetic-devel (denso_ros_control) for simulation usage (`#88 <https://github.com/start-jsk/denso/issues/88>`_)
  * add joint_state_controller
  * add joint_trajectory_controller depends denso_ros_control/package.xml
  * add position_controllers depends denso_ros_control/package.xml
  * CMakeLists.txt: add install target
  * position_control.launch: add --shutdown-timeout 0.1 to clean? shutdown of controllers
  * position_control.launch: add server_ip, udp_timeout, server_port, dryrun arguments
  * use position_trajectory_controller (use joint tarjectory action instead of poision conttoller for each joint)
  * loopback read and write for dryrun mode
  * skip fixed joint, see https://github.com/start-jsk/denso/issues/68#issuecomment-338449016
* Contributors: Kei Okada

2.0.1 (2017-08-09)
------------------

2.0.0 (2017-08-09)
------------------
* initial commit for supporting ros_control (`#82 <https://github.com/start-jsk/denso/issues/82>`)
  * fix compilation error
  * initial commit for supporting ros_control

* Contributors: Shohei Fujii

1.1.8 (2017-03-03)
------------------

1.1.7 (2016-08-23)
------------------

1.1.6 (2016-06-24)
------------------

1.1.5 (2016-04-05)
------------------

1.1.4 (2016-02-08)
------------------

1.1.3 (2016-02-04)
------------------

1.1.2 (2015-12-21)
------------------

1.1.1 (2015-11-03)
------------------

1.1.0 (2015-10-31)
------------------

1.0.0 (2015-10-30)
------------------

0.2.9 (2015-03-07)
------------------

0.2.8 (2014-07-30)
------------------

0.2.7 (2014-01-03)
------------------

0.2.6 (2013-12-13)
------------------

0.2.5 (2013-12-11)
------------------

0.2.4 (2013-12-10)
------------------

0.2.3 (2013-12-07)
------------------

0.2.2 (2013-12-06 20:20)
------------------------

0.2.1 (2013-12-06 02:54)
------------------------

0.2.0 (2013-11-19)
------------------
