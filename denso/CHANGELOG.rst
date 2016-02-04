^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package denso
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.3 (2016-02-04)
------------------

1.1.2 (2015-12-21)
------------------
* Add gazebo pkg to the metapkg suite
* Contributors: Isaac I.Y. Saito

1.1.1 (2015-11-03)
------------------
* [fix] Always run in a dryrun mode (fix `#62 <https://github.com/start-jsk/denso/issues/62>`_)
* Contributors: Shohei Fujii

1.1.0 (2015-10-31)
------------------

1.0.0 (2015-10-30)
------------------
* update to indigo

0.2.9 (2015-03-07)
------------------
* Add missing .setup_assistant file, without which moveit_setup_assistant can't open an existing moveit config pkg.
* Check-off query start state from RViz conf.
* Contributors: Isaac IY Saito

0.2.8 (2014-07-30)
------------------
* Add xtion checkerboard demo launch file.
* vs060_moveit_config/launch/moveit.rviz: add robot model to rviz
* Corrected build_dependency (add robot_mechanism_controllers, robot_state_publisher)
* Correct robot name (vs06 -> vs060)
* Rename repository (Fix https://github.com/start-jsk/denso/issues/13, https://github.com/start-jsk/denso/issues/14)
* Add moveit_commander test
* Add rostest for /denso_vs060_moveit_demo_simulation.launch
* Corrected copyright (only for some packages)
* Comfort to ROS Cpp style guide. Start adding comments (sorry two different types of commits mixed up here...editor did it and I can't revert it)
* Contributors: Kei Okada, Isaac IY Saito

0.2.7 (2014-01-03)
------------------
* Rename repository (Fix https://github.com/start-jsk/denso/issues/13, https://github.com/start-jsk/denso/issues/14)
* Contributors: Isao Isaac Saito

0.2.6 (2013-12-13)
------------------
* Remove manifest.xml since the entire package depends on wet packages open_industrial_ros_controllers.
* Contributors: Isao Isaac Saito

0.2.5 (2013-12-11)
------------------

0.2.4 (2013-12-10)
------------------
* add a demo script that runs w/o pendant.
* Contributors: Isao Isaac Saito

0.2.3 (2013-12-07)
------------------

0.2.2 (2013-12-06)
------------------
* Improvement to .rviz files
* Contributors: Isao Isaac Saito

0.2.1 (2013-12-06)
------------------

0.2.0
-----------

* Init commit
* Contributors: Isao Isaac Saito
