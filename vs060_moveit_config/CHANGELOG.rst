^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vs060_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.3 (2016-02-04)
------------------
* add move_group/MoveGroupGetPlanningSceneService @ move_group.launch
* remove demo_simulation_noenvironment: default launch is denso_vs060_moveit_demo_simulation.launch for dryrun, (i think...)
* update rviz view (add loop, alpha for robot model)
* use RRT Connect as default planner
* update planner configs
* fix for "Group 'manipulator_flange' is not a chain" error message
* Contributors: Kei Okada

1.1.2 (2015-12-21)
------------------
* [moveit config] Remove redundant portion in a launch
* Contributors: Isaac I.Y. Saito

1.1.1 (2015-11-03)
------------------

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
* Contributors: Isaac IY Saito, Kei Okada

0.2.7 (2014-01-03)
------------------
* Rename repository (Fix https://github.com/start-jsk/denso/issues/13, https://github.com/start-jsk/denso/issues/14)
* Contributors: Isao Isaac Saito

0.2.6 (2013-12-13)
------------------
* fix to https://github.com/start-jsk/denso/issues/15
* Contributors: Isao Isaac Saito

0.2.5 (2013-12-11)
------------------
* add launch files to enable standalone demos.
* Contributors: Isao Isaac Saito

0.2.4 (2013-12-10)
------------------
* Add launch file that emulates demo environment at iREX without the need of pendant device.
* Contributors: Isao Isaac Saito

0.2.3 (2013-12-07)
------------------

0.2.2 (2013-12-06)
------------------
* Improvement to .rviz files
* Contributors: Isao Isaac Saito

0.2.1 (2013-12-06)
------------------
* Rename a file to start with lowercase to follow common naming custom
* Contributors: Isao Isaac Saito

0.2.0
-----------
* Init commit
* Contributors: Isao Isaac Saito
