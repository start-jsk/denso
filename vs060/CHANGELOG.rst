^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vs060
^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.2 (2017-11-01)
------------------
* Fix kinetic-devel (denso_ros_control) for simulation usage (`#88 <https://github.com/start-jsk/denso/issues/88>`_)
  * import dummy pyassimp: workaround until https://github.com/ros-planning/moveit/pull/581 is released
  * vs060/model/vs060A1_AV6_NNN_NNN.urdf: add missing safety_controller for j4
  * target_compile_features only available on cmake 3
* [kinetic-devel] Update maintainer (`#85 <https://github.com/start-jsk/denso/issues/85>`_)
* Contributors: Isaac I.Y. Saito, Kei Okada

2.0.1 (2017-08-09)
------------------

2.0.0 (2017-08-09)
------------------
* initial commit for supporting ros_control (`#82 <https://github.com/fkanehiro/hrpsys-base/issues/82>`)
  * fix CATKIN-DEPENDS -> CATKIN_DEPENDS
  * add cxx_range_for publish_scene_from_text
  * drop drop dependency of vs060 to open_controllers_interface

* Contributors: Shohei Fujii

1.1.8 (2017-03-03)
------------------

1.1.7 (2016-08-23)
------------------
* publish_simple_scene.py : quit due to https://github.com/ros-planning/moveit_commander/pull/46/files and wait untel 3 collision object has been published
* Contributors: Kei Okada

1.1.6 (2016-06-24)
------------------

1.1.5 (2016-04-05)
------------------

1.1.4 (2016-02-08)
------------------
* publish_simple_scene: add missing imports
* Contributors: Kei Okada

1.1.3 (2016-02-04)
------------------
* wait for move_group
* add publish_simple_scene.py and fix test code
* add dummy root link, since we had "The root link BASE has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF." warning
* Contributors: Kei Okada

1.1.2 (2015-12-21)
------------------
* vs060_world.launch is no longer requred
* Contributors: Kei Okada

1.1.1 (2015-11-03)
------------------

1.1.0 (2015-10-31)
------------------

1.0.0 (2015-10-30)
------------------
* update to indigo
* Use arg for world file name
* [vs060] use stl file instead of dae file in order to work on gazebo
* Add launch file for Gazebo
* Contributors: Isaac IY Saito, Ryohei Ueda, Hisashi Kanda

0.2.9 (2015-03-07)
------------------

0.2.8 (2014-07-30)
------------------

0.2.7 (2014-01-03)
------------------
* Rename repository (Fix https://github.com/start-jsk/denso/issues/13, https://github.com/start-jsk/denso/issues/14)
* Contributors: Isaac Isao Saito

0.2.6 (2013-12-13)
------------------
* fix to https://github.com/start-jsk/denso/issues/12
* Remove manifest.xml since the entire package depends on wet packages open_industrial_ros_controllers.
* Contributors: Isao Isaac Saito

0.2.5 (2013-12-11)
------------------
* (vs060) Corrected scripts' install path.
* (vs060) add a demo script that runs w/o pendant.
* Contributors: Isao Isaac Saito

0.2.4 (2013-12-10)
------------------
* add a demo script that runs w/o pendant.
* Contributors: Isao Isaac Saito

0.2.3 (2013-12-07)
------------------

0.2.2 (2013-12-06)
------------------

0.2.1 (2013-12-06)
------------------

0.2.0
-----------

* Init commit
* Contributors: Isao Isaac Saito
