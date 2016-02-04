^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package denso_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.3 (2016-02-04)
------------------
* call ROS_FATAL before exitting
* do not print server:,,, udp_timeout:... for dryrun mode
* Contributors: Kei Okada

1.1.2 (2015-12-21)
------------------
* use newer api function
* set velocity correctly
* add functions for finalizing things, update recovery procedures
* replace sleep with boost::this_thread::sleep
* add function to reflect real robot state into controller manager
* add emergency_stop variable
* separate a procedure to fill buffer and re-use it
* Contributors: Shohei Fujii

1.1.1 (2015-11-03)
------------------
* [fix] Always run in a dryrun mode (fix `#62 <https://github.com/start-jsk/denso/issues/62>`_)
* Contributors: Shohei Fujii

1.1.0 (2015-10-31)
------------------
* Fix stability issues by @ompugao

  * use macro inside recoverController basically
  * clear error before turn on motor
  * update recovery procedure
  * swept fprintf
  * check bcap status before sending jointvalues
    Question: is it possible to ensure a realtime loop in this way?
  * update recovery procedure
  * add debugging message
  * not to exit inside bcap initialization functions
  * add GetMode function, update recovery procedure
  * add a boilerplait for recovering controller
  * add utf message library
  * fix udp things
  * remove forgotten static variables
  * do not exit inside slvmove
  * minor outputmessage updates
  * add errorcode/errormessage handlers
  * add BCAP_VARIANT alignment fix
  * get rid of static variables
  * add deconstructor
  * get rid of fprintf
  * follow standard c++ style class member naming in DensoController
  * follow standard c++ style class member naming
  * errorp is not set anywhere
  * fix initialization procedure, and fix some output messages
  * do not divide first
  * fix typo
  * remove ros::shutdown, it's called in quitRequest
  * remove useless send command
  * oops, we cannot send clearerror in slave mode
  * use initializeCM
  * remove unused member variables
  * remove ugly 'need_exit'
  * fix error_code redeclaration
  * move intialization procedure to intialize function
  * remove useless lines
  * do not allocate vector everytime get current joint vlaues
  * comment out stoplog
  * change errorlog output
  * minor bugfix
  * treat b-Cap.c as cxx source file
  * fix allocation, fix debug message
  * disable send/recv time output

* Contributors: Shohei Fujii

1.0.0 (2015-10-30)
------------------
* update to indigo

0.2.9 (2015-03-07)
------------------

0.2.8 (2014-07-30)
------------------
* (denso_controller) Corrected copyright
* Comfort to ROS Cpp style guide. Start adding comments (sorry two different types of commits mixed up here...editor did it and I can't revert it)
* Contributors: Isaac IY Saito

0.2.7 (2014-01-03)
------------------
* Rename repository (Fix https://github.com/start-jsk/denso/issues/13, https://github.com/start-jsk/denso/issues/14)
* Contributors: Isao Isaac Saito

0.2.6 (2013-12-13)
------------------

0.2.5 (2013-12-11)
------------------

0.2.4 (2013-12-10)
------------------

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
