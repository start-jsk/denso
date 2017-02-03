#include "denso_ros_control/denso_hardware_interface.h"
#include "controller_manager/controller_manager.h"

bool g_shutdown = false;

void mySigintHandler(int sig) {
  ROS_INFO("signal handler called");
  g_shutdown = true;
}

void shutdown(boost::shared_ptr<DensoRobotHW> robot, boost::shared_ptr<ros::AsyncSpinner> spinner)
{
  if (!!robot) {
    robot.reset();
  }
  //if (!!spinner) {
  //  spinner->stop();
  //  spinner.reset();
  //}
  ros::shutdown();
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "denso_ros_control", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  boost::shared_ptr<DensoRobotHW> robot;
  boost::shared_ptr<ros::AsyncSpinner> spinner;

  robot = boost::make_shared<DensoRobotHW>();
  controller_manager::ControllerManager cm(&(*(robot.get())), nh);

  spinner = boost::make_shared<ros::AsyncSpinner>(1);
  spinner->start();

  ros::Rate rate(1.0 / robot->getPeriod().toSec());
  if(!robot->init(nh,nh)) {
    shutdown(robot, spinner);
    return 1;
  }

  signal(SIGINT, mySigintHandler);
  signal(SIGTERM, mySigintHandler);
  signal(SIGHUP, mySigintHandler);
  while (ros::ok() && !g_shutdown)
  {
    ros::Time now = robot->getTime();
    ros::Duration dt = robot->getPeriod();
    robot->read(now, dt);
    cm.update(now, dt);
    robot->write(now, dt);
    rate.sleep();
  }
  shutdown(robot, spinner);
  return 0;
}
