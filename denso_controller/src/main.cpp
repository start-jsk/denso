/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include "ros/ros.h"
#include <execinfo.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/io_service.hpp>
#include <open_controllers_interface/open_controllers_interface.h>
#include <std_srvs/Empty.h>
#include <boost/shared_ptr.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <pr2_controller_manager/controller_manager.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <std_srvs/Empty.h>

#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#define BCAP_CONNECTION_UDP 1
#include "b-Cap.c"

#define DEFAULT_SERVER_IP_ADDRESS               "133.11.216.196"                /* Your controller IP address */
#define DEFAULT_SERVER_PORT_NUM                 5007
#define DEFAULT_UDP_TIMEOUT (10 * 1000)
#define CLOCK_PRIO 0
#define CONTROL_PRIO 0

#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) / 180.0 * M_PI)

class DensoController : public OpenControllersInterface::OpenController {
public:
  DensoController(): OpenControllersInterface::OpenController(), need_exit(false), loop(true) {
  }
#define SAFE_EXIT(exit_code) {                  \
    ROS_FATAL("fatal error has occurred");      \
    quitRequest();              \
  }

  void start() {
    OpenController::start();
  }
  bool loop;
private:
  char* server_ip_address;
  int server_port_number;
  // for bCap parameters
  int iSockFD;
  u_int lhController;
  u_int lhRobot;
  int udp_timeout;
  bool need_exit;
public:
  // bCAP interface
  void bCapOpen() {
    BCAP_HRESULT hr = BCAP_S_OK;
    hr = bCap_Open(server_ip_address, server_port_number, &iSockFD); /* Init socket  */
    if (FAILED(hr)) {
      ROS_FATAL("bCap_Open failed\n");
      exit(1);
    }
  }

  void bCapControllerConnect() {
    BCAP_HRESULT hr = BCAP_S_OK;
    hr = bCap_ControllerConnect(iSockFD, (char*)"", (char*)"caoProv.DENSO.VRC", 
                                server_ip_address, (char*)"", &lhController);
    if (FAILED(hr)) {
      ROS_FATAL("bCap_ControllerConnect failed\n");
      exit(1);
    }
  }

  void bCapClearError() {
    BCAP_HRESULT hr = BCAP_S_OK;
    long lResult;
    hr = bCap_ControllerExecute(iSockFD, lhController, (char*)"ClearError", (char*)"", &lResult);
    ROS_INFO("clearError %02x %02x", hr, lResult);
  }

  BCAP_HRESULT bCapGetRobot() {
    BCAP_HRESULT hr = BCAP_S_OK;
    long lResult;
    hr = bCap_ControllerGetRobot(iSockFD, lhController, (char*)"", (char*)"", &lhRobot);              /* Get robot handle */
    ROS_INFO("GetRobot %02x %02x", hr, lhRobot);
    return hr;
  }

  BCAP_HRESULT bCapRobotExecute(char* command, char* arg) {
    BCAP_HRESULT hr = BCAP_S_OK;
    long lResult;
    hr = bCap_RobotExecute(iSockFD, lhRobot, command, arg, &lResult);
    return hr;
  }

  void bCapTakearm() {
    BCAP_HRESULT hr = bCapRobotExecute("Takearm", "");
    if (FAILED(hr)) {
      ROS_FATAL("failed to takearm");
      exit(1);
    }
  }

  void bCapSetExternalSpeed(char* arg) {
    BCAP_HRESULT hr = bCapRobotExecute("ExtSpeed", arg);
    if (FAILED(hr)) {
      ROS_FATAL("failed to bCapSetExternalSpeed");
      exit(1);
    }
  }

  BCAP_HRESULT bCapMotor(bool command) {
    BCAP_HRESULT hr = BCAP_S_OK;
    if (command) {
      hr = bCapRobotExecute("Motor", "1");
    }
    else {
      hr = bCapRobotExecute("Motor", "0");
    }
    return hr;
  }

  std::vector<double> bCapCurJnt() {
    double dJnt[8];
    BCAP_HRESULT hr = BCAP_E_FAIL;
    std::vector<double> ret;
    while (FAILED(hr)) {
      hr = bCap_RobotExecute(iSockFD, lhRobot, "CurJnt", "", &dJnt);
      if (SUCCEEDED(hr)) {
        for (int i = 0; i < 8; i++)
          ret.push_back(dJnt[i]);
        fprintf(stderr, "CurJnt : %f %f %f %f %f %f\n", dJnt[0], dJnt[1], dJnt[2], dJnt[3], dJnt[4], dJnt[5], dJnt[6]);
        break;
      }
      else {
        fprintf(stderr, "failed to read joint angles, retry\n");
      }
    }
    return ret;
  }

  BCAP_HRESULT bCapSlvChangeMode(char* arg) {
    long lResult;
    BCAP_HRESULT hr = bCap_RobotExecute(iSockFD, lhRobot, "slvChangeMode", arg, &lResult);
    fprintf(stderr, "slvChangeMode %02x %02x\n", hr, lResult);
    return hr;
  }

  BCAP_HRESULT  bCapRobotSlvMove(BCAP_VARIANT* pose, BCAP_VARIANT* result) {
    //ROS_INFO("bCapRobotSlvMove");
    // ROS_INFO("sending angle: %f %f %f %f %f %f",
    //          pose->Value.DoubleArray[0],
    //          pose->Value.DoubleArray[1],
    //          pose->Value.DoubleArray[2],
    //          pose->Value.DoubleArray[3],
    //          pose->Value.DoubleArray[4],
    //          pose->Value.DoubleArray[5]);


    // struct timeval tv;
    // tv.tv_sec = 0;
    // tv.tv_usec = 1000 * 1;
    // setsockopt(iSockFD, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    extern int failed_to_send_packet;
    struct timespec tick, before;
    static std::vector<double> prev_angle;
    static std::vector<double> prev_vel;
    static struct timespec prev_time;
    std::vector<double> current_angle;
    std::vector<double> current_vel;
    std::vector<double> current_acc;
    for (size_t i = 0; i < 6; i++) {
      current_angle.push_back(pose->Value.DoubleArray[i]);
    }
    
    char* command = (char*)"slvMove";
    clock_gettime(CLOCK_MONOTONIC, &tick);
    BCAP_HRESULT hr = bCap_RobotExecute2(iSockFD, lhRobot, command, pose, result);
    clock_gettime(CLOCK_MONOTONIC, &before);
    static const int NSEC_PER_SECOND = 1e+9;
    //static const int USEC_PER_SECOND = 1e6;
    double roundtrip = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) -
      (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);

    if (prev_angle.size() > 0) {
      double tm = (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND) - (prev_time.tv_sec + double(prev_time.tv_nsec)/NSEC_PER_SECOND);
      for (size_t i = 0; i < 6; i++) {
        current_vel.push_back((current_angle[i] - prev_angle[i]) / tm);
      }
    }
    if (prev_vel.size() > 0) {
      double tm = (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND) - (prev_time.tv_sec + double(prev_time.tv_nsec)/NSEC_PER_SECOND);
      for (size_t i = 0; i < 6; i++) {
        current_acc.push_back((current_vel[i] - prev_vel[i]) / tm);
      }
    }

    prev_time = tick;
    ROS_INFO("current angle: %f %f %f %f %f %f",
               current_angle[0],
               current_angle[1],
               current_angle[2],
               current_angle[3],
               current_angle[4],
               current_angle[5]);

    // if (current_vel.size() > 0) {
    //   ROS_INFO("current vel: %f %f %f %f %f %f",
    //            current_vel[0],
    //            current_vel[1],
    //            current_vel[2],
    //            current_vel[3],
    //            current_vel[4],
    //            current_vel[5]);
    //   }
    //   if (prev_vel.size() > 0) {
    //     ROS_INFO("prev vel: %f %f %f %f %f %f",
    //            prev_vel[0],
    //            prev_vel[1],
    //            prev_vel[2],
    //            prev_vel[3],
    //            prev_vel[4],
    //            prev_vel[5]);
    //   }
    // if (current_acc.size() > 0) {
    //     ROS_INFO("current acc: %f %f %f %f %f %f",
    //            current_acc[0],
    //            current_acc[1],
    //            current_acc[2],
    //            current_acc[3],
    //            current_acc[4],
    //            current_acc[5]);
    //   }

    if (failed_to_send_packet) {
      fprintf(stderr, "roundtrip: %f\n", roundtrip);
      setUDPTimeout(0, udp_timeout);
      
      // print the angle
      // if (prev_angle.size() > 0) {
      //   ROS_INFO("prev angle: %f %f %f %f %f %f",
      //            prev_angle[0],
      //            prev_angle[1],
      //            prev_angle[2],
      //            prev_angle[3],
      //            prev_angle[4],
      //            prev_angle[5]);
      // }
      // ROS_INFO("current angle: %f %f %f %f %f %f",
      //          current_angle[0],
      //          current_angle[1],
      //          current_angle[2],
      //          current_angle[3],
      //          current_angle[4],
      //          current_angle[5]);
    }
    if (FAILED(hr)) {
      // hr = bCapRobotExecute("stopLog", "");
      // if (FAILED(hr)) {
      //   fprintf(stderr, "failed to disable logging mode\n");
      // }
      ROS_FATAL("bCapRobotSlvMove");
      if (hr == BCAP_E_BUF_FULL) {
        ROS_WARN("buffer over flow slvMove %02x", hr);
      }
      else {
        ROS_WARN("failed to send slvMove %02x", hr);
      }
      finalize();
      cleanupPidFile();
      kill(getpid(), SIGKILL);
    }

    // memoize prev angle
    prev_angle = current_angle;
    prev_vel = current_vel;
    return hr;
  }

  virtual void finalizeHW() {
    ROS_INFO("finalizeHW is called");
    if (!dryrunp) {
      setUDPTimeout(2, 0);
      sleep(1);
      long lResult;
      BCAP_HRESULT hr = BCAP_S_OK;
      //bCapClearError();
      // disable logging mode
      hr = bCapRobotExecute("stopLog", "");
      if (FAILED(hr)) {
        fprintf(stderr, "failed to disable logging mode\n");
      }

      hr = bCapSlvChangeMode((char*)"0");
      if (FAILED(hr)) {
        fprintf(stderr, "failed to change slave mode\n");
      }
      hr = bCapMotor(false);
      if (FAILED(hr)) {
        fprintf(stderr, "failed to motor off\n");
      }
      else {
        fprintf(stderr, "successed to motor off\n");
      }
      hr = bCap_RobotExecute(iSockFD, lhRobot, (char*)"Givearm", (char*)"", &lResult);
      if (FAILED(hr)) {
        fprintf(stderr, "failed to give arm\n");
      }
      else {
        fprintf(stderr, "successed to give arm\n");
      }
      hr = bCap_RobotRelease(iSockFD, lhRobot);    /* Release robot handle */
      if (FAILED(hr)) {
        fprintf(stderr, "failed to release the robot\n");
      }
      else {
        fprintf(stderr, "successed to release the robot\n");
      }
      hr = bCap_ControllerDisconnect(iSockFD, lhController);
      if (FAILED(hr)) {
        fprintf(stderr, "failed to disconnect from the controller\n");
      }
      else {
        fprintf(stderr, "successed to disconnect from the controller\n");
      }
      /* Stop b-CAP service (Very important in UDP/IP connection) */
      hr = bCap_ServiceStop(iSockFD);
      if (FAILED(hr)) {
        fprintf(stderr, "failed to stop the service\n");
      }
      else {
        fprintf(stderr, "successed to stop the service\n");
      }
      sleep(1);
      hr = bCap_Close(iSockFD);
      if (FAILED(hr)) {
        fprintf(stderr, "failed to close bCap\n");
      }
      else {
        fprintf(stderr, "successed to close bCap\n");
      }
    }
    fprintf(stderr, "finalizing done\n");
  }

  bool updateJoints(struct timespec* spec_result) {
    static bool initialp = true;
    bool errorp = false;
    if (initialp) {
      ROS_INFO("first loop");

      initialp = false;
      std::vector<double> cur_jnt;
      if (!dryrunp) {
        ROS_INFO("hoge");
        cur_jnt = bCapCurJnt();
        BCAP_VARIANT vntPose, vntResult;
        vntPose.Type = VT_R8 | VT_ARRAY;
        vntPose.Arrays = 8;
        for (int i = 0; i < 8; i++) {
          vntPose.Value.DoubleArray[i] = cur_jnt[i];
        }
        BCAP_HRESULT hr = BCAP_S_OK;
        while (hr == 0 && g_halt_requested == false) {
          hr = bCapRobotSlvMove(&vntPose, &vntResult);
          ROS_INFO("initialize slvmove");
          }
        ROS_INFO("initialization done");
      }
      // fill ac
      int i = 0;
      for (OpenControllersInterface::TransmissionIterator it = cm->model_.transmissions_.begin();
           it != cm->model_.transmissions_.end(); 
           ++it) { // *** js and ac must be consistent
        pr2_hardware_interface::Actuator *ac
          = hw->getActuator((*it)->actuator_names_[0]);
        if (!dryrunp) {
          ac->state_.position_ = DEG2RAD(cur_jnt[i]);
        }
        else {
          ac->state_.position_ = 0;
        }
        i++;
      }
    }
    // build vntPose
    BCAP_VARIANT vntPose;
    vntPose.Type = VT_R8 | VT_ARRAY;
    vntPose.Arrays = 8;
    {
      int i = 0;
      for (OpenControllersInterface::TransmissionIterator it = cm->model_.transmissions_.begin();
           it != cm->model_.transmissions_.end();
           ++it) { // *** js and ac must be consistent
        pr2_mechanism_model::JointState *js
          = cm->state_->getJointState((*it)->joint_names_[0]);
        pr2_hardware_interface::Actuator *ac
          = hw->getActuator((*it)->actuator_names_[0]);
        // ROS_INFO("js: %f, ac: %f", js->commanded_effort_, ac->state_.position_);
        double target_angle = RAD2DEG(ac->state_.position_);
        //if (i > 1) {     // moves only the 6-3rd joints
        if (true) {
          target_angle = RAD2DEG(ac->state_.position_ + js->commanded_effort_);
          // check min/max
#define SAFE_OFFSET_DEG 1
          if (RAD2DEG(cm->state_->model_->robot_model_.getJoint((*it)->joint_names_[0])->limits->lower) + SAFE_OFFSET_DEG > target_angle) {
            ROS_WARN("too small joint angle! %f", target_angle);
            target_angle = RAD2DEG(cm->state_->model_->robot_model_.getJoint((*it)->joint_names_[0])->limits->lower) + SAFE_OFFSET_DEG;
          }
          else if(RAD2DEG(cm->state_->model_->robot_model_.getJoint((*it)->joint_names_[0])->limits->upper) - SAFE_OFFSET_DEG < target_angle) {
            ROS_WARN("too large joint angle! %f", target_angle);
            target_angle = RAD2DEG(cm->state_->model_->robot_model_.getJoint((*it)->joint_names_[0])->limits->upper) - SAFE_OFFSET_DEG;
          }
          //ROS_INFO("target_angle: %f", target_angle);
        }
        else {
          target_angle = RAD2DEG(ac->state_.position_);
        }
        vntPose.Value.DoubleArray[i] = target_angle;
        i++;
      }
    }

    // send vntPose
    BCAP_VARIANT vntReturn;
    if (!dryrunp) {
      if (spec_result) {
        clock_gettime(CLOCK_REALTIME, spec_result);
      }
      BCAP_HRESULT hr = bCapRobotSlvMove(&vntPose, &vntReturn);
      if (hr == 0xF200501) {
        //ROS_INFO("buf is filled, OK");
      }
      else {
        while (hr == 0 && g_halt_requested == false) {             // 0 means that there is empty buffer
          fprintf(stderr, "buf space found, re-send the angles to the controller\n");
          //ROS_WARN("buf space found, re-send the angles to the controller");
          if (spec_result) {
            clock_gettime(CLOCK_REALTIME, spec_result);
          }
          hr = bCapRobotSlvMove(&vntPose, &vntReturn);
          if (hr == 0xF200501) {
            ROS_WARN("buf is filled");
          }
        }
      }
    }

    hw->current_time_ = ros::Time::now(); // ???
    if(!errorp) {
      int i = 0;
      for (OpenControllersInterface::TransmissionIterator it = cm->model_.transmissions_.begin();
           it != cm->model_.transmissions_.end();
           ++it) { // *** js and ac must be consistent
        pr2_mechanism_model::JointState *js = cm->state_->getJointState((*it)->joint_names_[0]);
        pr2_hardware_interface::Actuator *ac = hw->getActuator((*it)->actuator_names_[0]);
        ac->state_.velocity_ = 0;
        if (!dryrunp) {     // if not in the dryrun mode, we just copy the vntReturn value
          ac->state_.position_ = DEG2RAD(vntReturn.Value.DoubleArray[i]);
        }
        else {              // if in the dryrun mode, we just copy the target value
          double target_angle = ac->state_.position_ + js->commanded_effort_;
          ac->state_.position_ = target_angle;
        }
        i++;
      }
    }
      return !errorp;
  }

  void setUDPTimeout(long sec, long usec) {
#ifdef BCAP_CONNECTION_UDP
    struct timeval tv;
    tv.tv_sec = sec;
    tv.tv_usec = usec;
    if (setsockopt(iSockFD, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
      perror("Error");
    }
#endif
  }

  void bCapServerStart() {
    BCAP_HRESULT hr = bCap_ServiceStart(iSockFD);                       /* Start b-CAP service */
    if (FAILED(hr)) {
      ROS_FATAL("bCap_ServiceStart failed\n");
      SAFE_EXIT(1);
    }
  }
  
  void initializeHW() {
    
    hw = new pr2_hardware_interface::HardwareInterface();
    hw->addActuator(new pr2_hardware_interface::Actuator("j1_motor"));
    hw->addActuator(new pr2_hardware_interface::Actuator("j2_motor"));
    hw->addActuator(new pr2_hardware_interface::Actuator("j3_motor"));
    hw->addActuator(new pr2_hardware_interface::Actuator("j4_motor"));
    hw->addActuator(new pr2_hardware_interface::Actuator("j5_motor"));
    hw->addActuator(new pr2_hardware_interface::Actuator("flange_motor"));
    // // Create controller manager
    //pr2_controller_manager::ControllerManager cm(ec.hw_);
    cm = boost::shared_ptr<pr2_controller_manager::ControllerManager>(new pr2_controller_manager::ControllerManager(hw));

    // for denso connection
    if (!dryrunp) {
      need_exit = true;
      ROS_INFO("bCapOpen");
      bCapOpen();
      ROS_INFO("bCapServerStart");
      setUDPTimeout(2, 0);   // 200msec
      bCapServerStart();
      ROS_INFO("bCapControllerConnect");
      bCapControllerConnect();
      ROS_INFO("bCapClearError");
      bCapClearError();
      ROS_INFO("bCapGetRobot");
      bCapGetRobot();
      ROS_INFO("bCapTakearm");
      bCapTakearm();
      ROS_INFO("bCapSetExternalSpeed");
      bCapSetExternalSpeed((char*)"80.0");

      {
        ROS_WARN("bCapMotor On");
        BCAP_HRESULT hr = bCapMotor(true);
        if (FAILED(hr)) {
          ROS_FATAL("failed to motor on");
          finalize();
          SAFE_EXIT(1);
        }
      }
      // sleep(15);
      // // read joint angles sometimes to skip illegal values
      std::vector<double> cur_jnt = bCapCurJnt();
      // 100 * 50m = 5000
      // for (int i = 0; i < 5000 / 20; i++) {
      //   cur_jnt = bCapCurJnt();
      // }
      
      // enable logging
      {
        BCAP_HRESULT hr = bCapRobotExecute("clearLog", "");
        if (FAILED(hr)) {
          ROS_FATAL("failed to enable logging mode");
          SAFE_EXIT(1);
        }
      }

      // enable logging
      // {
      //   BCAP_HRESULT hr = bCapRobotExecute("startLog", "");
      //   if (FAILED(hr)) {
      //     ROS_FATAL("failed to start logging mode");
      //     SAFE_EXIT(1);
      //   }
      // }

      need_exit = false;
      {
        BCAP_HRESULT hr = bCapSlvChangeMode((char*)"514"); // 0x202
        //BCAP_HRESULT hr = bCapSlvChangeMode((char*)"258"); // 0x102
        if (FAILED(hr)) {
          ROS_FATAL("failed to change slave mode");
          SAFE_EXIT(1);
        }
      }

      

      //sleep(5);
      // fill ac
      int i = 0;
      for (OpenControllersInterface::TransmissionIterator it = cm->model_.transmissions_.begin();
           it != cm->model_.transmissions_.end();
           ++it) { // *** js and ac must be consistent
        pr2_hardware_interface::Actuator *ac = hw->getActuator((*it)->actuator_names_[0]);
        ac->state_.position_ = DEG2RAD(cur_jnt[i]); // degree -> degree
        i++;
      }
      setUDPTimeout(0, udp_timeout);
      sleep(5);
      bCapCurJnt();
      bCapCurJnt();
      bCapCurJnt();
      // for (int i = 0; i < 5; i++)
      //   bCapRobotSlvMove(&vntPose, &vntResult);
      // hr = bCapRobotSlvMove(&vntPose, &vntResult);
      // if (hr == 0) {
      //   ROS_WARN("buf space found");
      // }
      // hr = bCapRobotSlvMove(&vntPose, &vntResult);
      // if (hr == 0) {
      //   ROS_WARN("buf space found");
      // }
    }
  }

  void initializeROS(ros::NodeHandle& node) {
    std::string server_ip_address_str;
    if (!node.getParam("server_ip", server_ip_address_str)) {
      server_ip_address = (char*)DEFAULT_SERVER_IP_ADDRESS;
    }
    else {
      server_ip_address = (char*)malloc(sizeof(char) * (server_ip_address_str.length() + 1));
      for (size_t i = 0; i < server_ip_address_str.length(); i++) {
        server_ip_address[i] = server_ip_address_str.at(i);
      }
      server_ip_address[server_ip_address_str.length()] = '\0';
    }

    if (!node.getParam("server_port", server_port_number)) {
      server_port_number = DEFAULT_SERVER_PORT_NUM;
    }

    ROS_WARN("server: %s:%d", server_ip_address, server_port_number);

    if (!node.getParam("udp_timeout", udp_timeout)) {
      udp_timeout = DEFAULT_UDP_TIMEOUT;
    }
    ROS_WARN("udp_timeout: %d micro sec", udp_timeout);
  }

  void quitRequest() {
    fprintf(stderr, "DensoController::quitRequest\n");
    OpenController::quitRequest(); // call super class
    fprintf(stderr, "DensoController::quitRequest called\n");
    if (need_exit) {
      exit(1);
    }
  }
};

DensoController g_controller;
void quitRequested(int sigint) {
  // do nothing
  std::cerr << "quitRequested" << std::endl;
  g_controller.quitRequest();
  ros::shutdown();
}

int main(int argc, char *argv[]) {
  // Keep the kernel from swapping us out
  OpenControllersInterface::OpenController::initRT();

  // Initialize ROS and parse command-line arguments
  ros::init(argc, argv, "denso_controller", ros::init_options::NoSigintHandler);
  // Parse options
  //g_options.program_ = argv[0];
  g_controller.parseArguments(argc, argv);
  if (optind < argc) {
    g_controller.Usage("Extra arguments");
  }

  // do these work?
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);
  int rv = 1;
  g_controller.initialize();
  boost::thread t(&DensoController::start, &g_controller);
  {
    //OpenControllersInterface::Finalizer finalizer(&g_controller);
    ros::Rate loop_rate(100);
    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
    fprintf(stderr, "ROS has been terminated\n");
    t.join();
    fprintf(stderr, "start thread has been terminated\n");
    g_controller.finalize();
  }

  g_controller.cleanupPidFile();
  return rv;
}

