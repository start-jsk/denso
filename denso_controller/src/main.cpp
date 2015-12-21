/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, JSK Lab at University of Tokyo
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
 *   * Neither the name of the University of Tokyo nor the names of its
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
#include <map>
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
#include <boost/pointer_cast.hpp>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <pr2_controller_manager/controller_manager.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <std_srvs/Empty.h>

#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#define BCAP_CONNECTION_UDP 1
#include "utf8.h"
#include "b-Cap.h"

#define DEFAULT_SERVER_IP_ADDRESS               "133.11.216.196"                /* Your controller IP address */
#define DEFAULT_SERVER_PORT_NUM                 5007
#define DEFAULT_UDP_TIMEOUT (10 * 1000)
#define CLOCK_PRIO 0
#define CONTROL_PRIO 0

#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0)

class DensoController : public OpenControllersInterface::OpenController
{
  class DensoControllerStatus : public OpenControllersInterface::ControllerStatus {
    public:
      DensoControllerStatus(BCAP_HRESULT hr): hr_(hr){};
      DensoControllerStatus(u_int error_code): hr_((BCAP_HRESULT)error_code){};
      virtual ~DensoControllerStatus(){};
      virtual bool isHealthy(){
          return SUCCEEDED(hr_);
      }
      BCAP_HRESULT hr_;
  };
  typedef boost::shared_ptr<DensoControllerStatus> DensoControllerStatusPtr;

public:
  DensoController() :
      OpenControllersInterface::OpenController()
  {
    prev_angle_.resize(6);
    prev_vel_.resize(6);
  }
#define SAFE_EXIT(exit_code) {                  \
    ROS_FATAL("fatal error has occurred");      \
    quitRequest();              \
  }

  virtual ~DensoController(){}

  void start()
  {
    OpenController::start();
  }
private:
  std::string server_ip_address_;
  int server_port_number_;
  // for bCap parameters
  int iSockFD_;
  u_int lhController_;
  u_int lhRobot_;
  int udp_timeout_;
  std::map<std::string, u_int>   var_handlers_;
  std::map<std::string, u_short> var_types_;
  char errdesc_buffer_[2048]; // TODO: is this enough?

  std::vector<double> prev_angle_;
  std::vector<double> prev_vel_;
  struct timespec prev_time_;

public:
  /**
   * Open a bCap socket.
   */
  BCAP_HRESULT bCapOpen()
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    hr = bCap_Open(server_ip_address_.c_str(), server_port_number_, &iSockFD_); /* Init socket  */
    return hr;
  }

  BCAP_HRESULT bCapClose()
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    hr = bCap_Close(iSockFD_);
    return hr;
  }

  BCAP_HRESULT bCapControllerConnect()
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    hr = bCap_ControllerConnect(iSockFD_, (char*)"", (char*)"caoProv.DENSO.VRC", (char*)(server_ip_address_.c_str()), (char*)"",
                                &lhController_);
    return hr;
  }

  BCAP_HRESULT bCapControllerDisConnect()
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    hr = bCap_ControllerDisconnect(iSockFD_, lhController_);
    return hr;
  }

  BCAP_HRESULT bCapClearError()
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    long lResult;
    hr = bCap_ControllerExecute(iSockFD_, lhController_, (char*)"ClearError", (char*)"", &lResult);
    ROS_INFO("clearError %02x %02x", hr, lResult);
    return hr;
  }

  BCAP_HRESULT bCapGetRobot()
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    long lResult;
    hr = bCap_ControllerGetRobot(iSockFD_, lhController_, (char*)"", (char*)"", &lhRobot_); /* Get robot handle */
    ROS_INFO("GetRobot %02x %02x", hr, lhRobot_);
    return hr;
  }
  BCAP_HRESULT bCapReleaseRobot()
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    hr = bCap_RobotRelease(iSockFD_, lhRobot_); /* Release robot handle */
    return hr;
  }

  BCAP_HRESULT bCapRobotExecute(char* command, char* arg)
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    long lResult;
    hr = bCap_RobotExecute(iSockFD_, lhRobot_, command, arg, &lResult);
    return hr;
  }

  BCAP_HRESULT bCapTakearm()
  {
    BCAP_VARIANT takearmparam, takearmresult;
    takearmparam.Type = VT_I4 | VT_ARRAY;
    takearmparam.Arrays = 2;
    ((u_int*)(&takearmparam.Value.LongValue))[0] = 0;
    ((u_int*)(&takearmparam.Value.LongValue))[1] = 1;
    BCAP_HRESULT hr;
    //hr = bCap_RobotExecute(iSockFD_, lhRobot_, "Takearm", takearmparam, &takearmresult);
    hr = bCap_RobotExecute2(iSockFD_, lhRobot_, "Takearm", &takearmparam, &takearmresult);
    //BCAP_HRESULT hr = bCapRobotExecute("Takearm", "");
    return hr;
  }

  BCAP_HRESULT bCapGiveArm()
  {
    BCAP_VARIANT lResult;
    BCAP_HRESULT hr;
    hr = bCap_RobotExecute(iSockFD_, lhRobot_, (char*)"Givearm", (char*)"", &lResult);
    return hr;
  }

  BCAP_HRESULT bCapSetExternalSpeed(float arg)
  {
    BCAP_HRESULT hr;
    BCAP_VARIANT extspeedparam, extspeedresult;
    extspeedparam.Type = VT_R4 | VT_ARRAY;
    extspeedparam.Arrays = 1;
    extspeedparam.Value.FloatValue = arg;
    //((u_int*)(&extspeedparam.Value.LongValue))[1] = 0;
    //((u_int*)(&extspeedparam.Value.LongValue))[2] = 0;
    //hr = bCap_RobotExecute(iSockFD_, lhRobot_, "ExtSpeed", (char*)"80.0", &lResult);
    hr = bCap_RobotExecute2(iSockFD_, lhRobot_, "ExtSpeed", &extspeedparam, &extspeedresult);

    //BCAP_HRESULT hr = bCapRobotExecute("ExtSpeed", arg);
    return hr;
  }

  void bCapInitializeVariableHandlers()
  {
    var_handlers_.insert(std::map<std::string, u_int>::value_type("MODE",bCapControllerGetVariable("@MODE")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("ERROR_CODE",bCapControllerGetVariable("@ERROR_CODE")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("ERROR_DESCRIPTION",bCapControllerGetVariable("@ERROR_DESCRIPTION")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("VERSION",bCapControllerGetVariable("@VERSION")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("SPEED",bCapRobotGetVariable("@SPEED")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("ACCEL",bCapRobotGetVariable("@ACCEL")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("DECEL",bCapRobotGetVariable("@DECEL")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("JSPEED",bCapRobotGetVariable("@JSPEED")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("JACCEL",bCapRobotGetVariable("@JACCEL")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("JDECEL",bCapRobotGetVariable("@JDECEL")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("EXTSPEED",bCapRobotGetVariable("@EXTSPEED")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("EXTACCEL",bCapRobotGetVariable("@EXTACCEL")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("EXTDECEL",bCapRobotGetVariable("@EXTDECEL")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("SERVO_ON",bCapRobotGetVariable("@SERVO_ON")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("BUSY_STATUS",bCapControllerGetVariable("@BUSY_STATUS")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("CURRENT_TIME",bCapControllerGetVariable("@CURRENT_TIME")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("LOCK",bCapControllerGetVariable("@LOCK")));
    var_handlers_.insert(std::map<std::string, u_int>::value_type("EMERGENCY_STOP",bCapControllerGetVariable("@EMERGENCY_STOP")));

    var_types_.insert(std::map<std::string, u_short>::value_type("ERROR_CODE", VT_I4));
    var_types_.insert(std::map<std::string, u_short>::value_type("MODE", VT_I4));
    var_types_.insert(std::map<std::string, u_short>::value_type("ERROR_DESCRIPTION", VT_BSTR));
    var_types_.insert(std::map<std::string, u_short>::value_type("VERSION", VT_BSTR));
    var_types_.insert(std::map<std::string, u_short>::value_type("SPEED", VT_R4));
    var_types_.insert(std::map<std::string, u_short>::value_type("ACCEL", VT_R4));
    var_types_.insert(std::map<std::string, u_short>::value_type("DECEL", VT_R4));
    var_types_.insert(std::map<std::string, u_short>::value_type("JSPEED", VT_R4));
    var_types_.insert(std::map<std::string, u_short>::value_type("JACCEL", VT_R4));
    var_types_.insert(std::map<std::string, u_short>::value_type("JDECEL", VT_R4));
    var_types_.insert(std::map<std::string, u_short>::value_type("EXTSPEED", VT_R4));
    var_types_.insert(std::map<std::string, u_short>::value_type("EXTACCEL", VT_R4));
    var_types_.insert(std::map<std::string, u_short>::value_type("EXTDECEL", VT_R4));
    var_types_.insert(std::map<std::string, u_short>::value_type("BUSY_STATUS", VT_BOOL));
    var_types_.insert(std::map<std::string, u_short>::value_type("CURRENT_TIME", VT_DATE));
    var_types_.insert(std::map<std::string, u_short>::value_type("LOCK", VT_BOOL));
    var_types_.insert(std::map<std::string, u_short>::value_type("EMERGENCY_STOP", VT_BOOL));
  }

  u_int bCapControllerGetVariable(const std::string& varname)
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    u_int varresult;
    hr = bCap_ControllerGetVariable(iSockFD_, lhController_, (char*)(varname.c_str()), (char*)"", &varresult);
    if(FAILED(hr)) {
        ROS_WARN("failed to controller get variable '%s' hr:%02x result:%02x", varname.c_str(), hr, varresult);
    }
    return varresult;
  }

  u_int bCapRobotGetVariable(const std::string& varname)
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    u_int varresult;
    hr = bCap_RobotGetVariable(iSockFD_, lhRobot_, (char*)(varname.c_str()), (char*)"", &varresult);
    if(FAILED(hr)) {
        ROS_WARN("failed to controller get variable '%s' hr:%02x result:%02x", varname.c_str(), hr, varresult);
    }
    return varresult;
  }

  bool bCapGetBoolVariable(const std::string& var_name)
  {
    bool var = 0;
    if( var_types_.find(var_name.c_str()) != var_types_.end() ) {
      BCAP_HRESULT hr = bCap_VariableGetValue(iSockFD_, var_handlers_[var_name.c_str()], &var);
      if( FAILED(hr) ) {
        ROS_WARN("Failed to get %s, return 0 instead.", var_name.c_str());
        return 0;
      }
      return var;
    }
    ROS_WARN("No handler for %s, return 0 instead.", var_name.c_str());
    return 0;
  }

  u_int bCapGetIntegerVariable(const std::string& var_name)
  {
    u_int var = 0;
    if( var_types_.find(var_name.c_str()) != var_types_.end() ) {
      BCAP_HRESULT hr = bCap_VariableGetValue(iSockFD_, var_handlers_[var_name.c_str()], &var);
      if( FAILED(hr) ) {
        ROS_WARN("Failed to get %s, return 0 instead.", var_name.c_str());
        return 0;
      }
      return var;
    }
    ROS_WARN("No handler for %s, return 0 instead.", var_name.c_str());
    return 0;
  }

  u_int __errorcode__;
  u_int bCapGetErrorCode()
  {
    //u_int errorcode = 0; // this local variable is not allocated in debug mode...? why...
    __errorcode__ = 0;
    if( var_types_.find("ERROR_CODE") != var_types_.end() ) {
      BCAP_HRESULT hr = bCap_VariableGetValue(iSockFD_, var_handlers_["ERROR_CODE"], &__errorcode__);
      if( FAILED(hr) ) {
        ROS_WARN("Failed to get ERROR_CODE, return %02x instead.", hr);
        return hr;
      }
      return __errorcode__;
    }
    ROS_WARN("ERROR_CODE is not in variable handlers");
    return 0;
  }

  // 1: manual, 2: teachcheck, 3:auto
  u_int bCapGetMode()
  {
    return bCapGetIntegerVariable("MODE");
  }

  bool bCapGetEmergencyStop()
  {
    return bCapGetBoolVariable("EMERGENCY_STOP");
  }

  u_int bCapErrorDescription(std::string& errormsg)
  {
    if( var_types_.find("ERROR_DESCRIPTION") != var_types_.end() ) {
      BCAP_HRESULT hr = bCap_VariableGetValue(iSockFD_, var_handlers_["ERROR_DESCRIPTION"], errdesc_buffer_);
      errormsg = errdesc_buffer_; //copy

      // find null
      size_t strlen = 0;
      while(strlen<2000) {
        if( *((uint16_t*)errdesc_buffer_+strlen) == 0 ) {
          break;
        }
        ++strlen;
      }

      ROS_INFO("raw error description: %s", errdesc_buffer_);
      try {
          utf8::utf16to8((uint16_t*)errdesc_buffer_, (uint16_t*)errdesc_buffer_+strlen, std::back_inserter(errormsg));
      } catch (utf8::invalid_utf16& e) {
          errormsg = errdesc_buffer_; //copy
      }
      return hr;
    }
    ROS_WARN("ERROR_DESCRIPTION is not in variable handlers");
    return 0;
  }


  BCAP_HRESULT bCapMotor(bool command)
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    BCAP_VARIANT motorarmparam, motorarmresult;
    motorarmparam.Type = VT_I4 | VT_ARRAY;
    motorarmparam.Arrays = 2;
    if (command)
    {
      //hr = bCapRobotExecute("Motor", "1");
      ((u_int*)(&motorarmparam.Value.LongValue))[0] = 1;
      ((u_int*)(&motorarmparam.Value.LongValue))[1] = 0;

      hr = bCap_RobotExecute2(iSockFD_, lhRobot_, "Motor", &motorarmparam, &motorarmresult);
    }
    else
    {
      //hr = bCapRobotExecute("Motor", "0");
      ((u_int*)(&motorarmparam.Value.LongValue))[0] = 0;
      ((u_int*)(&motorarmparam.Value.LongValue))[1] = 1;

      hr = bCap_RobotExecute2(iSockFD_, lhRobot_, "Motor", &motorarmparam, &motorarmresult);
    }
    return hr;
  }

  BCAP_HRESULT bCapCurJnt(std::vector<double>& jointvalues)
  {
    jointvalues.resize(8);
    BCAP_HRESULT hr = BCAP_E_FAIL;
    hr = bCap_RobotExecute(iSockFD_, lhRobot_, "CurJnt", "", &jointvalues[0]);
    return hr;
  }

  BCAP_HRESULT bCapSlvChangeMode(u_int arg)
  {
    //long lResult;
    //BCAP_HRESULT hr = bCap_RobotExecute(iSockFD_, lhRobot_, "slvChangeMode", arg, &lResult);
    BCAP_HRESULT hr;
    BCAP_VARIANT slvchangeparam, slvchangeresult;
    slvchangeparam.Type = VT_I4;
    slvchangeparam.Arrays = 1;
    slvchangeparam.Value.LongValue = arg;
    hr = bCap_RobotExecute2(iSockFD_, lhRobot_, "slvChangeMode", &slvchangeparam, &slvchangeresult);
    //ROS_INFO("change slave mode: hr: %02x, result: %02x", hr, slvchangeresult);
    return hr;
  }

  BCAP_HRESULT bCapRobotSlvMove(BCAP_VARIANT* pose, BCAP_VARIANT* result)
  {
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
    // setsockopt(iSockFD_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    extern int failed_to_send_packet;
    struct timespec tick, before;
    std::vector<double> current_angle;
    std::vector<double> current_vel;
    std::vector<double> current_acc;
    for (size_t i = 0; i < 6; i++)
    {
      current_angle.push_back(pose->Value.DoubleArray[i]);
    }

    char* command = (char*)"slvMove";
    clock_gettime(CLOCK_MONOTONIC, &tick);
    BCAP_HRESULT hr = bCap_RobotExecute2(iSockFD_, lhRobot_, command, pose, result);

    if (FAILED(hr)) {
      ROS_WARN("failed to slvmove, errorcode: %02x", hr);
    }

    clock_gettime(CLOCK_MONOTONIC, &before);
    static const int NSEC_PER_SECOND = 1e+9;
    //static const int USEC_PER_SECOND = 1e6;
    double roundtrip = (before.tv_sec + double(before.tv_nsec) / NSEC_PER_SECOND)
        - (tick.tv_sec + double(tick.tv_nsec) / NSEC_PER_SECOND);

    if (prev_angle_.size() > 0)
    {
      double tm = (tick.tv_sec + double(tick.tv_nsec) / NSEC_PER_SECOND)
          - (prev_time_.tv_sec + double(prev_time_.tv_nsec) / NSEC_PER_SECOND);
      current_vel.resize(6);
      for (size_t i = 0; i < 6; i++)
      {
        current_vel.at(i) = (current_angle[i] - prev_angle_[i]) / tm;
      }
    }
    if (prev_vel_.size() > 0)
    {
      double tm = (tick.tv_sec + double(tick.tv_nsec) / NSEC_PER_SECOND)
          - (prev_time_.tv_sec + double(prev_time_.tv_nsec) / NSEC_PER_SECOND);
      current_acc.resize(6);
      for (size_t i = 0; i < 6; i++)
      {
        current_acc.at(i) = (current_vel[i] - prev_vel_[i]) / tm;
      }
    }

    prev_time_ = tick;
    //ROS_INFO("current angle: %f %f %f %f %f %f", current_angle[0], current_angle[1], current_angle[2], current_angle[3],
             //current_angle[4], current_angle[5]);

    // if (current_vel.size() > 0) {
    //   ROS_INFO("current vel: %f %f %f %f %f %f",
    //            current_vel[0],
    //            current_vel[1],
    //            current_vel[2],
    //            current_vel[3],
    //            current_vel[4],
    //            current_vel[5]);
    //   }
    //   if (prev_vel_.size() > 0) {
    //     ROS_INFO("prev vel: %f %f %f %f %f %f",
    //            prev_vel_[0],
    //            prev_vel_[1],
    //            prev_vel_[2],
    //            prev_vel_[3],
    //            prev_vel_[4],
    //            prev_vel_[5]);
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

    if (failed_to_send_packet)
    {
      ROS_WARN("  roundtrip: %f", roundtrip);
      setUDPTimeout((udp_timeout_ / 1000), (udp_timeout_ % 1000) * 1000);

      // print the angle
      // if (prev_angle_.size() > 0) {
      //   ROS_INFO("prev angle: %f %f %f %f %f %f",
      //            prev_angle_[0],
      //            prev_angle_[1],
      //            prev_angle_[2],
      //            prev_angle_[3],
      //            prev_angle_[4],
      //            prev_angle_[5]);
      // }
      // ROS_INFO("current angle: %f %f %f %f %f %f",
      //          current_angle[0],
      //          current_angle[1],
      //          current_angle[2],
      //          current_angle[3],
      //          current_angle[4],
      //          current_angle[5]);
    }

    // memoize prev angle
    for (size_t i = 0; i < 6; i++) {
        prev_angle_.at(i) = current_angle.at(i);
        prev_vel_.at(i) = current_vel.at(i);
        
    }
    return hr;
  }

  BCAP_HRESULT bCapFillBuffer()
  {
      ROS_WARN("try to fill buffer");
      BCAP_HRESULT hr;
      std::vector<double> cur_jnt;
      hr = bCapCurJnt(cur_jnt);
      BCAP_VARIANT vntPose, vntResult;
      vntPose.Type = VT_R8 | VT_ARRAY;
      vntPose.Arrays = 8;
      for (int i = 0; i < 8; i++)
      {
        vntPose.Value.DoubleArray[i] = cur_jnt[i];
      }
      // Fill the buffer for later use, 
      // unless fill the buffer, bCap slave will fall down 
      for (int i = 0; i < 4; i++) {
          hr = bCapRobotSlvMove(&vntPose, &vntResult);
          if (FAILED(hr)) {
            return hr;
          }
      }
  }

  BCAP_HRESULT bCapReflectRealState() {
      // fill ac
      std::vector<double> cur_jnt;
      BCAP_HRESULT hr;
      hr = bCapCurJnt(cur_jnt);
      int i = 0;
      for (OpenControllersInterface::TransmissionIterator it = cm_->model_.transmissions_.begin();
          it != cm_->model_.transmissions_.end(); ++it)
      { // *** js and ac must be consistent
        pr2_hardware_interface::Actuator *ac = hw_->getActuator((*it)->actuator_names_[0]);
        ac->state_.position_ = DEG2RAD(cur_jnt[i]); // degree -> radian
        i++;
      }
      return hr;
  }

  virtual void finalizeHW()
  {
    ROS_INFO("finalizeHW is called");
    if (!dryrunp_)
    {
      setUDPTimeout(2, 0);
      boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
      long lResult;
      BCAP_HRESULT hr = BCAP_S_OK;
      //bCapClearError();

      hr = bCapSlvChangeMode(0);
      if (FAILED(hr))
      {
        ROS_WARN("failed to change slave mode");
        SAFE_EXIT(1);
      }

      // disable logging mode
      // hr = bCapRobotExecute("stopLog", "");
      // if (FAILED(hr))
      // {
      //   ROS_WARN(failed to disable logging mode");
      // }

      hr = bCapMotor(false);
      if (FAILED(hr))
      {
        ROS_WARN("failed to motor off");
      }
      else
      {
        ROS_INFO("successed to motor off");
      }
      hr = bCapGiveArm();
      if (FAILED(hr))
      {
        ROS_WARN("failed to give arm");
      }
      else
      {
        ROS_INFO("successed to give arm");
      }
      hr = bCapReleaseRobot();
      if (FAILED(hr))
      {
        ROS_WARN("failed to release the robot");
      }
      else
      {
        ROS_INFO( "successed to release the robot");
      }
      hr = bCapControllerDisConnect();
      if (FAILED(hr))
      {
        ROS_WARN("failed to disconnect from the controller");
      }
      else
      {
        ROS_INFO("successed to disconnect from the controller");
      }
      /* Stop b-CAP service (Very important in UDP/IP connection) */
      hr = bCapServerStop();
      if (FAILED(hr))
      {
        ROS_WARN("failed to stop the service");
      }
      else
      {
        ROS_INFO("successed to stop the service");
      }
      boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
      hr = bCapClose();
      if (FAILED(hr))
      {
        ROS_WARN("failed to close bCap");
      }
      else
      {
        ROS_INFO("successed to close bCap");
      }
    }
    ROS_INFO("finalizing finished");
  }

  OpenControllersInterface::ControllerStatusPtr updateJoints(struct timespec* spec_result)
  {
    // build vntPose
    BCAP_VARIANT vntPose;
    vntPose.Type = VT_R8 | VT_ARRAY;
    vntPose.Arrays = 8;
    {
      int i = 0;
      for (OpenControllersInterface::TransmissionIterator it = cm_->model_.transmissions_.begin();
          it != cm_->model_.transmissions_.end(); ++it)
      { // *** js and ac must be consistent
        pr2_mechanism_model::JointState *js = cm_->state_->getJointState((*it)->joint_names_[0]);
        pr2_hardware_interface::Actuator *ac = hw_->getActuator((*it)->actuator_names_[0]);
        // ROS_INFO("js: %f, ac: %f", js->commanded_effort_, ac->state_.position_);
        double target_angle = RAD2DEG(ac->state_.position_);
        if (true)
        {
          target_angle = RAD2DEG(ac->state_.position_ + js->commanded_effort_);
          // check min/max
#define SAFE_OFFSET_DEG 1
          if (RAD2DEG(cm_->state_->model_->robot_model_.getJoint((*it)->joint_names_[0])->limits->lower)
              + SAFE_OFFSET_DEG > target_angle)
          {
            ROS_WARN("too small joint angle! %f", target_angle);
            target_angle = RAD2DEG(
                cm_->state_->model_->robot_model_.getJoint((*it)->joint_names_[0])->limits->lower) + SAFE_OFFSET_DEG;
          }
          else if (RAD2DEG(cm_->state_->model_->robot_model_.getJoint((*it)->joint_names_[0])->limits->upper)
              - SAFE_OFFSET_DEG < target_angle)
          {
            ROS_WARN("too large joint angle! %f", target_angle);
            target_angle = RAD2DEG(
                cm_->state_->model_->robot_model_.getJoint((*it)->joint_names_[0])->limits->upper) - SAFE_OFFSET_DEG;
          }
          //ROS_INFO("target_angle: %f", target_angle);
        }
        else
        {
          target_angle = RAD2DEG(ac->state_.position_);
        }
        vntPose.Value.DoubleArray[i] = target_angle;
        i++;
      }
    }
    ROS_DEBUG("target  angles: %f %f %f %f %f %f", vntPose.Value.DoubleArray[0], vntPose.Value.DoubleArray[1], vntPose.Value.DoubleArray[2], vntPose.Value.DoubleArray[3], vntPose.Value.DoubleArray[4], vntPose.Value.DoubleArray[5]);
    vntPose.Value.DoubleArray[6] = 0;
    vntPose.Value.DoubleArray[7] = 0;
    // send vntPose
    DensoControllerStatusPtr status;
    BCAP_VARIANT vntReturn;
    if (!dryrunp_)
    {
      if (spec_result)
      {
        clock_gettime(CLOCK_REALTIME, spec_result);
      }
      // check bcap status beforehand
      u_int errorcode;
      errorcode = bCapGetErrorCode();
      if (FAILED(errorcode)) {
        ROS_WARN("bad status! robot needs recovery! hr: %02x", errorcode);
        status.reset(new DensoControllerStatus(errorcode));
        return boost::static_pointer_cast<OpenControllersInterface::ControllerStatus>(status);
      }
      BCAP_HRESULT hr = bCapRobotSlvMove(&vntPose, &vntReturn);
      ROS_DEBUG("updateJoints hr: %02x", hr);
      if (hr == 0xF200501)
      {
        ROS_DEBUG("buf is filled, it's fine.");
        status.reset(new DensoControllerStatus(BCAP_S_OK));
      }
      else if (FAILED(hr))
      {
        status.reset(new DensoControllerStatus(hr));
      }
      else
      {
        //status.reset(new DensoControllerStatus(BCAP_S_OK));
        //while (hr == 0 && g_halt_requested_ == false)
        //{
        //  // 0 means that there is an empty buffer
        //  ROS_WARN("buf space found, re-send the angles to the controller");
        //  if (spec_result)
        //  {
        //    clock_gettime(CLOCK_REALTIME, spec_result);
        //  }
        //  hr = bCapRobotSlvMove(&vntPose, &vntReturn);
        //  if (hr == 0xF200501)
        //  {
        //    ROS_WARN("buf is filled");
        //  }
        //}
        status.reset(new DensoControllerStatus(hr));
      }
    } else {
      // always return a healthy status when it's running in dryurun mode
      status.reset(new DensoControllerStatus(BCAP_S_OK));
      for (size_t i = 0; i < 16; i++) {
        vntReturn.Value.DoubleArray[i] = vntPose.Value.DoubleArray[i];
      }

    }

    ROS_DEBUG("current angles: %f %f %f %f %f %f", vntReturn.Value.DoubleArray[0], vntReturn.Value.DoubleArray[1], vntReturn.Value.DoubleArray[2], vntReturn.Value.DoubleArray[3], vntReturn.Value.DoubleArray[4], vntReturn.Value.DoubleArray[5]);
    hw_->current_time_ = ros::Time::now(); // ???
    {
      int i = 0;
      for (OpenControllersInterface::TransmissionIterator it = cm_->model_.transmissions_.begin();
          it != cm_->model_.transmissions_.end(); ++it)
      { // *** js and ac must be consistent
        pr2_mechanism_model::JointState *js = cm_->state_->getJointState((*it)->joint_names_[0]);
        pr2_hardware_interface::Actuator *ac = hw_->getActuator((*it)->actuator_names_[0]);
        ac->state_.velocity_ = DEG2RAD(prev_vel_.at(i));
        if (!dryrunp_)
        { // if not in the dryrun mode, we just copy the vntReturn value
          ac->state_.position_ = DEG2RAD(vntReturn.Value.DoubleArray[i]);
        }
        else
        { // if in the dryrun mode, we just copy the target value
          double target_angle = ac->state_.position_ + js->commanded_effort_;
          ac->state_.position_ = target_angle;
        }
        i++;
      }
    }
    return boost::static_pointer_cast<OpenControllersInterface::ControllerStatus>(status);
  }


  OpenControllersInterface::ControllerStatusPtr recoverController()
  {
#define CAST_STATUS(hr) \
    boost::static_pointer_cast<OpenControllersInterface::ControllerStatus>(DensoControllerStatusPtr(new DensoControllerStatus(hr)))

    ROS_WARN("try to recover controller...");
    u_int errorcode;
    std::string errormsg;

    errorcode = bCapGetErrorCode();
    bCapErrorDescription(errormsg);
    
    ROS_INFO("errormsg: %s", errormsg.c_str());

    if (errorcode == BCAP_E_UNEXPECTED)
    {
      ROS_FATAL("Unexpected Error occured. no way to recover!");
      return CAST_STATUS(errorcode);
    }

    if (errorcode >= 0x83204071 && errorcode <= 0x83204078)
    {
      ROS_INFO("joint angle is over the software limit.");
      ROS_INFO("currently, there is no way to recover, quit.");
      // TODO publish message and return healthy status so that an application can send recovery-trajectory.
      return CAST_STATUS(BCAP_E_FAIL);
    }
    if (errorcode >= 0x84204081 && errorcode <= 0x842040A8)
    {
      ROS_INFO("joint angle velocity is over the software limit.");
      ROS_INFO("currently, there is no way to recover, quit.");
      // TODO publish message and return healthy status so that an application can send recovery-trajectory.
      return CAST_STATUS(BCAP_E_FAIL);
    }
    if (errorcode >= 0x84204051 && errorcode <= 0x84204058)
    {
      ROS_INFO("joint angle velocity(sent) is over the software limit.");
      //ROS_INFO("currently, there is no way to recover, quit.");
      //return CAST_STATUS(BCAP_E_FAIL);
      // TODO publish message and return healthy status so that an application can send recovery-trajectory.
      BCAP_HRESULT hr;
      hr = bCapClearError();
      if (FAILED(hr)) {
          ROS_WARN("failed to clear error %02x", hr);
          return CAST_STATUS(hr);
      }
      hr = bCapGiveArm();
      if (FAILED(hr)) {
          ROS_WARN("failed to give arm %02x", hr);
          return CAST_STATUS(hr);
      }
      //hr = bCap_RobotRelease(iSockFD_, lhRobot_); /* Release robot handle */
      //if (FAILED(hr))
      //{
      //  ROS_WARN("failed to release the robot");
      //}
      //hr = bCapGetRobot();
      //if (FAILED(hr)) {
      //    ROS_WARN("failed to get robot %02x", hr);
      //    return CAST_STATUS(hr);
      //}
      boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
      hr = bCapTakearm();
      if (FAILED(hr)) {
          ROS_WARN("failed to take arm %02x", hr);
          return CAST_STATUS(hr);
      }
      hr = bCapMotor(true);
      if (FAILED(hr)) {
          ROS_WARN("failed to turn on motor %02x", hr);
          return CAST_STATUS(hr);
      }
      hr = bCapSlvChangeMode(0x202);
      if (FAILED(hr)) {
          ROS_WARN("failed to change to slvmode %02x", hr);
          return CAST_STATUS(hr);
      }
      return CAST_STATUS(BCAP_S_OK);
    }
    if (errorcode >= 0x84204041 && errorcode <= 0x84204048)
    {
      ROS_INFO("joint angle acceleration is over the software limit.");
      ROS_INFO("currently, there is no way to recover, quit.");
      // TODO publish message and return healthy status so that an application can send recovery-trajectory.
      return CAST_STATUS(BCAP_E_FAIL);
    }


    if (errorcode == 0x83204231)
    {
      ROS_INFO("invalid command was sent and everything stopped, needs to restart everything");
      //TODO restart everything
      return CAST_STATUS(BCAP_E_FAIL);
    }

    if (errorcode == 0x81501003)
    {
      ROS_INFO("motor is off, turn on");
      BCAP_HRESULT hr = bCapMotor(true);
      return CAST_STATUS(hr);
    }

    if (errorcode == 0x84201482)
    {
      // error: postion buffer is empty.
      //BCAP_HRESULT hr = bCapFillBuffer(); // this does not work
      // now completely restart bcap things
      BCAP_HRESULT hr;
      hr = bCapMotor(false);
      if (FAILED(hr)) {
        ROS_WARN("failed to turn off motor %02x", hr);
        return CAST_STATUS(hr);
      }
      hr = bCapGiveArm();
      if (FAILED(hr)) {
        ROS_WARN("failed to give arm %02x", hr);
        return CAST_STATUS(hr);
      }
      hr = bCapReleaseRobot();
      if (FAILED(hr)) {
        ROS_WARN("failed to release robot %02x", hr);
        return CAST_STATUS(hr);
      }
      hr = bCapControllerDisConnect();
      if (FAILED(hr)) {
        ROS_WARN("failed to disconnect controller %02x", hr);
        return CAST_STATUS(hr);
      }
      hr = bCapServerStop();
      if (FAILED(hr)) {
        ROS_WARN("failed to stop bcap service %02x", hr);
        return CAST_STATUS(hr);
      }
      boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
      hr = bCapClose();
      if (FAILED(hr)) {
        ROS_WARN("failed to close bcap socket %02x", hr);
        return CAST_STATUS(hr);
      }
      hr = bCapOpen();
      if (FAILED(hr)) {
        ROS_WARN("failed to open socket %02x", hr);
        return CAST_STATUS(hr);
      }
      setUDPTimeout(2, 0); // 2000msec
      hr = bCapServerStart();
      if (FAILED(hr)) {
        ROS_WARN("failed to start service %02x", hr);
        return CAST_STATUS(hr);
      }
      hr = bCapControllerConnect();
      if (FAILED(hr)) {
        ROS_WARN("failed to connect controller %02x", hr);
        return CAST_STATUS(hr);
      }
      hr = bCapClearError();
      if (FAILED(hr)) {
        ROS_WARN("failed to clear error %02x", hr);
        return CAST_STATUS(hr);
      }
      hr = bCapGetRobot();
      if (FAILED(hr)) {
        ROS_WARN("failed to get robot %02x", hr);
        return CAST_STATUS(hr);
      }
      hr = bCapTakearm();
      if (FAILED(hr)) {
        ROS_WARN("failed to take arm %02x", hr);
        return CAST_STATUS(hr);
      }
      hr = bCapMotor(true);
      if (FAILED(hr))
      {
        ROS_WARN("failed to turn on motor %02x", hr);
        return CAST_STATUS(hr);
      }
      hr = bCapRobotExecute("clearLog", "");
      if (FAILED(hr))
      {
        ROS_FATAL("failed to clear logging");
        return CAST_STATUS(hr);
      }
      hr = bCapSlvChangeMode(0x202);
      if (FAILED(hr))
      {
        ROS_FATAL("failed to change slave mode");
        return CAST_STATUS(hr);
      }
      hr = bCapFillBuffer();
      if (FAILED(hr))
      {
        ROS_FATAL("failed to fill bcap buffer");
        return CAST_STATUS(hr);
      }
      hr = bCapReflectRealState();
      if (FAILED(hr))
      {
        ROS_FATAL("failed to reflect real robot state into controller manager");
        return CAST_STATUS(hr);
      }
      setUDPTimeout((udp_timeout_ / 1000), (udp_timeout_ % 1000) * 1000);

      return CAST_STATUS(hr);
    }

    if (errorcode == 0x84201486)
    {
        u_int mode;
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        while(!g_quit_)
        {
            mode = bCapGetMode();
            if (mode != 3) {
              // user set controller to manual or teachcheck mode, wait until the user set back to auto
              ROS_WARN("waiting until you set back to auto");
              boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            } else {
              bool estop = bCapGetEmergencyStop();
              if (estop) {
                ROS_WARN("please turn off emergency stop");
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                continue;
              } else {
                break;
              }
            }
        }
        //TODO restart bcap server
        BCAP_HRESULT hr;
        hr = bCapControllerConnect();
        if (FAILED(hr)) {
          ROS_WARN("failed to connect controller %02x", hr);
          return CAST_STATUS(hr);
        }
        hr = bCapClearError();
        if (FAILED(hr)) {
          ROS_WARN("failed to clear error %02x", hr);
          return CAST_STATUS(hr);
        }
        hr = bCapGiveArm();
        if (FAILED(hr)) {
          ROS_WARN("failed to give arm %02x", hr);
          return CAST_STATUS(hr);
        }
        hr = bCapGetRobot();
        if (FAILED(hr)) {
          ROS_WARN("failed to get robot %02x", hr);
          return CAST_STATUS(hr);
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        hr = bCapTakearm();
        if (FAILED(hr)) {
          ROS_WARN("failed to take arm %02x", hr);
          return CAST_STATUS(hr);
        }
        hr = bCapMotor(true);
        if (FAILED(hr)) {
          ROS_WARN("failed to turn on motor %02x", hr);
          return CAST_STATUS(hr);
        }
        hr = bCapSlvChangeMode(0x202);
        if (FAILED(hr)) {
          ROS_WARN("failed to change to slvmode %02x", hr);
          return CAST_STATUS(hr);
        }
        hr = bCapFillBuffer();
        if (FAILED(hr)) {
          ROS_WARN("failed to fill buffer in slave mode");
          return CAST_STATUS(hr);
        }
        hr = bCapReflectRealState();
        if (FAILED(hr)) {
          ROS_WARN("failed to reflect real state");
          return CAST_STATUS(hr);
        }
        return CAST_STATUS(BCAP_S_OK);
    }
    if (errorcode == 0x83500121)
    {
      ROS_INFO("robot is not in slavemode, try to change it");
      BCAP_HRESULT hr;
      hr = bCapClearError();
      if (FAILED(hr)) return CAST_STATUS(hr);
      hr = bCapMotor(true);
      if (FAILED(hr)) {
        ROS_WARN("failed to turn on motor, cannot recover");
        return CAST_STATUS(hr);
      }
      hr = bCapRobotExecute("clearLog", "");
      if (FAILED(hr)) {
        ROS_WARN("failed to clear log");
        return CAST_STATUS(hr);
      }
      hr = bCapSlvChangeMode(0x202);
      if (FAILED(hr)) {
        ROS_WARN("failed to change slvmode, cannot recover");
        return CAST_STATUS(hr);
      }
      hr = bCapFillBuffer();
      if (FAILED(hr)) {
        ROS_WARN("failed to fill buffer in slave mode");
        return CAST_STATUS(hr);
      }
      hr = bCapReflectRealState();
      if (FAILED(hr)) {
        ROS_WARN("failed to reflect real state");
        return CAST_STATUS(hr);
      }
      return CAST_STATUS(hr);
    }
    if (errorcode == 0x83501032)
    {
      //TODO rethink the way to recover
      ROS_INFO("invalid command when the robot is in slave mode, disable slave mode");
      BCAP_HRESULT hr = bCapSlvChangeMode(0);
      return CAST_STATUS(hr);
    }

    if (errorcode == 0x81501025)
    {
      ROS_INFO("do not send message while an error is occuring");
      BCAP_HRESULT hr = bCapClearError();
      return CAST_STATUS(hr);
    }

    return CAST_STATUS(BCAP_S_OK);
#undef CAST_STATUS(hr)
  }
 
  void setUDPTimeout(long sec, long usec)
  {
#ifdef BCAP_CONNECTION_UDP
    struct timeval tv;
    tv.tv_sec = sec;
    tv.tv_usec = usec;
    if (setsockopt(iSockFD_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0)
    {
      ROS_WARN("Failed to set send timeout");
    }
    if (setsockopt(iSockFD_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
      ROS_WARN("Failed to set recv timeout");
    }
#endif
  }

  BCAP_HRESULT bCapServerStart()
  {
    BCAP_HRESULT hr = bCap_ServiceStart(iSockFD_); /* Start b-CAP service */
    return hr;
  }
  BCAP_HRESULT bCapServerStop()
  {
    BCAP_HRESULT hr = bCap_ServiceStop(iSockFD_); /* Start b-CAP service */
    return hr;
  }

  /**
   * @overridden
   */
  void initializeCM()
  {
    hw_ = new pr2_hardware_interface::HardwareInterface();
    hw_->addActuator(new pr2_hardware_interface::Actuator("j1_motor"));
    hw_->addActuator(new pr2_hardware_interface::Actuator("j2_motor"));
    hw_->addActuator(new pr2_hardware_interface::Actuator("j3_motor"));
    hw_->addActuator(new pr2_hardware_interface::Actuator("j4_motor"));
    hw_->addActuator(new pr2_hardware_interface::Actuator("j5_motor"));
    hw_->addActuator(new pr2_hardware_interface::Actuator("flange_motor"));
    // // Create controller manager
    //pr2_controller_manager::ControllerManager cm_(ec.hw_);
    cm_ = boost::shared_ptr<pr2_controller_manager::ControllerManager>(
        new pr2_controller_manager::ControllerManager(hw_));
  }

  /**
   * @overridden
   */
  void initializeHW()
  {
    // for denso connection
    if (!dryrunp_)
    {
      BCAP_HRESULT hr;
      ROS_INFO("bCapOpen");
      hr = bCapOpen();
      if(FAILED(hr)) exit(1);
      setUDPTimeout(2, 0); // 2000msec
      ROS_INFO("bCapServerStart");
      hr = bCapServerStart();
      if(FAILED(hr)) exit(1);
      ROS_INFO("bCapControllerConnect");
      hr = bCapControllerConnect();
      if(FAILED(hr)) exit(1);
      ROS_INFO("bCapClearError");
      hr = bCapClearError();
      if(FAILED(hr)) exit(1);
      ROS_INFO("bCapGetRobot");
      hr = bCapGetRobot();
      if(FAILED(hr)) exit(1);
      ROS_INFO("bCapTakearm");
      hr = bCapTakearm();
      if(FAILED(hr)) exit(1);
      ROS_INFO("bCapSetExternalSpeed");
      hr = bCapSetExternalSpeed(100.0);
      if(FAILED(hr)) exit(1);

      bCapInitializeVariableHandlers();

      {
        ROS_WARN("bCapMotor On");
        hr = bCapMotor(true);
        if (FAILED(hr))
        {
          u_int errorcode;
          std::string errormsg;
          errorcode = bCapGetErrorCode();
          bCapErrorDescription(errormsg);
          ROS_INFO("errormsg: %s", errormsg.c_str());
          ROS_WARN("failed to motor on, errorcode: %02x, errormsg: %s", errorcode, errormsg.c_str());
          //ROS_FATAL("failed to motor on");
          finalize();
          SAFE_EXIT(1);
        }
      }

      // enable logging
      hr = bCapRobotExecute("clearLog", "");
      if (FAILED(hr))
      {
        ROS_FATAL("failed to enable logging mode");
        SAFE_EXIT(1);
      }

      // enable logging
      // {
      //   BCAP_HRESULT hr = bCapRobotExecute("startLog", "");
      //   if (FAILED(hr)) {
      //     ROS_FATAL("failed to start logging mode");
      //     SAFE_EXIT(1);
      //   }
      // }

      hr = bCapSlvChangeMode(0x202);
      //BCAP_HRESULT hr = bCapSlvChangeMode((char*)"258"); // 0x102
      if (FAILED(hr))
      {
        ROS_FATAL("failed to change slave mode");
        SAFE_EXIT(1);
      }

      ROS_INFO("initialize bCap slave connection");
      hr = bCapFillBuffer();
      if (FAILED(hr))
      {
        ROS_FATAL("failed to fill bcap buffer");
        SAFE_EXIT(1);
      }
      ROS_INFO("bCap slave initialization done");

      hr = bCapReflectRealState();
      if (FAILED(hr))
      {
        ROS_FATAL("failed to reflect real robot state into controller manager");
        SAFE_EXIT(1);
      }
      setUDPTimeout((udp_timeout_ / 1000), (udp_timeout_ % 1000) * 1000);
    }
  }

  void initializeROS(ros::NodeHandle& node)
  {
    //TODO (Trivial) Since all of the tasks done within this method don't seem to be ROS-specific,
    //     moving them to somewhere else might make more sense.

    // Determine ip address of the robot's embedded machine.
    if (!node.getParam("server_ip", server_ip_address_))
    {
      server_ip_address_ = (char*)DEFAULT_SERVER_IP_ADDRESS;
    }

    // Determine the pre-set port number used to communicate the robot's embedded computer via bCap.
    if (!node.getParam("server_port", server_port_number_))
    {
      server_port_number_ = DEFAULT_SERVER_PORT_NUM;
    }

    ROS_INFO("server: %s:%d", server_ip_address_.c_str(), server_port_number_);

    // Determine the pre-set UDP timeout length.
    if (!node.getParam("udp_timeout", udp_timeout_))
    {
      udp_timeout_ = DEFAULT_UDP_TIMEOUT;
    }
    ROS_INFO("udp_timeout: %d micro sec", udp_timeout_);
  }

  void quitRequest()
  {
    ROS_INFO("denso_controller received a quit request");
    OpenController::quitRequest(); // call super class
    ros::shutdown();
  }
};

DensoController g_controller;
void quitRequested(int sigint)
{
  // do nothing
  ROS_WARN("request quit to denso controller using signal");
  g_controller.quitRequest();
}

int main(int argc, char *argv[])
{
  // Keep the kernel from swapping us out
  OpenControllersInterface::OpenController::initRT();

  // Initialize ROS and parse command-line arguments
  ros::init(argc, argv, "denso_controller",
            // quitRequested is used as a SIGINT handler instead of the handler prepared by ROS.
            ros::init_options::NoSigintHandler);
  // Parse options
  //g_options.program_ = argv[0];
  g_controller.parseArguments(argc, argv);
  if (optind < argc)
  {
    g_controller.Usage("Extra arguments");
  }

  // do these work?
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);
  int rv = 1;
  // Set up realtime context, start ROS processes, load robot description.
  g_controller.initialize();
  boost::thread t(&DensoController::start, &g_controller);
  {
    OpenControllersInterface::Finalizer finalizer(&g_controller);
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_WARN("ROS has been terminated");
    t.join();
    ROS_WARN("start thread has been terminated");
  }

  g_controller.cleanupPidFile();
  return 0;
}

