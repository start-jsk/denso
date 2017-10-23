#include <execinfo.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include "b-Cap.h"
#define DEFAULT_SERVER_IP_ADDRESS               "133.11.216.196"                /* Your controller IP address */
#define DEFAULT_SERVER_PORT_NUM                 5007
#define DEFAULT_UDP_TIMEOUT (10 * 1000)
#define CLOCK_PRIO 0
#define CONTROL_PRIO 0
#include <boost/thread.hpp>

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

class DensoRobot
{
public:
  DensoRobot(): dryrunp_(false)
  {
    jntvalues_.resize(8);
  }

  virtual ~DensoRobot(){
    finalizeHW();
  }

public:
  BCAP_HRESULT bCapOpen()
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    hr = bCap_Open(server_ip_address_.c_str(), server_port_number_, &iSockFD_);
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
    hr = bCap_ControllerConnect(iSockFD_, (char*)"", (char*)"caoProv.DENSO.VRC",
        (char*)(server_ip_address_.c_str()), (char*)"", &lhController_);
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
    long lResult = 0;
    hr = bCap_ControllerExecute(iSockFD_, lhController_, (char*)"ClearError", (char*)"", &lResult);
    ROS_INFO("clearError %02x %02x", hr, lResult);
    return hr;
  }

  BCAP_HRESULT bCapGetRobot()
  {
    BCAP_HRESULT hr = BCAP_S_OK;
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
      //try {
      //    utf8::utf16to8((uint16_t*)errdesc_buffer_, (uint16_t*)errdesc_buffer_+strlen, std::back_inserter(errormsg));
      //} catch (utf8::invalid_utf16& e) {
      //}
      errormsg = errdesc_buffer_; //copy
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
    BCAP_HRESULT hr = BCAP_E_FAIL;
    //TODO: armgroup defines the size of jntvalues_
    // we should set it collectly (using wincaps?) and use it when you send TakeArm command
    hr = bCap_RobotExecute(iSockFD_, lhRobot_, "CurJnt", "", &jntvalues_[0]);
    for (int i = 0; i < num_joints_; i++) {
      jointvalues[i] = jntvalues_[i];
    }
    ROS_DEBUG_STREAM("curjoints: " << jointvalues);
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
    // struct timeval tv;
    // tv.tv_sec = 0;
    // tv.tv_usec = 1000 * 1;
    // setsockopt(iSockFD_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    extern int failed_to_send_packet;
    struct timespec tick, before;

    clock_gettime(CLOCK_MONOTONIC, &tick);
    BCAP_HRESULT hr = bCap_RobotExecute2(iSockFD_, lhRobot_, (char*)"slvMove", pose, result);

    if (FAILED(hr)) {
      ROS_WARN("failed to slvmove, errorcode: %02x", hr);
    }

    clock_gettime(CLOCK_MONOTONIC, &before);
    static const int NSEC_PER_SECOND = 1e+9;
    //static const int USEC_PER_SECOND = 1e6;
    double roundtrip = (before.tv_sec + double(before.tv_nsec) / NSEC_PER_SECOND)
        - (tick.tv_sec + double(tick.tv_nsec) / NSEC_PER_SECOND);

    prev_time_ = tick;

    if (failed_to_send_packet)
    {
      ROS_WARN("  roundtrip: %f", roundtrip);
      setUDPTimeout((udp_timeout_ / 1000), (udp_timeout_ % 1000) * 1000);
    }

    return hr;
  }

  BCAP_HRESULT bCapFillBuffer()
  {
      ROS_INFO("try to fill buffer");
      BCAP_HRESULT hr;
      std::vector<double> cur_jnt;
      cur_jnt.resize(num_joints_);
      for (int i = 0; i < 3; i++) {
        // sometimes robot returns [0,0,0...]
        hr = bCapCurJnt(cur_jnt);
      }
      BCAP_VARIANT vntPose, vntResult;
      vntPose.Type = VT_R8 | VT_ARRAY;
      vntPose.Arrays = 8;
      for (int i = 0; i < num_joints_; i++)
      {
        vntPose.Value.DoubleArray[i] = cur_jnt[i];
      }
      for (int i = num_joints_; i < 8; i++) {
        vntPose.Value.DoubleArray[i] = 0;
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

//  BCAP_HRESULT bCapReflectRealState() {
//      // fill ac
//      std::vector<double> cur_jnt;
//      BCAP_HRESULT hr;
//      hr = bCapCurJnt(cur_jnt);
//      int i = 0;
//      for (OpenControllersInterface::TransmissionIterator it = cm_->model_.transmissions_.begin();
//          it != cm_->model_.transmissions_.end(); ++it)
//      { // *** js and ac must be consistent
//        pr2_hardware_interface::Actuator *ac = hw_->getActuator((*it)->actuator_names_[0]);
//        bool is_prismatic = (*it)->actuator_names_[0].find("prismatic") != std::string::npos;
//        if (is_prismatic) {
//          ac->state_.position_ = cur_jnt[i] / 1000;
//        } else {
//          ac->state_.position_ = DEG2RAD(cur_jnt[i]); // degree -> radian
//        }
//        i++;
//      }
//      return hr;
//  }

  virtual void finalizeHW()
  {
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

  bool read(std::vector<double>& pos) {
    if (dryrunp_)
    {
      for (int i = 0; i < num_joints_; i++) {
        pos[i] = jntvalues_[i];
      }
      return true;
    }
    // fill ac
    BCAP_HRESULT hr;
    hr = bCapCurJnt(pos);
    if (FAILED(hr)) {
      return false;
    }
    return true;
  }
  bool write(const std::vector<double>& cmd, std::vector<double>& pos)
  {
    if (dryrunp_)
    {
      for (int i = 0; i < num_joints_; i++) {
        pos[i] = cmd[i];
        jntvalues_[i] = cmd[i];
      }
      return true;
    }
    // build vntPose
    BCAP_VARIANT vntPose;
    vntPose.Type = VT_R8 | VT_ARRAY;
    vntPose.Arrays = 8;
    std::memset(vntPose.Value.DoubleArray, 0, sizeof(double)*16);
    for (int i = 0; i < cmd.size(); i++) {
      vntPose.Value.DoubleArray[i] = cmd[i];
    }

    bool status;
    BCAP_VARIANT vntReturn;
    // check bcap status beforehand
    // u_int errorcode;
    // errorcode = bCapGetErrorCode();
    // if (FAILED(errorcode)) {
    //   ROS_WARN("bad status! robot needs recovery! hr: %02x", errorcode);
    //   //status.reset(new DensoRobotStatus(errorcode));
    //   status = false;
    // }
    BCAP_HRESULT hr = bCapRobotSlvMove(&vntPose, &vntReturn);
    ROS_DEBUG("slvmove hr: %02x", hr);
    if (hr == 0xF200501)
    {
      ROS_DEBUG("buf is filled, it's fine.");
      status = true;
    }
    else if (FAILED(hr))
    {
      if (hr == 0x83500121)
      {
        ROS_INFO("robot is not in slavemode, try to change it");
        BCAP_HRESULT hr;
        hr = bCapClearError();
        if (FAILED(hr)) {
          ROS_WARN("failed to clear error, cannot recover");
          return false;
        }
        hr = bCapRobotExecute("clearLog", "");
        if (FAILED(hr)) {
          ROS_WARN("failed to clear log");
          return false;
        }
        hr = bCapSlvChangeMode(0x102);
        if (FAILED(hr)) {
          ROS_WARN("failed to change slvmode, cannot recover");
          return false;
        }
        hr = bCapFillBuffer();
        if (FAILED(hr)) {
          ROS_WARN("failed to fill buffer in slave mode");
          return false;
        }
        hr = bCapCurJnt(pos);
        if (FAILED(hr)) {
          ROS_WARN("failed to read cur jnt");
          return false;
        }
      }
      return true;
    } else {
      for (int i = 0; i < pos.size(); i++) {
        pos[i] = vntReturn.Value.DoubleArray[i];
      }
      return true;
    }
  }

// #define CAST_STATUS(hr) \
//     boost::static_pointer_cast<OpenControllersInterface::ControllerStatus>(DensoRobotStatusPtr(new DensoRobotStatus(hr)))
// 
//   bool recoverController()
//   {
//     ROS_WARN("try to recover controller...");
//     u_int errorcode;
//     std::string errormsg;
// 
//     errorcode = bCapGetErrorCode();
//     bCapErrorDescription(errormsg);
//     
//     ROS_INFO("errormsg: %s", errormsg.c_str());
// 
//     if (errorcode == BCAP_E_UNEXPECTED)
//     {
//       ROS_FATAL("Unexpected Error occured. no way to recover!");
//       return CAST_STATUS(errorcode);
//     }
// 
//     if (errorcode >= 0x83204071 && errorcode <= 0x83204078)
//     {
//       ROS_INFO("joint angle is over the software limit.");
//       ROS_INFO("currently, there is no way to recover, quit.");
//       // TODO publish message and return healthy status so that an application can send recovery-trajectory.
//       return CAST_STATUS(BCAP_E_FAIL);
//     }
//     if (errorcode >= 0x84204081 && errorcode <= 0x842040A8)
//     {
//       ROS_INFO("joint angle velocity is over the software limit.");
//       ROS_INFO("currently, there is no way to recover, quit.");
//       // TODO publish message and return healthy status so that an application can send recovery-trajectory.
//       return CAST_STATUS(BCAP_E_FAIL);
//     }
//     if (errorcode >= 0x84204051 && errorcode <= 0x84204058)
//     {
//       ROS_INFO("joint angle velocity(sent) is over the software limit.");
//       //ROS_INFO("currently, there is no way to recover, quit.");
//       //return CAST_STATUS(BCAP_E_FAIL);
//       // TODO publish message and return healthy status so that an application can send recovery-trajectory.
//       BCAP_HRESULT hr;
//       hr = bCapClearError();
//       if (FAILED(hr)) {
//           ROS_WARN("failed to clear error %02x", hr);
//           return CAST_STATUS(hr);
//       }
//       hr = bCapGiveArm();
//       if (FAILED(hr)) {
//           ROS_WARN("failed to give arm %02x", hr);
//           return CAST_STATUS(hr);
//       }
//       //hr = bCap_RobotRelease(iSockFD_, lhRobot_); /* Release robot handle */
//       //if (FAILED(hr))
//       //{
//       //  ROS_WARN("failed to release the robot");
//       //}
//       //hr = bCapGetRobot();
//       //if (FAILED(hr)) {
//       //    ROS_WARN("failed to get robot %02x", hr);
//       //    return CAST_STATUS(hr);
//       //}
//       boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
//       hr = bCapTakearm();
//       if (FAILED(hr)) {
//           ROS_WARN("failed to take arm %02x", hr);
//           return CAST_STATUS(hr);
//       }
//       hr = bCapMotor(true);
//       if (FAILED(hr)) {
//           ROS_WARN("failed to turn on motor %02x", hr);
//           return CAST_STATUS(hr);
//       }
//       hr = bCapSlvChangeMode(0x202);
//       if (FAILED(hr)) {
//           ROS_WARN("failed to change to slvmode %02x", hr);
//           return CAST_STATUS(hr);
//       }
//       return CAST_STATUS(BCAP_S_OK);
//     }
//     if (errorcode >= 0x84204041 && errorcode <= 0x84204048)
//     {
//       ROS_INFO("joint angle acceleration is over the software limit.");
//       ROS_INFO("currently, there is no way to recover, quit.");
//       // TODO publish message and return healthy status so that an application can send recovery-trajectory.
//       return CAST_STATUS(BCAP_E_FAIL);
//     }
// 
// 
//     if (errorcode == 0x83204231)
//     {
//       ROS_INFO("invalid command was sent and everything stopped, needs to restart everything");
//       //TODO restart everything
//       return CAST_STATUS(BCAP_E_FAIL);
//     }
// 
//     if (errorcode == 0x81501003)
//     {
//       ROS_INFO("motor is off, turn on");
//       BCAP_HRESULT hr = bCapMotor(true);
//       return CAST_STATUS(hr);
//     }
// 
//     if (errorcode == 0x84201482)
//     {
//       // error: postion buffer is empty.
//       //BCAP_HRESULT hr = bCapFillBuffer(); // this does not work
//       // now completely restart bcap things
//       BCAP_HRESULT hr;
//       hr = bCapMotor(false);
//       if (FAILED(hr)) {
//         ROS_WARN("failed to turn off motor %02x", hr);
//         return CAST_STATUS(hr);
//       }
//       hr = bCapGiveArm();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to give arm %02x", hr);
//         return CAST_STATUS(hr);
//       }
//       hr = bCapReleaseRobot();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to release robot %02x", hr);
//         return CAST_STATUS(hr);
//       }
//       hr = bCapControllerDisConnect();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to disconnect controller %02x", hr);
//         return CAST_STATUS(hr);
//       }
//       hr = bCapServerStop();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to stop bcap service %02x", hr);
//         return CAST_STATUS(hr);
//       }
//       boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
//       hr = bCapClose();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to close bcap socket %02x", hr);
//         return CAST_STATUS(hr);
//       }
//       hr = bCapOpen();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to open socket %02x", hr);
//         return CAST_STATUS(hr);
//       }
//       setUDPTimeout(2, 0); // 2000msec
//       hr = bCapServerStart();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to start service %02x", hr);
//         return CAST_STATUS(hr);
//       }
//       hr = bCapControllerConnect();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to connect controller %02x", hr);
//         return CAST_STATUS(hr);
//       }
//       hr = bCapClearError();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to clear error %02x", hr);
//         return CAST_STATUS(hr);
//       }
//       hr = bCapGetRobot();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to get robot %02x", hr);
//         return CAST_STATUS(hr);
//       }
//       hr = bCapTakearm();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to take arm %02x", hr);
//         return CAST_STATUS(hr);
//       }
//       hr = bCapMotor(true);
//       if (FAILED(hr))
//       {
//         ROS_WARN("failed to turn on motor %02x", hr);
//         return CAST_STATUS(hr);
//       }
//       //hr = bCapRobotExecute("clearLog", "");
//       //if (FAILED(hr))
//       //{
//       //  ROS_FATAL("failed to clear logging");
//       //  return CAST_STATUS(hr);
//       //}
//       hr = bCapSlvChangeMode(0x202);
//       if (FAILED(hr))
//       {
//         ROS_FATAL("failed to change slave mode");
//         return CAST_STATUS(hr);
//       }
//       hr = bCapFillBuffer();
//       if (FAILED(hr))
//       {
//         ROS_FATAL("failed to fill bcap buffer");
//         return CAST_STATUS(hr);
//       }
//       hr = bCapReflectRealState();
//       if (FAILED(hr))
//       {
//         ROS_FATAL("failed to reflect real robot state into controller manager");
//         return CAST_STATUS(hr);
//       }
//       setUDPTimeout((udp_timeout_ / 1000), (udp_timeout_ % 1000) * 1000);
// 
//       return CAST_STATUS(hr);
//     }
// 
//     if (errorcode == 0x803022ef || errorcode == 0x84201486)
//     {
//         u_int mode;
//         boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
//         while(!g_quit_)
//         {
//             mode = bCapGetMode();
//             if (mode != 3) {
//               // user set controller to manual or teachcheck mode, wait until the user set back to auto
//               ROS_WARN("waiting until you set back to auto");
//               boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
//             } else {
//               bool estop = bCapGetEmergencyStop();
//               if (estop) {
//                 ROS_WARN("please turn off emergency stop");
//                 boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
//                 continue;
//               } else {
//                 break;
//               }
//             }
//         }
//         //TODO restart bcap server
//         BCAP_HRESULT hr;
//         hr = bCapControllerConnect();
//         if (FAILED(hr)) {
//           ROS_WARN("failed to connect controller %02x", hr);
//           return CAST_STATUS(hr);
//         }
//         hr = bCapClearError();
//         if (FAILED(hr)) {
//           ROS_WARN("failed to clear error %02x", hr);
//           return CAST_STATUS(hr);
//         }
//         hr = bCapGiveArm();
//         if (FAILED(hr)) {
//           ROS_WARN("failed to give arm %02x", hr);
//           return CAST_STATUS(hr);
//         }
//         hr = bCapGetRobot();
//         if (FAILED(hr)) {
//           ROS_WARN("failed to get robot %02x", hr);
//           return CAST_STATUS(hr);
//         }
//         boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
//         hr = bCapTakearm();
//         if (FAILED(hr)) {
//           ROS_WARN("failed to take arm %02x", hr);
//           return CAST_STATUS(hr);
//         }
//         hr = bCapMotor(true);
//         if (FAILED(hr)) {
//           ROS_WARN("failed to turn on motor %02x", hr);
//           return CAST_STATUS(hr);
//         }
//         hr = bCapSlvChangeMode(0x202);
//         if (FAILED(hr)) {
//           ROS_WARN("failed to change to slvmode %02x", hr);
//           return CAST_STATUS(hr);
//         }
//         hr = bCapFillBuffer();
//         if (FAILED(hr)) {
//           ROS_WARN("failed to fill buffer in slave mode");
//           return CAST_STATUS(hr);
//         }
//         hr = bCapReflectRealState();
//         if (FAILED(hr)) {
//           ROS_WARN("failed to reflect real state");
//           return CAST_STATUS(hr);
//         }
//         return CAST_STATUS(BCAP_S_OK);
//     }
//     if (errorcode == 0x83500121)
//     {
//       ROS_INFO("robot is not in slavemode, try to change it");
//       BCAP_HRESULT hr;
//       hr = bCapClearError();
//       if (FAILED(hr)) return CAST_STATUS(hr);
//       hr = bCapMotor(true);
//       if (FAILED(hr)) {
//         ROS_WARN("failed to turn on motor, cannot recover");
//         return CAST_STATUS(hr);
//       }
//       hr = bCapRobotExecute("clearLog", "");
//       if (FAILED(hr)) {
//         ROS_WARN("failed to clear log");
//         return CAST_STATUS(hr);
//       }
//       hr = bCapSlvChangeMode(0x202);
//       if (FAILED(hr)) {
//         ROS_WARN("failed to change slvmode, cannot recover");
//         return CAST_STATUS(hr);
//       }
//       hr = bCapFillBuffer();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to fill buffer in slave mode");
//         return CAST_STATUS(hr);
//       }
//       hr = bCapReflectRealState();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to reflect real state");
//         return CAST_STATUS(hr);
//       }
//       return CAST_STATUS(hr);
//     }
//     if (errorcode == 0x83501032)
//     {
//       //TODO rethink the way to recover
//       ROS_INFO("invalid command when the robot is in slave mode, disable slave mode");
//       BCAP_HRESULT hr = bCapSlvChangeMode(0);
//       return CAST_STATUS(hr);
//     }
// 
//     if (errorcode == 0x81501025)
//     {
//       ROS_INFO("do not send message while an error is occuring");
//       BCAP_HRESULT hr = bCapClearError();
//       return CAST_STATUS(hr);
//     }
// 
//     return CAST_STATUS(BCAP_S_OK);
//   }


//   OpenControllersInterface::ControllerStatusPtr moveGripper(double value) {
//     BCAP_HRESULT hr;
//     ROS_INFO("turn off slavemode");
//     hr = bCapSlvChangeMode(0x0);
//     if (FAILED(hr)) {
//       ROS_WARN("failed to change from slvmode to normal mode");
//       //return CAST_STATUS(hr);
//     }
//     std::vector<double> cur_jnt;
//     hr = bCapCurJnt(cur_jnt);
// 
//     //BCAP_VARIANT gripperparam, gripperresult;
//     //gripperparam.Type = VT_R4 | VT_ARRAY;
//     //gripperparam.Arrays = 2;
//     //gripperparam.Value.FloatArray[0] = 7;
//     //gripperparam.Value.FloatArray[1] = value;
//     //gripperparam.Type = VT_BSTR;
//     //strcpy((char*)gripperparam.Value.String, "(7,23)");
//     ROS_INFO("move gripper");
//     //hr = bCap_RobotExecute2(iSockFD_, lhRobot_, "DriveAEx", &gripperparam, &gripperresult);
// 
//     std::stringstream ss;
//     ss << "@P J(" << cur_jnt[0] << ","
//       << cur_jnt[1] << ","
//       << cur_jnt[2] << ","
//       << cur_jnt[3] << ","
//       << cur_jnt[4] << ","
//       << cur_jnt[5] << ") EXA((7," << value << "))";
// 
//     hr = bCap_RobotMove(iSockFD_, lhRobot_, 1, (char*) ss.str().c_str(), "NEXT");
//     //      @param  lComp           :       [in]  completion parameter
//     //      @param  pStrPose                :       [in]  Pose string in AsciiZ
//     //      @param  pstrOption              :       [in]  Option string in AsciiZ
//     if (SUCCEEDED(hr)) {
//       ROS_INFO("succeeded to move gripper");
//     } else {
//       ROS_INFO("failed to move gripper %02x", hr);
//     }
// 
//     boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
//     ROS_INFO("turn on slavemode");
//     bool b_success_turnonslavemode = false;
//     for (int i = 0; i < 10; i++) {
//       hr = bCapSlvChangeMode(0x202);
//       if (FAILED(hr)) {
//         ROS_WARN("failed to change slvmode %02x, %d", hr, i);
//         boost::this_thread::sleep(boost::posix_time::milliseconds(100));
//         continue;
//         //return CAST_STATUS(hr);
//       }
//       b_success_turnonslavemode = true;
//       break;
//     }
//     if (!b_success_turnonslavemode) {
//       ROS_WARN("COMPLETELY failed to change slvmode", hr);
//     }
//     for (int i = 0; i < 5; i++) {
//       hr = bCapFillBuffer();
//       if (FAILED(hr)) {
//         ROS_WARN("failed to fill buffer in slave mode %02x", hr);
//         //return CAST_STATUS(hr);
//         continue;
//       }
//       break;
//     }
//     ROS_INFO("reflect real state");
//     hr = bCapReflectRealState();
//     if (FAILED(hr)) {
//       ROS_WARN("failed to reflect real state %02x", hr);
//       //return CAST_STATUS(hr);
//     }
//     return CAST_STATUS(hr);
//   }
// #undef CAST_STATUS(hr)

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

  bool initialize(ros::NodeHandle& node, int num_joints)
  {
    // Set dryrunp_ to true,  when no real robot exists
    node.getParam("dryrun", dryrunp_);
    if (dryrunp_)
    {
      ROS_WARN("running with loopback mode");
    }

    num_joints_ = num_joints;
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

    if (!dryrunp_)
    {
      ROS_INFO("server: %s:%d", server_ip_address_.c_str(), server_port_number_);
    }

    // Determine the pre-set UDP timeout length.
    if (!node.getParam("udp_timeout", udp_timeout_))
    {
      udp_timeout_ = DEFAULT_UDP_TIMEOUT;
    }
    if (!dryrunp_)
    {
      ROS_INFO("udp_timeout: %d micro sec", udp_timeout_);
    }
    if (!dryrunp_)
    {
      BCAP_HRESULT hr;
      ROS_INFO("bCapOpen");
      hr = bCapOpen();
      if(FAILED(hr))
      {
        ROS_FATAL("failed to open bCap socket. Exitting...");
        ROS_FATAL("make sure that your robot is connected to %s:%d", server_ip_address_.c_str(), server_port_number_);
        return false;
      }
      setUDPTimeout(2, 0); // 2000msec
      ROS_INFO("bCapServerStart");
      hr = bCapServerStart();
      if(FAILED(hr))
      {
        ROS_FATAL("failed to start bCap server. Exitting...");
        return false;
      }
      ROS_INFO("bCapControllerConnect");
      hr = bCapControllerConnect();
      if(FAILED(hr))
      {
        ROS_FATAL("failed to connect bCap controller. Exitting...");
        return false;
      }
      ROS_INFO("bCapClearError");
      hr = bCapClearError();
      if(FAILED(hr))
      {
        ROS_FATAL("failed to clear bCap error. Exitting...");
        return false;
      }
      ROS_INFO("bCapGetRobot");
      hr = bCapGetRobot();
      if(FAILED(hr))
      {
        ROS_FATAL("failed to get bCap robot. Exitting...");
        return false;
      }
      ROS_INFO("bCapTakearm");
      hr = bCapTakearm();
      if(FAILED(hr))
      {
        ROS_FATAL("failed to take bCap arm. Exitting...");
        return false;
      }
      ROS_INFO("bCapSetExternalSpeed");
      hr = bCapSetExternalSpeed(100.0);
      if(FAILED(hr))
      {
        ROS_FATAL("failed to set external speed. Exitting...");
        return false;
      }

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
          //finalize();
          return false;
        }
      }

      // enable logging
      hr = bCapRobotExecute("clearLog", "");
      if (FAILED(hr))
      {
        ROS_FATAL("failed to enable logging mode");
        return false;
      }

      // enable logging
      // {
      //   BCAP_HRESULT hr = bCapRobotExecute("startLog", "");
      //   if (FAILED(hr)) {
      //     ROS_FATAL("failed to start logging mode");
      //     return;
      //   }
      // }

      hr = bCapSlvChangeMode(0x102);
      //hr = bCapSlvChangeMode((char*)"258"); // 0x102
      //hr = bCapSlvChangeMode(0x102); // 0x102
      if (FAILED(hr))
      {
        ROS_FATAL("failed to change slave mode");
        return false;
      }

      ROS_INFO("initialize bCap slave connection");
      hr = bCapFillBuffer();
      if (FAILED(hr))
      {
        ROS_FATAL("failed to fill bcap buffer");
        return false;
      }
      ROS_INFO("bCap slave initialization done");

//      hr = bCapReflectRealState();
//      if (FAILED(hr))
//      {
//        ROS_FATAL("failed to reflect real robot state into controller manager");
//        return;
//      }
      setUDPTimeout((udp_timeout_ / 1000), (udp_timeout_ % 1000) * 1000);
      return true;
    } else {
      return true;
    }
  }

//  void quitRequest()
//  {
//    ROS_INFO("denso_controller received a quit request");
//    OpenController::quitRequest(); // call super class
//    ros::shutdown();
//  }

private:
  std::vector<double> jntvalues_;
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
  bool dryrunp_;
  int num_joints_;

  struct timespec prev_time_;
};
