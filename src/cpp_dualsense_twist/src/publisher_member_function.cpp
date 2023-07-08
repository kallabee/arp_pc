// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "camera_actuator_interfaces/msg/camera_actuator.hpp"
#include "hand_actuator_interfaces/msg/hand_actuator.hpp"

#include "../include/ds5w.h"
#include "publisher_member_function.h"
#include <Windows.h>

using namespace std::chrono_literals;

dead_time_manager::dead_time_manager(const std::chrono::milliseconds dead_time) : dead_time(dead_time), in_dead_time(false)
{
}

bool dead_time_manager::is_in_dead_time(bool &changed)
{
  if (in_dead_time)
  {
    auto elapsed = std::chrono::system_clock::now() - start_time;
    if (elapsed > dead_time)
    {
      // printf("Dead time ended.\n");
      in_dead_time = false;
      changed = true;
    }
  }
  return in_dead_time;
}

void dead_time_manager::start()
{
  start_time = std::chrono::system_clock::now();
  in_dead_time = true;
}

double limit(double angle, double angle_limit, double &new_angle)
{
  if (angle < -angle_limit)
  {
    new_angle = -angle_limit;
    return false;
  }
  else if (angle > angle_limit)
  {
    new_angle = angle_limit;
    return false;
  }
  new_angle = angle;
  return true;
}

class CamAngleSpeedConverter
{
public:
  CamAngleSpeedConverter(double pan_speed_coef, double tilt_speed_coef, double max_pan, double max_tilt) : pan(0), tilt(0), pan_speed_coef(pan_speed_coef), tilt_speed_coef(tilt_speed_coef), max_pan(max_pan), max_tilt(max_tilt)
  {
  }

  bool ConvertAngleSpeed(double pan_speed, double tilt_speed, double &pan, double &tilt)
  {
    double desired_pan = this->pan + this->pan_speed_coef * pan_speed;
    double desired_tilt = this->tilt + this->tilt_speed_coef * tilt_speed;
    // printf("ConvertAngleSpeed pan, tilt = %f, %f, ", desired_pan, desired_tilt);

    bool r = limit(desired_pan, this->max_pan, this->pan) && limit(desired_tilt, this->max_tilt, this->tilt);
    pan = this->pan;
    tilt = this->tilt;

    // printf("after limit = %+.1ff, %+.1f,", this->pan, this->tilt);
    return r;
  }

  bool ResetAngle(double &pan, double &tilt)
  {
    this->pan = pan = 0;
    this->tilt = tilt = 0;
    return true;
  }

  void Normalize(double pan, double tilt, double &n_pan, double &n_tilt)
  {
    n_pan = pan / this->max_pan;
    n_tilt = tilt / this->max_tilt;
  }

private:
  double pan;
  double tilt;
  const double pan_speed_coef;
  const double tilt_speed_coef;
  const double max_pan;
  const double max_tilt;
};

class CameraFocusValueController
{
public:
  CameraFocusValueController(int focus_home, int focus_step, int focus_min, int focus_max)
      : focus(focus_home), focus_home(focus_home), focus_step(focus_step), focus_min(focus_min), focus_max(focus_max)
  {
  }

  bool MoveForFar(int &focus)
  {
    bool r = true;
    focus = this->focus - this->focus_step;
    if (focus < this->focus_min)
    {
      focus = this->focus_min;
      r = false;
    }
    this->focus = focus;
    return r;
  }

  bool MoveForNear(int &focus)
  {
    bool r = true;
    focus = this->focus + this->focus_step;
    if (focus > this->focus_max)
    {
      focus = this->focus_max;
      r = false;
    }
    this->focus = focus;
    return r;
  }

  bool MoveToHome(int &focus)
  {
    this->focus = focus = this->focus_home;
    return true;
  }

  int GetFocus()
  {
    return this->focus;
  }

private:
  int focus;
  const int focus_home;
  const int focus_step;
  const int focus_min;
  const int focus_max;
};

class TwistPublisher : public rclcpp::Node
{
public:
  TwistPublisher()
      : Node("minimal_publisher"), count_(0)
  {
    if (this->connect_ds5() != 0)
    {
      RCLCPP_INFO(this->get_logger(), "Failed to connect Dual Sense.");
    }

    const std::string root_node = "/turtle1";
    const int queue_num = 1;

    this->enableAcc = false;

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(root_node + "/cmd_vel", queue_num);
    hand_act_pub_ = this->create_publisher<hand_actuator_interfaces::msg::HandActuator>(root_node + "/hand_act", queue_num);
    cam_act_pub_ = this->create_publisher<camera_actuator_interfaces::msg::CameraActuator>(root_node + "/cam_act", queue_num);

    const double pan_tilt_speed_coef = 2.0; // [degre / s?]
    const double pan_tilt_max_angle = 60;   // [degree]
    this->csc = std::make_unique<CamAngleSpeedConverter>(pan_tilt_speed_coef, pan_tilt_speed_coef, pan_tilt_max_angle, pan_tilt_max_angle);
    this->cfvc = std::make_unique<CameraFocusValueController>(512, 50, 0, 1023);

    timer_ = this->create_wall_timer(
        50ms, std::bind(&TwistPublisher::timer_callback, this));
  }

  ~TwistPublisher()
  {
    DS5W::freeDeviceContext(&con);
  }

private:
  int connect_ds5()
  {
    // Array of controller infos
    DS5W::DeviceEnumInfo infos[16];

    // Number of controllers found
    unsigned int controllersCount = 0;

    // Call enumerate function and switch on return value
    switch (DS5W::enumDevices(infos, 16, &controllersCount))
    {
    case DS5W_OK:
      // The buffer was not big enough. Ignore for now
    case DS5W_E_INSUFFICIENT_BUFFER:
      break;

      // Any other error will terminate the application
    default:
      // Insert your error handling
      return -1;
    }

    // Check number of controllers
    if (!controllersCount)
    {
      RCLCPP_INFO(this->get_logger(), "Invalid count of controllers was found.");
      return -1;
    }

    // Init controller and close application if failed
    if (DS5W_FAILED(DS5W::initDeviceContext(&infos[0], &(this->con))))
    {
      RCLCPP_INFO(this->get_logger(), "Failed to initialize device context.");
      return -1;
    }

    return 0;
  }

  // This coordinate is not same as ROS's coordinate.
  // x in arguments means Square to Circle axis.
  // y in arguments means Cross to Triangle axis.
  // return true if any buttom is pressed.
  static bool parseButtonsForCamera(const DS5W::DS5InputState &inState, char &x, char &y, bool &angleReset, int &ir_cut)
  {
    static dead_time_manager dtm_ir(std::chrono::milliseconds(200));
    static dead_time_manager dtm_pt(std::chrono::milliseconds(200));
    bool button_pressed = false;
    // printf("inState.buttonsA %02x, inState.buttonsAndDpad %02x\n", inState.buttonsA, inState.buttonsAndDpad);

    // IR cut filter on/off/auto switch
    if (inState.buttonsA & DS5W_ISTATE_BTN_A_SELECT)
    {
      bool dummy;
      if (!dtm_ir.is_in_dead_time(dummy))
      {
        ir_cut++;
        if (ir_cut >= 2)
        {
          ir_cut = 0;
        }
        button_pressed = true;
        dtm_ir.start();
        printf("IR cut filter mode is changed to %d\n", ir_cut);
      }
      printf("Share button is pressed.\n");
    }

    // Pan & Tilt
    if (inState.buttonsA & DS5W_ISTATE_BTN_A_MENU)
    {
      bool dummy;
      if (!dtm_pt.is_in_dead_time(dummy))
      {
        angleReset = true;
        button_pressed = true;
        dtm_pt.start();
      }
    }
    else
    {
      angleReset = false;

      // Pan
      if (inState.buttonsAndDpad & DS5W_ISTATE_BTX_CIRCLE)
      {
        x = 1;
        button_pressed = true;
      }
      else if (inState.buttonsAndDpad & DS5W_ISTATE_BTX_SQUARE)
      {
        x = -1;
        button_pressed = true;
      }
      else
      {
        x = 0;
      }

      // Tilt
      if (inState.buttonsAndDpad & DS5W_ISTATE_BTX_TRIANGLE)
      {
        y = 1;
        button_pressed = true;
      }
      else if (inState.buttonsAndDpad & DS5W_ISTATE_BTX_CROSS)
      {
        y = -1;
        button_pressed = true;
      }
      else
      {
        y = 0;
      }
    }

    return button_pressed;
  }

  // static bool parseButtonsForCamera(const DS5W::DS5InputState &inState, char &x, char &y, char &f, bool &angleReset, bool &focusReset)
  // {
  //   bool button_pressed = false;
  //   // printf("inState.buttonsA %02x, inState.buttonsAndDpad %02x\n", inState.buttonsA, inState.buttonsAndDpad);

  //   // Pan & Tilt
  //   if (inState.buttonsA & DS5W_ISTATE_BTN_A_MENU)
  //   {
  //     angleReset = true;
  //     button_pressed = true;
  //   }
  //   else
  //   {
  //     angleReset = false;

  //     // Pan
  //     if (inState.buttonsAndDpad & DS5W_ISTATE_BTX_CIRCLE)
  //     {
  //       x = 1;
  //       button_pressed = true;
  //     }
  //     else if (inState.buttonsAndDpad & DS5W_ISTATE_BTX_SQUARE)
  //     {
  //       x = -1;
  //       button_pressed = true;
  //     }
  //     else
  //     {
  //       x = 0;
  //     }

  //     // Tilt
  //     if (inState.buttonsAndDpad & DS5W_ISTATE_BTX_TRIANGLE)
  //     {
  //       y = 1;
  //       button_pressed = true;
  //     }
  //     else if (inState.buttonsAndDpad & DS5W_ISTATE_BTX_CROSS)
  //     {
  //       y = -1;
  //       button_pressed = true;
  //     }
  //     else
  //     {
  //       y = 0;
  //     }
  //   }

  //   // Focus
  //   if (inState.buttonsA & DS5W_ISTATE_BTN_A_SELECT) // ドキュメントではShareと書いてあるが、コードにはないので…
  //   {
  //     focusReset = true;
  //     button_pressed = true;
  //   }
  //   else
  //   {
  //     focusReset = false;

  //     if (inState.buttonsAndDpad & DS5W_ISTATE_DPAD_UP)
  //     {
  //       f = 1;
  //       button_pressed = true;
  //     }
  //     else if (inState.buttonsAndDpad & DS5W_ISTATE_DPAD_DOWN)
  //     {
  //       f = -1;
  //       button_pressed = true;
  //     }
  //     else
  //     {
  //       f = 0;
  //     }
  //   }

  //   return button_pressed;
  // }

  static void parseDpadButtons(const DS5W::DS5InputState &inState, char &x, char &y)
  {
    if (inState.buttonsAndDpad & DS5W_ISTATE_DPAD_RIGHT)
    {
      x = 1;
    }
    else if (inState.buttonsAndDpad & DS5W_ISTATE_DPAD_LEFT)
    {
      x = -1;
    }
    else
    {
      x = 0;
    }

    if (inState.buttonsAndDpad & DS5W_ISTATE_DPAD_UP)
    {
      y = 1;
    }
    else if (inState.buttonsAndDpad & DS5W_ISTATE_DPAD_DOWN)
    {
      y = -1;
    }
    else
    {
      y = 0;
    }
  }

  static void parseRLButtons(const DS5W::DS5InputState &inState, float &roll, float &grab)
  {
    roll = (float)((inState.rightTrigger - inState.leftTrigger) / 2 / TwistPublisher::scale);

    char gp = inState.buttonsA & DS5W_ISTATE_BTN_A_RIGHT_BUMPER ? 1 : 0;
    char gm = inState.buttonsA & DS5W_ISTATE_BTN_A_LEFT_BUMPER ? 1 : 0;
    grab = (float)(gp - gm);
  }

  static float normalize_acc(const short &v)
  {
    float r = (float)(v / std::pow(2, 13));
    if (r < -1)
    {
      r = -1;
    }
    else if (r > 1)
    {
      r = 1;
    }
    return r;
  }

  static geometry_msgs::msg::Twist extractTwist(const DS5W::DS5InputState &inState, bool &enableAcc)
  {
    auto vel = geometry_msgs::msg::Twist();
    static bool enableAccInDeadTime = false;
    // static auto start = std::chrono::system_clock::now(); // Value doesn't have meaning. It will be updated.
    // const std::chrono::milliseconds dead_time(200);
    static dead_time_manager dtm(std::chrono::milliseconds(200));
    bool dead_time_changed;

    // Linear
    vel.linear.x = inState.leftStick.y / TwistPublisher::scale;
    vel.linear.y = -inState.rightStick.x / TwistPublisher::scale;
    vel.linear.z = inState.rightStick.y / TwistPublisher::scale;

    // Angular
    if (inState.buttonsB & DS5W_ISTATE_BTN_B_MIC_BUTTON)
    {
      // if (enableAccInDeadTime)
      // {
      //   if (std::chrono::system_clock::now() - start > dead_time)
      //   {
      //     enableAccInDeadTime = false;
      //   }
      // }
      // else
      // {
      //   enableAcc = !enableAcc;
      //   enableAccInDeadTime = true;
      //   start = std::chrono::system_clock::now();
      // }
      if (!dtm.is_in_dead_time(dead_time_changed))
      {
        enableAcc = !enableAcc;
        dtm.start();
      }
    }
    char dp_x, dp_y;
    TwistPublisher::parseDpadButtons(inState, dp_x, dp_y);
    if (dp_x != 0 || dp_y != 0)
    {
      vel.angular.x = 1.0 * dp_x;
      vel.angular.y = 1.0 * dp_y;
    }
    else if (enableAcc)
    {
      // Accelerometer and gyroscope difinitions in the library is vice versa. It must be bug.
      vel.angular.x = normalize_acc(-inState.gyroscope.x);
      vel.angular.y = normalize_acc(inState.gyroscope.z);

      // const auto &a = inState.accelerometer;
      // const auto &g = inState.gyroscope;
      // printf("Acc x, y, z, Gyro x, y, z = %+6d, %+6d, %+6d, %+6d, %+6d, %+6d\n", a.x, a.y, a.z, g.x, g.y, g.z);

      // int t = enableAcc ? 1 : 0;
      // RCLCPP_INFO(this->get_logger(), "Twist: tilt and roll control by accelerometer is changed to %d", t);
    }

    vel.angular.z = -inState.leftStick.x / TwistPublisher::scale;

    auto hoge = inState.accelerometer.y;
    auto poge = inState.accelerometer.x;

    const auto &a = inState.accelerometer;
    const auto &g = inState.gyroscope;
    // printf("Acc x, y, z, Gyro x, y, z = %+6.2f, %+6.2f, %+6.2f, %+6.2f, %+6.2f, %+6.2f\n", a.x, a.y, a.z, g.x, g.y, g.z);

    return vel;
  }

  static bool isTwistActive(const geometry_msgs::msg::Twist &vel)
  {
    const auto th = 0.05;
    auto dz = [&](const double &v) // Check if the value is in the dead zone. v is in -1.0 to 1.0.
    {
      return th >= v && -th <= v;
    };

    return !dz(vel.linear.x) || !dz(vel.linear.y) || !dz(vel.linear.z) || !dz(vel.angular.x) || !dz(vel.angular.y) || !dz(vel.angular.z);
  }

  static hand_actuator_interfaces::msg::HandActuator extractHandAct(const DS5W::DS5InputState &inState)
  {
    auto hand_act = hand_actuator_interfaces::msg::HandActuator();
    parseRLButtons(inState, hand_act.roll, hand_act.grab);

    return hand_act;
  }

  static bool isHandActActive(const hand_actuator_interfaces::msg::HandActuator &hand_act)
  {
    return hand_act.grab != 0 || hand_act.roll != 0;
  }

  static void getZoomAndFocus(const DS5W::DS5InputState &inState, float &zoom, float &focus, bool &updated)
  {
    static unsigned int last_x1 = 0;
    static unsigned int last_y1 = 0;
    static dead_time_manager dtm(std::chrono::milliseconds(500));
    bool dead_time_changed;
    // static bool in_dead_time = false;
    // static auto start = std::chrono::system_clock::now(); // Value doesn't have meaning. It will be updated.
    // const std::chrono::milliseconds dead_time(500);
    const float zoom_reset = 0.5;
    const float focus_reset = 0.5;

    auto x1 = inState.touchPoint1.x;
    auto y1 = inState.touchPoint1.y;

    // if (in_dead_time)
    // {
    //   auto elapsed = std::chrono::system_clock::now() - start;
    //   if (elapsed > dead_time)
    //   {
    //     // printf("Dead time ended.\n");
    //     in_dead_time = false;
    //     updated = false;
    //     last_x1 = x1;
    //     last_y1 = y1;
    //     return;
    //   }
    //   else
    //   {
    //     // printf("New zoom and focus value are ignored because it is in dead time.\n");
    //     updated = false;
    //     return;
    //   }
    // }

    if (dtm.is_in_dead_time(dead_time_changed))
    {
      if (dead_time_changed)
      {
        // printf("Dead time ended.\n");
        last_x1 = x1;
        last_y1 = y1;
      }
      else
      {
        // printf("New zoom and focus value are ignored because it is in dead time.\n");
      }
      updated = false;
      return;
    }

    if (inState.buttonsB & DS5W_ISTATE_BTN_B_PAD_BUTTON)
    {
      // Reset.
      zoom = zoom_reset;
      focus = focus_reset;
      // last_x1 = (unsigned int)(zoom_reset * 1920);
      // last_y1 = (unsigned int)(focus_reset * 1080);
      dtm.start();
      // start = std::chrono::system_clock::now();
      // in_dead_time = true;
      updated = true;
    }
    else
    {

      // auto x2 = inState.touchPoint2.x;
      // auto y2 = inState.touchPoint2.y;
      // printf("x1, y1, x2, y2 = %4d, %4d, %4d, %4d\n", x1, y1, x2, y2);

      if (last_x1 != x1 || last_y1 != y1)
      {
        updated = true;
      }
      else
      {
        updated = false;
      }

      last_x1 = x1;
      last_y1 = y1;

      zoom = (float)x1 / 1980;
      focus = (float)y1 / 1080;
    }
  }

  camera_actuator_interfaces::msg::CameraActuator extractCamAct(const DS5W::DS5InputState &inState, bool &buttom_pressed)
  {
    auto cam_act = camera_actuator_interfaces::msg::CameraActuator();
    double pan;
    double tilt;
    char psb_x, psb_y;
    bool angle_reset;
    int ir_cut;

    // char focus_sign;
    // bool focus_reset;
    // int focus;
    // buttom_pressed = TwistPublisher::parseButtonsForCamera(inState, psb_x, psb_y, focus_sign, angle_reset, focus_reset);

    buttom_pressed = TwistPublisher::parseButtonsForCamera(inState, psb_x, psb_y, angle_reset, ir_cut);
    // printf("psb_x, psb_y, reset = %+03d, %+03d, %d, ", psb_x, psb_y, reset);

    if (angle_reset)
    {
      // printf("reset true.");
      this->csc->ResetAngle(pan, tilt);
    }
    else
    {
      // printf("reset false.");
      auto pan_speed = (float)(1.0 * psb_x);
      auto tilt_speed = (float)(1.0 * psb_y);

      this->csc->ConvertAngleSpeed(pan_speed, tilt_speed, pan, tilt);
    }
    // printf("pan, tilt = %+.1f, %+.1f, ", pan, tilt);
    double n_pan, n_tilt;
    this->csc->Normalize(pan, tilt, n_pan, n_tilt);
    cam_act.pan = -(float)n_pan;
    cam_act.tilt = -(float)n_tilt;

    cam_act.ir_cut = ir_cut;

    // if (focus_reset)
    // {
    //   this->cfvc->MoveToHome(focus);
    // }
    // else
    // {
    //   if (focus_sign < 0)
    //   {
    //     this->cfvc->MoveForNear(focus);
    //   }
    //   else if (focus_sign > 0)
    //   {
    //     this->cfvc->MoveForFar(focus);
    //   }
    //   else
    //   {
    //     // Do nothing.
    //   }
    // }
    // cam_act.focus = this->cfvc->GetFocus();

    // float zoom, focus;
    // bool updated;
    // getZoomAndFocus(inState, zoom, focus, updated);
    // cam_act.zoom = zoom;
    // cam_act.focus = focus;

    bool updated;
    getZoomAndFocus(inState, cam_act.zoom, cam_act.focus, updated);

    buttom_pressed |= updated;
    // printf("buttom_pressed = %d, updated = %d\n", buttom_pressed, updated);

    return cam_act;
  }

  void timer_callback()
  {
    // Input state
    DS5W::DS5InputState inState;
    auto vel = geometry_msgs::msg::Twist();
    auto hand_act = hand_actuator_interfaces::msg::HandActuator();
    bool cam_button_pressed;
    auto cam_act = camera_actuator_interfaces::msg::CameraActuator();

    // Create struct and zero it
    DS5W::DS5OutputState outState;
    ZeroMemory(&outState, sizeof(DS5W::DS5OutputState));

    // Retrieve data
    if (DS5W_SUCCESS(DS5W::getDeviceInputState(&(this->con), &inState)))
    {
      // Check for the Logo button
      if (inState.buttonsB & DS5W_ISTATE_BTN_B_PLAYSTATION_LOGO)
      {
        // Break from while loop
        // break;
        RCLCPP_INFO(this->get_logger(), "Byebye.");
        // ToDo stop program
      }

      vel = TwistPublisher::extractTwist(inState, this->enableAcc);
      // Chage LED state.
      if (this->enableAcc)
      {
        outState.microphoneLed = DS5W::MicLed::ON;
      }
      else
      {
        outState.microphoneLed = DS5W::MicLed::OFF;
      }

      hand_act = TwistPublisher::extractHandAct(inState);
      cam_act = TwistPublisher::extractCamAct(inState, cam_button_pressed);

      // Set output data
      outState.leftRumble = inState.leftTrigger;
      outState.rightRumble = inState.rightTrigger;

      // Send output to the controller
      DS5W::setDeviceOutputState(&(this->con), &outState);
      // RCLCPP_INFO(this->get_logger(), "In loop...");
    }

    if (isTwistActive(vel))
    {
      twist_pub_->publish(vel);
      const auto &l = vel.linear;
      const auto &a = vel.angular;
      RCLCPP_INFO(this->get_logger(), "Twist: lin x, y, z, ang x, y, z = %+.2f, %+.2f, %+.2f, %+6.2f, %+6.2f, %+6.2f", l.x, l.y, l.z, a.x, a.y, a.z);
    }
    if (isHandActActive(hand_act))
    {
      hand_act_pub_->publish(hand_act);
      RCLCPP_INFO(this->get_logger(), "HandAct: roll, grab = %+.2f, %+.2f", hand_act.roll, hand_act.grab);
    }
    if (cam_button_pressed)
    {
      cam_act_pub_->publish(cam_act);
      RCLCPP_INFO(this->get_logger(), "CamAct: pan, tilt, zoom, focus = %+.2f, %+.2f, %.2f, %.2f", cam_act.pan, cam_act.tilt, cam_act.zoom, cam_act.focus);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<hand_actuator_interfaces::msg::HandActuator>::SharedPtr hand_act_pub_;
  rclcpp::Publisher<camera_actuator_interfaces::msg::CameraActuator>::SharedPtr cam_act_pub_;

  std::unique_ptr<CamAngleSpeedConverter> csc;
  std::unique_ptr<CameraFocusValueController> cfvc;

  size_t count_;
  bool enableAcc;

  // Context for controller
  DS5W::DeviceContext con;
  static const double scale;
};

const double TwistPublisher::scale = 128;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistPublisher>());
  rclcpp::shutdown();
  return 0;
}
