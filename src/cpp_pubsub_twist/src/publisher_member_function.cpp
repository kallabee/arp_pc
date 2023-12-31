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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class TwistPublisher : public rclcpp::Node
{
public:
  TwistPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TwistPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // auto message = std_msgs::msg::String();
    auto vel = geometry_msgs::msg::Twist();

    if(count_ > 10){
      vel.linear.x=1;
    }else{
      vel.linear.x=-1;
    }
    count_++;
    if(count_ > 20){
      count_=0;
    }
    
    vel.angular.z=1;

    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", vel.linear.x);
    publisher_->publish(vel);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistPublisher>());
  rclcpp::shutdown();
  return 0;
}
