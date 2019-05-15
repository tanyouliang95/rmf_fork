/*
  Created by : Pallavi
  Created on : 25-09-2018
  Description: This file subscribes to the ROS2 topic and if there is an open door command over the topic, it sets the GPIO pin on the Pi
*/

#ifndef PATIENTDEVICE_HPP
#define PATIENTDEVICE_HPP

#include <iostream>
#include <string>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "hope_msgs/msg/call_button_state.hpp"
#include "hope_msgs/msg/device_state.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

#ifndef BCM2835
#define BCM2835
#endif

#ifdef BCM2835
#include <bcm2835.h>
#endif

using std::placeholders::_1;

#define GPIO_OUTPIN RPI_BPLUS_GPIO_J8_11
#define INPUT_PIN RPI_BPLUS_GPIO_J8_12

class PatientDeviceController : public rclcpp::Node
{
public:
  PatientDeviceController();
  virtual ~PatientDeviceController();
  void param_initialize();
  void read_call_button_callback();
  void publish_call_button_state();
  void set_button_state_flag(const hope_msgs::msg::DeviceState::SharedPtr msg);
  
  //WiringPi Libraries
  void gpiosetup();
  void gpiosetpin();
  void gpioresetpin();
  void gpioon();

private:
  rclcpp::Publisher<hope_msgs::msg::CallButtonState>::SharedPtr call_button_state_publisher_;
  rclcpp::Subscription<hope_msgs::msg::DeviceState>::SharedPtr call_state_subscription_;

  bool button_pressed = false;
  std::string patient_device;
  bool button_state_ = false;
  bool pressed_first_time = false;
  hope_msgs::msg::CallButtonState::SharedPtr button_state;
  rclcpp::TimerBase::SharedPtr call_publish_timer_;
  rclcpp::TimerBase::SharedPtr call_button_state_timer_;

};

#endif
