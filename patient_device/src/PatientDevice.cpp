/*
  Created by : Pallavi
  Created on : 25-09-2018
  Description: This file subscribes to the ROS2 topic and if there is an open door command over the topic, it sets the GPIO pin on the Pi
*/

#include "patient_device/PatientDevice.hpp"

using namespace std;
using std::placeholders::_1;

PatientDeviceController::PatientDeviceController() : Node("patient_device_controller")
{
#ifdef BCM2835
  gpiosetup();
#endif
  button_state = std::make_shared<hope_msgs::msg::CallButtonState>();
  call_button_state_publisher_ = this->create_publisher<hope_msgs::msg::CallButtonState>("/patient_device/call_button_state");

  //check if patient has pressed the button
  call_button_state_timer_ = this->create_wall_timer(200ms, std::bind(&PatientDeviceController::read_call_button_callback, this));
  //Send the state of the button every 2 sec
  call_publish_timer_ = this->create_wall_timer(2000ms, std::bind(&PatientDeviceController::publish_call_button_state, this));
  //Check call deactivation
  call_state_subscription_ = this->create_subscription<hope_msgs::msg::DeviceState>("/patient_device/call_button_action", std::bind(&PatientDeviceController::set_button_state_flag, this, _1));
    
}

PatientDeviceController::~PatientDeviceController()
{
#ifdef BCM2835
  bcm2835_gpio_write(GPIO_OUTPIN, LOW);
#endif
}

//Read the parameters from the param file
void PatientDeviceController::param_initialize()
{
  RCLCPP_INFO(this->get_logger(), "initialising parameters");
  this->get_parameter_or("patient_device_id", patient_device, std::string("0"));
}

//Check if the patient has pressed the patient call button. If pressed, read the input and publish it over to the ROS2 topic
void PatientDeviceController::read_call_button_callback()
{
#ifdef BCM2835
  if (bcm2835_gpio_lev(INPUT_PIN) == 0)
  {
    gpioon();
    button_pressed = true;
    button_state->deviceid = patient_device;
    button_state->state = 1;
    call_button_state_publisher_->publish(button_state);
  }
#endif
}

void PatientDeviceController::publish_call_button_state()
{
  button_state->deviceid = patient_device;
  if (button_pressed)
  {
    button_state->state = 1;
  }
  else
  {
    button_state->state = 0;
  }
  call_button_state_publisher_->publish(button_state);
}

//A flag is set when we receive response from the webserver
void PatientDeviceController::set_button_state_flag(const hope_msgs::msg::DeviceState::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received device Id : '%s'", msg->deviceid.c_str());
  if (msg->deviceid == patient_device)
  {
    if (msg->state)
    {
      RCLCPP_INFO(this->get_logger(), "Call started");
      button_pressed = true;
#ifdef BCM2835
      gpioon();
#endif
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Call ended");
      button_pressed = false;
#ifdef BCM2835
      gpioresetpin();
#endif
    }
  }
}

// Initial wiring setup needs to done before using BCM2835 functions/APIs
void PatientDeviceController::gpiosetup()
{
  RCLCPP_INFO(this->get_logger(), "setup BCM2835");
#ifdef BCM2835
  if (!bcm2835_init())
  {
    RCLCPP_INFO(this->get_logger(), "setup wiringPi failed");
  }
  bcm2835_gpio_fsel(GPIO_OUTPIN, BCM2835_GPIO_FSEL_OUTP);
  gpioresetpin();
  bcm2835_gpio_fsel(INPUT_PIN, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(INPUT_PIN, BCM2835_GPIO_PUD_UP);
#endif
}

//Sets LED ON
void PatientDeviceController::gpioon()
{
#ifdef BCM2835
  bcm2835_gpio_write(GPIO_OUTPIN, HIGH);
#endif
  RCLCPP_INFO(this->get_logger(), "LED Pin set");
}

//Switches OFF LED
void PatientDeviceController::gpioresetpin()
{
#ifdef BCM2835
  bcm2835_gpio_write(GPIO_OUTPIN, LOW);
#endif
  RCLCPP_INFO(this->get_logger(), "LED Pin reset");
}

void h_sig_sigint(int signum)
{
  // RCLCPP_INFO(this->get_logger(), "Receive signum: '%d'");
  rclcpp::shutdown();
  exit(1);
}

int main(int argc, char *argv[])
{
  signal(SIGINT, h_sig_sigint);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatientDeviceController>();
  node->param_initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
