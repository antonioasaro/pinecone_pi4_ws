#include "arduino_comms/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>


void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}


void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

void ArduinoComms::setServoValues(int val_1, int val_2, int val_3, int val_4, int val_5, int val_6, bool print_output)
{
    std::stringstream ss;
    ss << "s " << val_1 << " " << val_2 << " " << val_3 << " " << val_4 << " " << val_5 << " " << val_6 << "\r";
    sendMsg(ss.str(), print_output);
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline();

    if (print_output)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinoComms"),"Sent: " << msg_to_send);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinoComms"),"Received: " << response);
    }

    return response;
}
