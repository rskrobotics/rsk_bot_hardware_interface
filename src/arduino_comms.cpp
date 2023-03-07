#include "rsk_bot_hardware_interface/arduino_comms.hpp"
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

// void ArduinoComms::readRPM(double &val_1, double &val_2)
// {
//     std::string response = sendMsg("e\r");

//     std::string delimiter = " ";
//     size_t del_pos = response.find(delimiter);
//     if (del_pos == std::string::npos) {
//         // RCLCPP_INFO(rclcpp::get_logger("RSKBotSystemHardware"), "String does not contain space. I got: %s :", response.c_str());
//         return;
//     }

//     std::string token_1 = response.substr(0, del_pos);
//     std::string token_2 = response.substr(del_pos + delimiter.length());

//     try {
//         val_1 = std::stod(token_1);
//         val_2 = std::stod(token_2);
//     } catch (const std::exception& e) {
//         RCLCPP_INFO(rclcpp::get_logger("RSKBotSystemHardware"), "Exception, what i got is = %s", response.c_str());
//     }
// }

void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    // auto start = std::chrono::high_resolution_clock::now();
    std::string response = sendMsg("e\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    if (del_pos == std::string::npos) {
        RCLCPP_INFO(rclcpp::get_logger("RSKBotSystemHardware"), "String does not contain space. I got: %s :", response.c_str());
        return;
    }

    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    try {
        val_1 = std::stod(token_1);
        val_2 = std::stod(token_2);
            // RCLCPP_INFO(rclcpp::get_logger("RSKBotSystemHardware"), "Setting values: %f", val_1);

    } catch (const std::exception& e) {
        RCLCPP_INFO(rclcpp::get_logger("RSKBotSystemHardware"), "Exception, what i got is = %s", response.c_str());
    }
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    // std::cout << duration.count() << std::endl;

    // RCLCPP_INFO(rclcpp::get_logger("RSKBotSystemHardware"),"Duration: %ld", duration.count());

}


void ArduinoComms::setMotorValues(double val_1, double val_2)
{
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    sendMsg(ss.str(), false);

    // RCLCPP_INFO(rclcpp::get_logger("RSKBotSystemHardware"), "Setting values: %f", val_1);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline();

    // if (print_output)
    // {
    //    RCLCPP_INFO(rclcpp::get_logger("RSKBotSystemHardware"), "Sent: %s", msg_to_send.c_str());
    //    RCLCPP_INFO(rclcpp::get_logger("RSKBotSystemHardware"), "Response: %s", response.c_str());

        // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    // }

    return response;
}
