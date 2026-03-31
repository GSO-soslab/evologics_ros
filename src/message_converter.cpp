#include <chrono>
#include <functional>
#include <memory>


#include <rclcpp/rclcpp.hpp>
#include "message_converter.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;


MessageConverter::MessageConverter(std::string name) : Node(name)
{
  
    this->declare_parameter<std::string>("subbufer_id", "message_converter");

    m_subbuffer_id = this->get_parameter("subbufer_id").as_string();

    this->declare_parameter<std::string>("frame_id", "");

    m_frame_id = this->get_parameter("frame_id").as_string();

    m_to_acomm_pub = this->create_publisher<acomms_msgs::msg::AcommsTxByteArray>("message_converter/to_acomm_data", 10);
    m_to_c2_pub = this->create_publisher<std_msgs::msg::ByteMultiArray>("message_converter/to_c2_data", 10);

    m_from_c2_sub = this->create_subscription<std_msgs::msg::ByteMultiArray>("message_converter/data_from_c2", 10, 
                                                                std::bind(&MessageConverter::c2_to_acomm, 
                                                                this, _1));
    m_from_acomm_sub = this->create_subscription<acomms_msgs::msg::AcommsRxByteArray>("message_converter/data_from_acomm", 10, 
                                                                std::bind(&MessageConverter::acomm_to_c2, 
                                                                this, _1));
                               
}

void MessageConverter::acomm_to_c2(const acomms_msgs::msg::AcommsRxByteArray::SharedPtr msg)
{
    // std_msgs::msg::ByteMultiArray data_out;
    // data_out.data.assign(msg->msg.data.begin(), msg->msg.data.end());
    // m_to_c2_pub->publish(data_out);

}

void MessageConverter::c2_to_acomm(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
{
    // acomms_msgs::msg::AcommsTxByteArray data_out;
    // data_out.msg.data.assign(msg->data.begin(), msg->data.end());
    // data_out.subbfer_id = m_subbuffer_id;
    // data_out.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    // data_out.header.frame_id = m_frame_id;
    // m_to_acomm_pub->publish(data_out);
}