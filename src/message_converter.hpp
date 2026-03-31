#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <acomms_msgs/msg/acomms_rx.hpp>
#include <acomms_msgs/msg/acomms_tx.hpp>
#include <acomms_msgs/msg/acomms_rx_byte_array.hpp>
#include <acomms_msgs/msg/acomms_tx_byte_array.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

using namespace std::chrono_literals;




class MessageConverter : public rclcpp::Node
{
public:
    MessageConverter(std::string name = "message_converter_node");
  

private:


    rclcpp::Publisher<acomms_msgs::msg::AcommsTxByteArray>::SharedPtr m_to_acomm_pub;
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr m_to_c2_pub;

    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr m_from_c2_sub;
    rclcpp::Subscription<acomms_msgs::msg::AcommsRxByteArray>::SharedPtr m_from_acomm_sub;

    void acomm_to_c2(const acomms_msgs::msg::AcommsRxByteArray::SharedPtr msg);
    void c2_to_acomm(const std_msgs::msg::ByteMultiArray::SharedPtr msg);
    std::string m_subbuffer_id;
    std::string m_frame_id;


};