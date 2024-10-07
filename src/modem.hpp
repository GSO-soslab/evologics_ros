/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Author: Jason Miller, jason_miller@uri.edu
    Author: Lin Zhao, linzhao@uri.edu
    Year: 2023-2024

    Copyright (C) 2023-2024 Smart Ocean Systems Laboratory
*/

#pragma once

// c++
#include <thread>

// ros2
#include <rclcpp/rclcpp.hpp>

//ros messages
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <acomms_msgs/msg/acomms_rx.hpp>
#include <acomms_msgs/msg/acomms_tx.hpp>
#include <acomms_msgs/msg/acomms_rx_byte_array.hpp>
#include <acomms_msgs/msg/acomms_tx_byte_array.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

//ros services
#include <robot_localization/srv/to_ll.hpp>

//goby includes
#include <goby/acomms/connect.h>
#include <goby/acomms/amac.h>
#include <goby/acomms/buffer/dynamic_buffer.h>
#include <goby/acomms/queue.h>
#include <goby/acomms/bind.h>
#include <goby/acomms/modem_driver.h>
#include <goby/util/binary.h>
#include <goby/util/debug_logger.h>
#include <goby/util/debug_logger/flex_ostream.h>         // for FlexOs...
#include <goby/util/debug_logger/flex_ostreambuf.h>      // for DEBUG1

#include "evologics_driver.h"

class Modem : public rclcpp::Node
{

public:
    Modem(std::string name = "modem");

    ~Modem();

private:

    // ===================================================================== //
    // types
    // ===================================================================== //

    struct Interface
    {
        std::string if_type;
        std::string tcp_address;
        int tcp_port;
        std::string device;
        int baudrate;
    };

    struct DynamicBuffer
    {
        std::vector<std::string> messages;
    };

    struct Config
    {
        std::string type;
        std::string driver;
        int max_frame_bytes;
        int mac_slot_time;

        DynamicBuffer dynamic_buffer;

        Interface interface;
        int source_level;
        int source_control;
        int gain_level;
        int carrier_waveform_id;
        int local_address;
        int remote_address;
        int highest_address;
        int cluster_size;
        int packet_time;
        int retry_count;
        int retry_timeout;
        int keep_online_count;
        int idle_timeout;
        int channel_protocol_id;
        int sound_speed;
    };

    struct MessageConfig
    {
        bool ack;
        int blackout_time;
        int max_queue;
        bool newest_first;
        int ttl;
        int value_base;
    };

    // ===================================================================== //
    // global variables
    // ===================================================================== //

    std::thread loop_worker_;

    Config config_;

    std::map<std::string, MessageConfig> dynamic_buffer_config_;

    goby::acomms::EvologicsDriver evo_driver_;

    goby::acomms::MACManager mac_;

    goby::acomms::DynamicBuffer<std::string> buffer_;

    // ===================================================================== //
    // functions
    // ===================================================================== //

    void loop();

    /**
     * @brief the goby dccl, mac, queue, and driver are configured and initialized
     *
     */
    void loadGoby();

    void loadBuffer();

    void parseGobyParams();

    void parseEvologicsParams();

    void evologicsPositioningData(UsbllongMsg msg);

    void addToBuffer(const acomms_msgs::msg::AcommsTx::SharedPtr msg);

    void addBytesToBuffer(const acomms_msgs::msg::AcommsTxByteArray::SharedPtr msg);

    /**
     * @brief slot that the driver calls when it wants to send data
     *
     * @param msg pointer to the outgoing message the driver is requesting
     */
    void dataRequest(goby::acomms::protobuf::ModemTransmission *msg);

    /**
     * @brief the slot that is called back from the driver when a new message is received.
     *
     * @param data_msg the incoming message
     */
    void receivedData(const goby::acomms::protobuf::ModemTransmission &data_msg);

    // ===================================================================== //
    // ROS2 related
    // ===================================================================== //
    rclcpp::Publisher<geographic_msgs::msg::GeoPoint>::SharedPtr 
        track_pub_;    

    rclcpp::Client<robot_localization::srv::ToLL>::SharedPtr 
        toll_client_;

    rclcpp::Subscription<acomms_msgs::msg::AcommsTx>::SharedPtr 
        modem_tx_sub_;    

    rclcpp::Subscription<acomms_msgs::msg::AcommsTxByteArray>::SharedPtr 
        modem_tx_bytearray_sub_;    

    rclcpp::Publisher<acomms_msgs::msg::AcommsRx>::SharedPtr 
        modem_rx_pub_;

    rclcpp::Publisher<acomms_msgs::msg::AcommsRxByteArray>::SharedPtr 
        modem_rx_bytearray_pub_;
};