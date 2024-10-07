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

#include "modem.hpp"

using goby::glog;
using goby::util::as;

using namespace std::chrono_literals;

Modem::Modem(std::string name) : Node(name)
{
    RCLCPP_INFO(get_logger(), "Modem constructor");

    // ===================================================================== //
    // load param
    // ===================================================================== //

    parseGobyParams();

    if (config_.driver == "evologics") { 
        parseEvologicsParams();
    }
    else if (config_.driver == "seatrac") { 
        parseSeatracParams();
    }    

    if (config_.type == "usbl")
    {

        track_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoint>(
            "usbl_track", 10 );        

        toll_client_ = this->create_client<robot_localization::srv::ToLL>("toLL");

        while (!toll_client_->wait_for_service(2s)) {
            RCLCPP_WARN(get_logger(), 
                "service(%s) not available, waiting again...", "toLL");
        }


        if (config_.driver == "evologics")
        {
            evo_driver_.set_usbl_callback(
                std::bind(&Modem::evologicsPositioningData, this, std::placeholders::_1));
        }
        else if (config_.driver == "seatrac")
        {
            seatrac_driver_.set_usbl_callback(
                std::bind(&Modem::seatracPositioningData, this, std::placeholders::_1));
        }
    }

    loadGoby();

    modem_tx_sub_ = this->create_subscription<acomms_msgs::msg::MvpAcommsTx>(
        "/tx", 10, std::bind(&Modem::addToBuffer, this, std::placeholders::_1));

    modem_tx_bytearray_sub_ = this->create_subscription<acomms_msgs::msg::MvpAcommsTxByteArray>(
        "/tx_bytearray", 10, std::bind(&Modem::addBytesToBuffer, this, std::placeholders::_1));

    modem_rx_pub_ = this->create_publisher<acomms_msgs::msg::MvpAcommsRx>(
        config_.type + "/rx", 10);

    modem_rx_bytearray_pub_ = this->create_publisher<acomms_msgs::msg::MvpAcommsRxByteArray>(
        config_.type + "/rx_bytearray", 10); 
    
    loop_worker_ = std::thread([this] { loop(); });
    loop_worker_.detach();  

}

Modem::~Modem()
{
    rclcpp::shutdown();
}

void Modem::loop()
{
    // loop at 10Hz
    rclcpp::Rate rate(10); 

    while (rclcpp::ok())
    {
        evo_driver_.do_work();
        mac_.do_work();
        buffer_.expire();

        rate.sleep();
    }
    
}

void Modem::parseGobyParams() {

    this->declare_parameter("goby.driver", "evologics");
    this->get_parameter("goby.driver", config_.driver);

    this->declare_parameter("goby.max_frame_bytes", 100);
    this->get_parameter("goby.max_frame_bytes", config_.max_frame_bytes);

    this->declare_parameter("goby.mac_slot_time", 10);
    this->get_parameter("goby.mac_slot_time", config_.mac_slot_time);

    this->declare_parameter("goby.dynamic_buffer.messages", 
        std::vector<std::string>({"aa", "bb", "cc", "dd"}));
    this->get_parameter("goby.dynamic_buffer.messages", 
        config_.dynamic_buffer.messages);

    for (std::string message : config_.dynamic_buffer.messages)
    {
        this->declare_parameter("goby.dynamic_buffer."+message+".ack", false);
        this->get_parameter("goby.dynamic_buffer."+message+".ack", 
            dynamic_buffer_config_[message].ack);

        this->declare_parameter("goby.dynamic_buffer."+message+".blackout_time", 0);
        this->get_parameter("goby.dynamic_buffer."+message+".blackout_time", 
            dynamic_buffer_config_[message].blackout_time);

        this->declare_parameter("goby.dynamic_buffer."+message+".max_queue", 0);
        this->get_parameter("goby.dynamic_buffer."+message+".max_queue", 
            dynamic_buffer_config_[message].max_queue);

        this->declare_parameter("goby.dynamic_buffer."+message+".newest_first", true);
        this->get_parameter("goby.dynamic_buffer."+message+".newest_first", 
            dynamic_buffer_config_[message].newest_first);      

        this->declare_parameter("goby.dynamic_buffer."+message+".ttl", 1800);
        this->get_parameter("goby.dynamic_buffer."+message+".ttl", 
            dynamic_buffer_config_[message].ttl);            

        this->declare_parameter("goby.dynamic_buffer."+message+".value_base", 1);
        this->get_parameter("goby.dynamic_buffer."+message+".value_base", 
            dynamic_buffer_config_[message].value_base);                             
    }   
}

void Modem::parseEvologicsParams()
{
    this->declare_parameter(
        "type", "modem");
    this->get_parameter(
        "type", config_.type);

    this->declare_parameter(
        config_.type+"_configuration.interface.connection_type", "tcp");
    this->get_parameter(
        config_.type+"_configuration.interface.connection_type", 
        config_.interface.if_type);

    this->declare_parameter(
        config_.type+"_configuration.interface.tcp_address", "192.168.2.109");
    this->get_parameter(
        config_.type+"_configuration.interface.tcp_address", 
        config_.interface.tcp_address);

    this->declare_parameter(
        config_.type+"_configuration.interface.tcp_port", 9200);
    this->get_parameter(
        config_.type+"_configuration.interface.tcp_port", 
        config_.interface.tcp_port);

    this->declare_parameter(
        config_.type+"_configuration.interface.device", "/dev/ttyUSB0");
    this->get_parameter(
        config_.type+"_configuration.interface.device", 
        config_.interface.device);

    this->declare_parameter(
        config_.type+"_configuration.interface.baudrate", 115200);
    this->get_parameter(
        config_.type+"_configuration.interface.baudrate", 
        config_.interface.baudrate);

    this->declare_parameter(
        config_.type+"_configuration.source_level", 0);
    this->get_parameter(
        config_.type+"_configuration.source_level", 
        config_.source_level);

    this->declare_parameter(
        config_.type+"_configuration.source_control", 1);
    this->get_parameter(
        config_.type+"_configuration.source_control", 
        config_.source_control);

    this->declare_parameter(
        config_.type+"_configuration.gain_level", 0);
    this->get_parameter(
        config_.type+"_configuration.gain_level", 
        config_.gain_level);

    this->declare_parameter(
        config_.type+"_configuration.carrier_waveform_id", 0);
    this->get_parameter(
        config_.type+"_configuration.carrier_waveform_id", 
        config_.carrier_waveform_id);

    this->declare_parameter(
        config_.type+"_configuration.local_address", 2);
    this->get_parameter(
        config_.type+"_configuration.local_address", 
        config_.local_address);

    this->declare_parameter(
        config_.type+"_configuration.remote_address", 1);
    this->get_parameter(
        config_.type+"_configuration.remote_address", 
        config_.remote_address);

    this->declare_parameter(
        config_.type+"_configuration.highest_address", 2);
    this->get_parameter(
        config_.type+"_configuration.highest_address", 
        config_.highest_address);

    this->declare_parameter(
        config_.type+"_configuration.cluster_size", 10);
    this->get_parameter(
        config_.type+"_configuration.cluster_size", 
        config_.cluster_size);

    this->declare_parameter(
        config_.type+"_configuration.packet_time", 750);
    this->get_parameter(
        config_.type+"_configuration.packet_time", 
        config_.packet_time);

    this->declare_parameter(
        config_.type+"_configuration.retry_count", 3);
    this->get_parameter(
        config_.type+"_configuration.retry_count", 
        config_.retry_count);

    this->declare_parameter(
        config_.type+"_configuration.retry_timeout", 4000);
    this->get_parameter(
        config_.type+"_configuration.retry_timeout", 
        config_.retry_timeout);

    this->declare_parameter(
        config_.type+"_configuration.keep_online_count", 0);
    this->get_parameter(
        config_.type+"_configuration.keep_online_count", 
        config_.keep_online_count);

    this->declare_parameter(
        config_.type+"_configuration.idle_timeout", 120);
    this->get_parameter(
        config_.type+"_configuration.idle_timeout", 
        config_.idle_timeout);

    this->declare_parameter(
        config_.type+"_configuration.channel_protocol_id", 0);
    this->get_parameter(
        config_.type+"_configuration.channel_protocol_id", 
        config_.channel_protocol_id);

    this->declare_parameter(
        config_.type+"_configuration.sound_speed", 1500);
    this->get_parameter(
        config_.type+"_configuration.sound_speed", 
        config_.sound_speed);
}

void Modem::parseSeatracParams()
{
    this->declare_parameter(
        "type", "modem");
    this->get_parameter(
        "type", config_.type);

    config_.interface.if_type = "tcp";

    this->declare_parameter(
        config_.type+"_configuration.interface.device", "/dev/ttyUSB0");
    this->get_parameter(
        config_.type+"_configuration.interface.device", 
        config_.interface.device);

    this->declare_parameter(
        config_.type+"_configuration.interface.baudrate", 115200);
    this->get_parameter(
        config_.type+"_configuration.interface.baudrate", 
        config_.interface.baudrate);


    this->declare_parameter(
        config_.type+"_configuration.local_address", 1);
    this->get_parameter(
        config_.type+"_configuration.local_address", 
        config_.local_address);

    this->declare_parameter(
        config_.type+"_configuration.remote_address", 2);
    this->get_parameter(
        config_.type+"_configuration.remote_address", 
        config_.remote_address);

    this->declare_parameter(
        config_.type+"_configuration.sound_speed", 1500);
    this->get_parameter(
        config_.type+"_configuration.sound_speed", 
        config_.sound_speed);
}

void Modem::evologicsPositioningData(UsbllongMsg msg)
{
    // call the toll srv
    auto request = std::make_shared<robot_localization::srv::ToLL::Request>();

    request->map_point.x = msg.pose.enu.e;
    request->map_point.y = msg.pose.enu.n;
    request->map_point.z = msg.pose.enu.u;

    RCLCPP_INFO(get_logger(), "E: %f\tN: %f\tU: %f", msg.pose.enu.e, msg.pose.enu.n, msg.pose.enu.u);

    auto result = toll_client_->async_send_request(request);

    // get the srv result and Publish USBL Tracking

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) 
            == rclcpp::FutureReturnCode::SUCCESS)
    {
        geographic_msgs::msg::GeoPoint track;
        geographic_msgs::msg::GeoPoseStamped geo_pose;

        track.latitude = result.get()->ll_point.latitude;
        track.longitude = result.get()->ll_point.longitude;
        track.altitude = result.get()->ll_point.altitude;
        track_pub_->publish(track);

        RCLCPP_INFO(get_logger(), "Lat: %f\t Lon: %f", track.latitude, track.longitude);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call ToLL service");
    }
}

void Modem::seatracPositioningData(ACOFIX_T msg)
{

}

void Modem::loadGoby()
{
    goby::acomms::bind(mac_, evo_driver_);

    // connect the receive signal from the driver to the modem slot
    goby::acomms::connect(&evo_driver_.signal_receive, this, &Modem::receivedData);

    // connect the outgoing data request signal from the driver to the modem slot
    goby::acomms::connect(&evo_driver_.signal_data_request, this, &Modem::dataRequest);

    // Initiate modem driver
    goby::acomms::protobuf::DriverConfig driver_cfg;

    driver_cfg.set_modem_id(config_.local_address);

    if (config_.interface.if_type == "tcp")
    {
        driver_cfg.set_connection_type(
            goby::acomms::protobuf::DriverConfig_ConnectionType_CONNECTION_TCP_AS_CLIENT);
        driver_cfg.set_tcp_server(config_.interface.tcp_address);
        driver_cfg.set_tcp_port(config_.interface.tcp_port);
    }
    else if (config_.interface.if_type == "serial")
    {
        driver_cfg.set_connection_type(
            goby::acomms::protobuf::DriverConfig_ConnectionType_CONNECTION_SERIAL);
        driver_cfg.set_serial_port(config_.interface.device);
        driver_cfg.set_serial_baud(config_.interface.baudrate);
    }

    // Initiate medium access control
    goby::acomms::protobuf::MACConfig mac_cfg;
    mac_cfg.set_type(goby::acomms::protobuf::MAC_FIXED_DECENTRALIZED);
    mac_cfg.set_modem_id(config_.local_address);

    // setup our modem slot
    goby::acomms::protobuf::ModemTransmission my_slot;
    my_slot.set_src(config_.local_address);
    my_slot.set_dest(config_.remote_address);
    my_slot.set_max_frame_bytes(config_.max_frame_bytes);
    my_slot.set_max_num_frames(1);
    my_slot.set_type(goby::acomms::protobuf::ModemTransmission::DATA);
    my_slot.set_slot_seconds(config_.mac_slot_time);

    // setup the modem slot
    goby::acomms::protobuf::ModemTransmission buddy_slot;
    buddy_slot.set_src(config_.remote_address);
    buddy_slot.set_dest(config_.local_address);
    buddy_slot.set_max_frame_bytes(config_.max_frame_bytes);
    buddy_slot.set_max_num_frames(1);
    buddy_slot.set_type(goby::acomms::protobuf::ModemTransmission::DATA);
    buddy_slot.set_slot_seconds(config_.mac_slot_time);

    if (config_.local_address < config_.remote_address)
    {
        mac_cfg.add_slot()->CopyFrom(my_slot);
        mac_cfg.add_slot()->CopyFrom(buddy_slot);
    }
    else
    {
        mac_cfg.add_slot()->CopyFrom(buddy_slot);
        mac_cfg.add_slot()->CopyFrom(my_slot);
    }

    goby::glog.set_name(config_.type);
    goby::glog.add_stream(goby::util::logger::DEBUG1, &std::clog);

    // startup the mac and evo_driver_
    mac_.startup(mac_cfg);
    evo_driver_.startup(driver_cfg);

    loadBuffer();
}

void Modem::loadBuffer()
{
    // create a buffer cfg
    goby::acomms::protobuf::DynamicBufferConfig cfg;

    std::map<std::string, MessageConfig>::iterator it = dynamic_buffer_config_.begin();

    while (it != dynamic_buffer_config_.end())
    {
        cfg.set_ack_required(it->second.ack);
        cfg.set_blackout_time(it->second.blackout_time);
        cfg.set_max_queue(it->second.max_queue);
        cfg.set_newest_first(it->second.newest_first);
        cfg.set_ttl(it->second.ttl);
        cfg.set_value_base(it->second.value_base);

        buffer_.create(config_.remote_address, it->first, cfg);

        cfg.Clear();

        ++it;
    }
}

void Modem::receivedData(const goby::acomms::protobuf::ModemTransmission &data_msg)
{
    acomms_msgs::msg::MvpAcommsRx msg;

    msg.data = data_msg.frame()[0];

    modem_rx_pub_->publish(msg);

    acomms_msgs::msg::MvpAcommsRxByteArray byte_msg;
    std::vector<uint8_t> data(data_msg.frame()[0].begin(), data_msg.frame()[0].end());
    byte_msg.msg.data = data;

    modem_rx_bytearray_pub_->publish(byte_msg);
}

void Modem::dataRequest(goby::acomms::protobuf::ModemTransmission *msg)
{
    int dest = msg->dest();

    std::string *frame = msg->add_frame();

    while (frame->size() < msg->max_frame_bytes())
    {
        try
        {
            auto buffer_value =
                buffer_.top(dest, msg->max_frame_bytes() - frame->size());

            // *frame += buffer_value.data.data();
            frame->append(buffer_value.data + '\n');

            buffer_.erase(buffer_value);
        }
        catch (goby::acomms::DynamicBufferNoDataException &)
        {
            break;
        }
    }
    
}

void Modem::addToBuffer(const acomms_msgs::msg::MvpAcommsTx::SharedPtr msg)
{
    if (dynamic_buffer_config_.find(msg->subbuffer_id) != 
        dynamic_buffer_config_.end())
    {
        buffer_.push({config_.remote_address, msg->subbuffer_id, 
                      goby::time::SteadyClock::now(), msg->data});

        RCLCPP_INFO(get_logger(), "Data Added to Buffer: %s", 
            goby::util::hex_encode(msg->data).c_str()); //cnr
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Subbuffer ID: %s has not been added to the configuratiron file goby.yaml", 
            msg->subbuffer_id.data()); //cnr
    }
}

void Modem::addBytesToBuffer(const acomms_msgs::msg::MvpAcommsTxByteArray::SharedPtr msg)
{
    if (dynamic_buffer_config_.find(msg->subbuffer_id) 
        != dynamic_buffer_config_.end())
    {
        auto out = msg->msg.data;

        std::string str(msg->msg.data.begin(), msg->msg.data.end());

        buffer_.push({config_.remote_address, msg->subbuffer_id, goby::time::SteadyClock::now(),str });

        RCLCPP_INFO(get_logger(), "Data Added to Buffer: %s", 
            goby::util::hex_encode(str).c_str()); 
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Subbuffer ID: %s has not been added to the configuratiron file goby.yaml", 
            msg->subbuffer_id.data()); 
    }
}