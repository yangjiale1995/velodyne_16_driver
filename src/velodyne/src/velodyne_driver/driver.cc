#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "velodyne/VelodyneScan.h"
#include "velodyne_driver/driver.h"

namespace velodyne_driver
{
	VelodyneDriver::VelodyneDriver(ros::NodeHandle &node)
	{
		node.param("frame_id", config_.frame_id, std::string("velodyne"));
		std::string tf_prefix = tf::getPrefixParam(node);
		config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

		node.param("model", config_.model, std::string("VLP16"));
		double packet_rate;
		std::string model_full_name;
		if(config_.model == "64E")
		{
			packet_rate = 2600.0;
			model_full_name = std::string("HDL-") + config_.model;
		}
		else if(config_.model == "32E")
		{
			packet_rate = 1808.0;
			model_full_name = std::string("HDL-") + config_.model;
		}
		else if(config_.model == "32C")
		{
			packet_rate = 1507.0;
			model_full_name = std::string("VLP-") + config_.model;
		}
		else if(config_.model == "VLP16")
		{
			packet_rate = 754;
			model_full_name = "VLP-16";
		}
		else
		{
			ROS_ERROR_STREAM("unknown Velodyne LIDAR model : " << config_.model);
			packet_rate = 2600.0;
		}
		std::string deviceName(std::string("Velodyne ") + model_full_name);

		node.param("rpm", config_.rpm, 600.0);
		ROS_INFO_STREAM(deviceName << "rotating at " << config_.rpm << " RPM");
		double frequency = (config_.rpm / 60.0);

		config_.npackets = (int) ceil(packet_rate / frequency);

		node.getParam("npackets", config_.npackets);
		ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

		std::string dump_file;
		node.param("pcap", dump_file, std::string(""));

		double cut_angle;
		node.param("cut_angle", cut_angle, -0.01);
		if(cut_angle < 0.0)
		{
			ROS_INFO_STREAM("Cut at specific angle feature deactivated.");
		}
		else if(cut_angle < (2 * M_PI))
		{
			ROS_INFO_STREAM("Cut at specific angle feature activated."
					"Cutting velodyne points always at " << cut_angle << " rad.");
		}
		else
		{
			ROS_ERROR_STREAM("cut_angle parameter is out of range. Allowed range is " << "between 0.0 and 2 * M_PI or negative values to deactivate this feature.");
			cut_angle = -0.01;
		}

		config_.cut_angle = int((cut_angle * 360.0 / (2 * M_PI)) * 100);
		
		int udp_port;
		node.param("port", udp_port, (int)DATA_PORT_NUMBER);

		//srv_ = boost::make_shared<dynamic_reconfigure::Server<VelodyneNodeConfig> >(node);
		//dynamic_reconfigure::Server<VelodyneNodeConfig>::CallbackType f;
		//f = boost::bind(&VelodyneDriver::callback, this, _1, _2);
		//srv_->setCallback(f);
		
		diagnostics_.setHardwareID(deviceName);
		const double diag_freq = packet_rate / config_.npackets;

		diag_max_freq_ = diag_freq;
		diag_min_freq_ = diag_freq;
		ROS_INFO("expected frequency : %.3f (Hz)", diag_freq);

		using namespace diagnostic_updater;
		diag_topic_.reset(new TopicDiagnostic("Velodyne_packets", diagnostics_, FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10), TimeStampStatusParam()));

		diag_timer_ = node.createTimer(ros::Duration(0.2), &VelodyneDriver::diagTimerCallback, this);


		if(dump_file != "")
		{
			input_.reset(new velodyne_driver::InputPCAP(node, udp_port, packet_rate, dump_file));
		}
		else
		{
			input_.reset(new velodyne_driver::InputSocket(node, udp_port));
		}


		output_ = node.advertise<velodyne::VelodyneScan>("velodyne_packets", 10);

		std::cout << "..............config loaded ................" << std::endl;
	}


	bool VelodyneDriver::poll(void)
	{
		velodyne::VelodyneScanPtr scan(new velodyne::VelodyneScan);

		if(config_.cut_angle >= 0)
		{
			scan->packets.reserve(config_.npackets);
			velodyne::VelodynePacket tmp_packet;
			while(true)
			{
				while(true)
				{
					int rc = input_->getPacket(&tmp_packet, config_.time_offset);
					if(rc == 0)	break;
					if(rc < 0)	return false;
				}
				scan->packets.push_back(tmp_packet);


				static int last_azimuth = -1;
				std::size_t azimuth_data_pos = 100*0+2;
				int azimuth = *((u_int16_t*)(&tmp_packet.data[azimuth_data_pos]));

				if(last_azimuth == -1)
				{
					last_azimuth = azimuth;
					continue;
				}
				if((last_azimuth < config_.cut_angle && config_.cut_angle <= azimuth) || (config_.cut_angle <= azimuth && azimuth < last_azimuth) || (azimuth < last_azimuth && last_azimuth < config_.cut_angle))
				{
					last_azimuth = azimuth;
					break;
				}
				last_azimuth = azimuth;
			}
		}
		else
		{
			scan->packets.resize(config_.npackets);
			for(int i = 0; i < config_.npackets; ++i)
			{
				while(true)
				{
					int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
					if(rc == 0)	break;
					if(rc < 0)	return false;
				}
			}
		}


		ROS_DEBUG("Publishing a full Velodyne scan.");
		scan->header.stamp = scan->packets.back().stamp;
		scan->header.frame_id = config_.frame_id;
		output_.publish(scan);


		diag_topic_->tick(scan->header.stamp);
		diagnostics_.update();

		return true;
	}


	/*
	void VelodyneDriver::callback(velodyne_driver::VelodyneNodeConfig &config, uint32_t level)
	{
		ROS_INFO("Reconfigure Request");
		config_.time_offset = config.time_offset;
	}
	*/

	void VelodyneDriver::diagTimerCallback(const ros::TimerEvent &event)
	{
		(void)event;
		diagnostics_.update();
	}
}
