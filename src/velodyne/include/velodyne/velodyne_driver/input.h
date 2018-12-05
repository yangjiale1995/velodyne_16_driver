#ifndef __VELODYNE_DRIVER_INPUT_H
#define __VELODYNE_DRIVER_INPUT_H


#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>

#include <ros/ros.h>
#include "velodyne/VelodynePacket.h"


namespace velodyne_driver
{
	static uint16_t DATA_PORT_NUMBER = 2368;
	static uint16_t POSITION_PORT_NUMBER = 8308;

	class Input
	{
		public:
			Input(ros::NodeHandle &node, uint16_t port);
			virtual ~Input() {}


			virtual int getPacket(velodyne::VelodynePacket *ptk, const double time_offset) = 0;
		protected:
			ros::NodeHandle private_nh_;
			uint16_t port_;
			std::string devip_str_;
	};


	class InputSocket : public Input
	{
		public:
			InputSocket(ros::NodeHandle &node, uint16_t port = DATA_PORT_NUMBER);
			virtual ~InputSocket();

			virtual int getPacket(velodyne::VelodynePacket *pkt, const double time_offset);
			void setDeviceIP(const std::string &ip);
		private:
			int sockfd_;
			in_addr devip_;
	};


	class InputPCAP : public Input
	{
		public:
			InputPCAP(ros::NodeHandle &node, uint16_t port = DATA_PORT_NUMBER, double packet_rate = 0.0, std::string fialename="", bool read_once = false, bool read_fast = false, double repeat_delay = 0.0);
			virtual ~InputPCAP();
			virtual int getPacket(velodyne::VelodynePacket *pkt, const double time_offset);
			void setDeviceIP(const std::string &ip);
		private:
			ros::Rate packet_rate_;
			std::string filename_;
			pcap_t *pcap_;
			bpf_program pcap_packet_filter_;
			char errbuf_[PCAP_ERRBUF_SIZE];
			bool empty_;
			bool read_once_;
			bool read_fast_;
			double repeat_delay_;
	};
}

#endif
