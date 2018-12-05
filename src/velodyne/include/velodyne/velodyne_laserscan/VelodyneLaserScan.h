#ifndef VELODYNE_LASERSCAN_VELODYNELASERSCAN_H
#define VELODYNE_LASERSCAN_VELODYNELASERSCAN_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

//#include <dynamic_reconfigure/server.h>
//#include "velodyne/VelodyneLaserScanConfig.h"


namespace velodyne_laserscan
{
	class VelodyneLaserScan
	{
		public:
			VelodyneLaserScan(ros::NodeHandle &nh);
		private:
			boost::mutex connect_mutex_;
			void connectCb();
			void recvCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

			ros::NodeHandle nh_;
			ros::Subscriber sub_;
			ros::Publisher pub_;

			typedef struct
			{
				int ring;
				double resolution;
			} Config;
			Config cfg_;
			//VelodyneLaserScanConfig cfg_;
			//dynamic_reconfigure::Server<VelodyneLaserScanConfig> srv_;
			//void reconfig(VelodyneLaserScanConfig& config, uint32_t level);

			unsigned int  ring_count_;
	};
}


#endif
