
#ifndef __VELODYNE_POINTCLOUD_CONVERT_H
#define __VELODYNE_POINTCLOUD_CONVERT_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "velodyne_pointcloud/rawdata.h"
#include "velodyne_pointcloud/pointcloudXYZIR.h"
//#include "velodyne/CloudNodeConfig.h"

//#include <dynamic_reconfigure/server.h>


namespace velodyne_pointcloud
{
	class Convert
	{
		public:
			Convert(ros::NodeHandle &node);
			~Convert() {}
		private:

			//void callback(CloudNodeConfig &config, uint32_t level);
			void processScan(const velodyne::VelodyneScan::ConstPtr &scanMsg);

			//boost::shared_ptr<dynamic_reconfigure::Server<CloudNodeConfig> > srv_;
			boost::shared_ptr<velodyne_rawdata::RawData> data_;

			ros::Subscriber velodyne_scan_;
			ros::Publisher output_;

			/*
			typedef struct
			{
				int npackets;
			} Config;

			Config config_;
			*/
	};
}


#endif
