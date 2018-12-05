#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include "velodyne_pointcloud/convert.h"


namespace velodyne_pointcloud
{
	Convert::Convert(ros::NodeHandle &node): data_(new velodyne_rawdata::RawData())
	{
		data_->setup(node);

		output_ = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

		//srv_ == boost::make_shared<dynamic_reconfigure::Server<CloudNodeConfig> > (node);
		//dynamic_reconfigure::Server<CloudNodeConfig>::CallbackType f;
		//f = boost::bind(&Convert::callback, this, _1, _2);
		//srv_->setCallback(f);

		velodyne_scan_ = node.subscribe("velodyne_packets", 10, &Convert::processScan, (Convert *)this, ros::TransportHints().tcpNoDelay(true));
	}

	/*
	void Convert::callback(CloudNodeConfig &config, uint32_t level)
	{
		ROS_INFO("Reconfigure Request");
		data_->setParameters(config.min_range, config.max_range, config.view_direction, config.view_width);
	}
	*/

	void Convert::processScan(const velodyne::VelodyneScan::ConstPtr &scanMsg)
	{
		if(output_.getNumSubscribers() == 0)
			return;
		
		PointcloudXYZIR out;
		for(size_t i = 0; i < scanMsg->packets.size(); ++i)
		{
			data_->unpack(scanMsg->packets[i], out);
		}

		sensor_msgs::PointCloud2 laserMsg;
		laserMsg.header.stamp = scanMsg->header.stamp;
		laserMsg.header.frame_id = scanMsg->header.frame_id;
		pcl::toROSMsg(out.pc, laserMsg);
		output_.publish(laserMsg);
	}
}
