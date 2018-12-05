#ifndef __VELODYNE_DRIVER_DRIVER_H
#define __VELODYNE_DRIVER_DRIVER_H


#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
//#include <dynamic_reconfigure/server.h>

#include "velodyne_driver/input.h"
//#include "velodyne/VelodyneNodeConfig.h"


namespace velodyne_driver
{
	class VelodyneDriver
	{
		public:
			VelodyneDriver(ros::NodeHandle &node);
			~VelodyneDriver() {}

			bool poll(void);


		private:
			//void callback(VelodyneNodeConfig &config, uint32_t level);
			void diagTimerCallback(const ros::TimerEvent &event);
			//boost::shared_ptr<dynamic_reconfigure::Server<VelodyneNodeConfig> > srv_;


			struct
			{
				std::string frame_id;
				std::string model;
				int npackets;
				double rpm;
				int cut_angle;
				double time_offset;
			} config_;


			boost::shared_ptr<Input> input_;
			ros::Publisher output_;


			ros::Timer diag_timer_;
			diagnostic_updater::Updater diagnostics_;
			double diag_min_freq_;
			double diag_max_freq_;
			boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
	};
}

#endif
