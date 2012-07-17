#include <sr_robot_msgs/EthercatDebug.h>
#include "sr_spike_catcher/sr_spike_catcher.hpp"

#include <time.h>

namespace sr_spike_catcher
{
  void SrSpikeCatcher::fill_sensor_list_ ()
  {
    struct_sensor sensor;
    for (uint x = 0; x < SR_SPIKE_CATCHER_DEBUG_CHANNEL_COUNT; x++)
      {
	std::stringstream sensor_ignore_param;
	sensor_ignore_param << "ignore_" << sensor_names[x];
	int ignore;
	node_local_.param<int>(sensor_ignore_param.str(), ignore, 0);

	if (!ignore)
	  {
	    sensor.id = x;
	    sensor.name = sensor_names[x];
	    sensors_.push_back(sensor);
	  }
      }
  }

  SrSpikeCatcher::SrSpikeCatcher()
    : node_local_("~"), last_time_(-1.0), msg_count_(0)
  {
    std::string log_file_dir_;
    node_local_.param<std::string>("log_file_dir", log_file_dir_, "./log");

    std::stringstream ss;
    ss << log_file_dir_ << "/" << time (NULL) << ".txt";
    output_file_name_ = ss.str();

    ROS_INFO("Spike catcher will log any errors to %s", output_file_name_.c_str());

    fill_sensor_list_();
    std::string test = "debug_etherCAT_data";
    debug_subscriber_ = node_.subscribe(test, 1000, &SrSpikeCatcher::debug_callback_, this);
  }

  void SrSpikeCatcher::debug_callback_ (const sr_robot_msgs::EthercatDebug::ConstPtr& msg)
  {
    std::stringstream output;
    check_sensors_(output, msg);

    double this_time = msg->header.stamp.toSec();

    if ( ( (this_time - last_time_) > 2.0 ) && last_time_ > 0.0 )
      {
	debug_subscriber_.shutdown();
	ROS_INFO("Problem with driver - log stopped");

	return;
      }
    last_time_ = this_time;

    if (output.str().length())
      {
	append_sensor_data_(output);
	output_stream_ << output.str();
	ROS_INFO ("%s", output.str().c_str());
      }
    msg_count_ ++;
  }

  SrSpikeCatcher::~SrSpikeCatcher()
  {
    output_stream_.flush();
    output_stream_.close();
    debug_subscriber_.shutdown();
  }
  
  void SrSpikeCatcher::open_output_ ()
  { 
    output_stream_.open(output_file_name_.c_str());

    for (uint x = 0; x < sensors_.size() ; x++)
      {
	output_stream_ << sensors_[x].id << ":" << sensors_[x].name << "\n";
      }
  }

  void SrSpikeCatcher::check_sensors_(std::stringstream & output, const sr_robot_msgs::EthercatDebug::ConstPtr& msg)
  {
    for (uint x = 0; x < sensors_.size() ; x++)
      {
	struct_sensor sensor = sensors_[x];
	
	if (msg_count_ > 1000 && last_[sensor.id] != 0 && msg->sensors[sensor.id] != 0)
	  {
	    int dif = msg->sensors[sensor.id] - last_[sensor.id];
	    if ( (dif > 30) || (dif < -30) )
	      {
		if (!output_stream_.is_open())
		  open_output_ ();
		
		if (output.str().length() == 0)
		  output << "\n-----------------" << msg->header.seq;
		
		output << "\nsensor - " << sensor.id << " - " << sensor.name;
		output << "\nvalue  - " << msg->sensors[sensor.id];
		output << "\nlast   - " << last_[sensor.id];
		output << "\ndiff   - " << dif;
	      }
	  }
	ancient_[sensor.id] = previous_[sensor.id];
	previous_[sensor.id] = last_[sensor.id];
	last_[sensor.id] = msg->sensors[sensor.id];
      }
  }
  void SrSpikeCatcher::append_sensor_data_(std::stringstream & output)
  {
    std::stringstream s_ancient;
    std::stringstream s_previous;
    std::stringstream s_last;

    for (uint x = 0; x < sensors_.size() ; x++)
      {
	uint id = sensors_[x].id;
	s_ancient  << ancient_[id] << ",";
	s_previous << previous_[id] << ",";
	s_last     << last_[id] << ",";
      }

    output << "\n" << s_ancient.str() 
	   << "\n" << s_previous.str() 
	   << "\n" << s_last.str() 
	   << "\n";
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sr_spike_catcher");
  
  sr_spike_catcher::SrSpikeCatcher catcher;
  
  ros::spin();

  return 0;
}
