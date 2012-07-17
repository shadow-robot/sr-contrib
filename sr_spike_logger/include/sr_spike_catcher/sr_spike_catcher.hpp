#ifndef _SR_SPIKE_CATCHER_HPP_
#define _SR_SPIKE_CATCHER_HPP_

#define SR_SPIKE_CATCHER_DEBUG_CHANNEL_COUNT 32


#include <ros/node_handle.h>
#include <fstream>
#include <sstream>
#include <vector>

namespace sr_spike_catcher
{
  struct struct_sensor
  {
    uint id;
    std::string name;
  };

  const std::string sensor_names[]
  {
    "FFJ1",  "FFJ2",  "FFJ3", "FFJ4", //0-3
    "MFJ1",  "MFJ2",  "MFJ3", "MFJ4", //4-7
    "RFJ1",  "RFJ2",  "RFJ3", "RFJ4", //8-11
    "LFJ1",  "LFJ2",  "LFJ3", "LFJ4", "LFJ5", //12-16
    "THJ1",  "THJ2",  "THJ3", "THJ4", "THJ5A", "THJ5B",//17-22
    "WRJ1A", "WRJ1B", "WRJ2",//23-25
    "ACCX",  "ACCY",  "ACCZ",
    "GYRX",  "GYRY",  "GYRZ"
      };

  class SrSpikeCatcher
  {
  public:
    SrSpikeCatcher();
    virtual ~SrSpikeCatcher();
  private:
    ros::NodeHandle node_;
    ros::NodeHandle node_local_;

    ros::Subscriber debug_subscriber_;

    double last_time_;
    int msg_count_;
    
    int last_[SR_SPIKE_CATCHER_DEBUG_CHANNEL_COUNT],
      previous_[SR_SPIKE_CATCHER_DEBUG_CHANNEL_COUNT],
      ancient_[SR_SPIKE_CATCHER_DEBUG_CHANNEL_COUNT];
    
    void append_sensor_data_(std::stringstream & output);
    void check_sensors_(std::stringstream & output, const sr_robot_msgs::EthercatDebug::ConstPtr& msg);
    void debug_callback_ (const sr_robot_msgs::EthercatDebug::ConstPtr& msg);
    std::ofstream output_stream_;
    std::string output_file_name_;
    std::vector <struct_sensor> sensors_;
    void fill_sensor_list_ ();
    void open_output_ ();
  };
};







#endif
