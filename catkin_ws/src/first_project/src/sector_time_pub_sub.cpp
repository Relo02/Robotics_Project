#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "first_project/Custom.h"
#include "sensor_msgs/NavSatFix.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <cmath>

class SectorTimePubSub {
  private:
      // ROS node handle
      ros::NodeHandle nh_;

      //message filtered subscribers
      message_filters::Subscriber<geometry_msgs::PointStamped> speedsteer_sub_;
      message_filters::Subscriber<sensor_msgs::NavSatFix> gps_pos_sub_;
      // Publisher for custom message
      ros::Publisher sector_time_pub_;
      //  Subscribing pose
      sensor_msgs::NavSatFix gps_poseMSG_;
      // Subscribing speedsteer
      geometry_msgs::PointStamped speedsteerMSG_;
      // Publishing custom message
      first_project::Custom customMSG_;

      // Synchronizer for message filters
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::PointStamped> SyncPolicy;
      message_filters::Synchronizer<SyncPolicy> sync_;

      

  public:
      // Constructor: sets up subscribers, publisher, and timer
      SectorTimePubSub() : gps_pos_sub_(nh_, "/swiftnav/front/gps_pose", 1), speedsteer_sub_(nh_, "/speedsteer", 1), sync_(SyncPolicy(10), gps_pos_sub_, speedsteer_sub_) {
      
        sector_time_pub_ = nh_.advertise<first_project::Custom>("sector_times", 10);
      
        // Register callback for synchronized messages
        sync_.registerCallback(boost::bind(&SectorTimePubSub::Callback, this, _1, _2));
      };

      void Callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg, const geometry_msgs::PointStamped::ConstPtr& speed_msg ){
      

      } 

  


};

int main(int argc, char **argv) {
  ros::init(argc, argv, "sector_time"); // Initialize the ROS node with the name sector_time
  SectorTimePubSub  sector_time; 
  ros::spin(); // Keep the node running and processing callbacks
  return 0;
}
