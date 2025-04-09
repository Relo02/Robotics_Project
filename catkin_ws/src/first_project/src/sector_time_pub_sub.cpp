#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "first_project/Custom.h"
#include <cmath>

class SectorTimePubSub {
  private:
      // ROS node handle
      ros::NodeHandle nh_;

      // Subscriber for "/speedsteer" topic
      ros::Subscriber speedsteer_sub_;
      ros::Subscriber gps_pos_sub_;

      ros::Publisher sector_time_pub_;

      //  Subscribing pose
      sensor_msgs::NavSatFix gps_poseMSG_;
      // Subscribing speedsteer
      geometry_msgs::PointStamped speedsteerMSG_;
      // Publishing custom message
      first_project::Custom customMSG_;


  public:
      // Constructor: sets up subscribers, publisher, and timer
      SectorTimePubSub() {

          // Subscribe to topics with a queue size of 1
          gps_pos_sub_ = nh_.subscribe("/swiftnav/front/gps_pose", 1, &SectorTimePubSub::gps_poseCallback, this);
          // Subscribe to "/speedsteer" topic 
          speedsteer_sub_ = nh_.subscribe("/speedsteer", 1, &SectorTimePubSub::speedsteerCallback,this);
          // Advertise the publisher on "nav_msgs/Odometry" topic
          sector_time_pub_ = nh_.advertise<first_project::Custom>("chatter", 10);
    };

    void speedsteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
      

    } 

    void gps_poseCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){

      
    }


};

int main(int argc, char **argv) {
  ros::init(argc, argv, "sector_time"); // Initialize the ROS node with the name odometer
  SectorTimePubSub  sector_time; 
  ros::spin(); // Keep the node running and processing callbacks
  return 0;
}
