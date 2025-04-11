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

      //Sector Flag initilization 
      int sector_flag = 1; // 1: sector1, 2: sector2, 3: sector3

      //Declare Message Variables
      double speed; 
      double latitude;
      double longitude;
      double altitude;

      //Delcare LLA holding arrays
      double current_LLA[3];
      double sector_2_LLA[3] = {0.0, 0.0, 0.0}; // Replace with actual LLA values
      double sector_3_LLA[3] = {0.0, 0.0, 0.0}; // Replace with actual LLA values
      double sector_1_LLA[3] = {0.0, 0.0, 0.0}; // Replace with actual LLA values
      double starting_LLA[3] = {0.0, 0.0, 0.0}; // Replace with actual LLA values

      // Declare variables for the sector time
      int counter_1 = 0;
      int counter_2 = 0;
      int counter_3 = 0;


      double totalspeed_1 = 0.0;
      double totalspeed_2 = 0.0;  
      double totalspeed_3 = 0.0;

      //Declare time variables
      double sectorstart_time = 0.0;
      double curr_time;
      double sector_time;
      double totalsector_time_1 = 0.0;
      double totalsector_time_2 = 0.0;
      double totalsector_time_3 = 0.0;

      //Declare speed variable speed
      double meanspeed = 0.0;
      double meanspeed_1 = 0.0;
      double meanspeed_2 = 0.0;
      double meanspeed_3 = 0.0;


 
      


      

  public:
      // Constructor: sets up subscribers, publisher, and timer
      SectorTimePubSub() : gps_pos_sub_(nh_, "/swiftnav/front/gps_pose", 1), speedsteer_sub_(nh_, "/speedsteer", 1), sync_(SyncPolicy(10), gps_pos_sub_, speedsteer_sub_) {
      
        sector_time_pub_ = nh_.advertise<first_project::Custom>("sector_times", 10);
      
        // Register callback for synchronized messages
        sync_.registerCallback(boost::bind(&SectorTimePubSub::Callback, this, _1, _2));
      };

      void Callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg, const geometry_msgs::PointStamped::ConstPtr& speed_msg ){
        // Extract the speed and steer input from the message
        speed = speed_msg->point.x; // Speed in km/h
        
        // Extract the GPS data
        latitude = gps_msg->latitude;
        longitude = gps_msg->longitude;
        altitude = gps_msg->altitude;

        //Extract currrent time
        curr_time = gps_poseMSG_.header.stamp.toSec();

        //Save current LLA into an array
        current_LLA[0] = latitude;
        current_LLA[1] = longitude;
        current_LLA[2] = altitude; 

        
        if(current_LLA == sector_2_LLA){
            sector_flag = 2;
            sectorstart_time = curr_time;
        }
        else if (current_LLA == sector_3_LLA){
            sector_flag = 3;
            sectorstart_time = curr_time;
      }
        else if (current_LLA == sector_1_LLA){
            sector_flag = 1;
            sectorstart_time = curr_time;
        }
        else{
            sector_flag = 1;
            sectorstart_time = 0;
        }

        

        // Calculate the sector time based on the sector flag
        if(sector_flag == 1) {
            counter_1++;
            totalspeed_1 += speed;
            meanspeed_1 = totalspeed_1 / counter_1;
            totalsector_time_1 += curr_time - sectorstart_time;
            sector_time = totalsector_time_1;
            meanspeed = meanspeed_1;
          }
         else if (sector_flag == 2) {
            counter_2++;
            totalspeed_2 += speed;
            meanspeed_2 = totalspeed_2 / counter_2;
            totalsector_time_2 += curr_time - sectorstart_time;
            sector_time = totalsector_time_2;
            meanspeed = meanspeed_2;
          }
        else if (sector_flag == 3) {
            counter_3++; 
            totalspeed_3 += speed;
            meanspeed_3 = totalspeed_3 / counter_3;
            totalsector_time_3 += curr_time - sectorstart_time;
            sector_time = totalsector_time_3;
            meanspeed = meanspeed_3;
        }

        

        // Create a custom message to publish
        customMSG_.header.stamp = ros::Time::now();
        customMSG_.header.frame_id = "sector_time";
        customMSG_.current_sector = sector_flag;
        customMSG_.current_sector_time = sector_time;
        customMSG_.current_sector_mean_speed = meanspeed;

        // Publish the custom message
        sector_time_pub_.publish(customMSG_);
      
      }
};

  



int main(int argc, char **argv) {
  ros::init(argc, argv, "sector_time"); // Initialize the ROS node with the name sector_time
  SectorTimePubSub  sector_time; 
  ros::spin(); // Keep the node running and processing callbacks
  return 0;
}
