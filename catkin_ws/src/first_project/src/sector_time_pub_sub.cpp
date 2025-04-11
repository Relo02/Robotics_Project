#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "first_project/Custom.h"
#include "sensor_msgs/NavSatFix.h"
#include <cmath>

class SectorTimePubSub {
  private:
      // ROS node handle
      ros::NodeHandle nh_;

      //Subscribers
      ros::Subscriber speedsteer_sub_;
      ros::Subscriber gps_pos_sub_;
      // Publisher for custom message
      ros::Publisher sector_time_pub_;
      //  Subscribing pose
      sensor_msgs::NavSatFix gps_poseMSG_;
      // Subscribing speedsteer
      geometry_msgs::PointStamped speedsteerMSG_;
      // Publishing custom message
      first_project::Custom customMSG_;


      //Sector Flag initilization 
      int sector_flag = 1; // 1: sector1, 2: sector2, 3: sector3

      //Declare Message Variables
      double speed; 
      double latitude;
      double longitude;
      double altitude;

      //Delcare LLA holding arrays
      double current_LLA[3];
      const double sector_2_LLA[3] = {45.62988851421235, 9.288771322131693, 237.17390273898968}; // Replace with actual LLA values
      const double sector_3_LLA[3] = {45.62326750069244, 9.288356463257482, 230.00294296684058}; // Replace with actual LLA values
      const double sector_1_LLA[3] = {45.61636342409999, 9.280814244709745, 227.5143122522162}; // Replace with actual LLA values
      const double starting_LLA[3] = {45.61893238659240, 9.281178887031235, 229.04906147731415}; // Replace with actual LLA values

      // Declare variables for the sector time
      int counter_1 = 0;
      int counter_2 = 0;
      int counter_3 = 0;


      double totalspeed_1 = 0.0;
      double totalspeed_2 = 0.0;  
      double totalspeed_3 = 0.0;

      // //Declare time variables
      // int sec = 1574431772;
      // int nsec = 358392953;

      // ros::Time sectorstart_time;
      // sectorstart_time.secs = sec;
      // sectorstart_time.nsecs = nsec;
      // double sectorstart_time = start_time.toSec();
      ros::Time sectorstart_time = ros::Time(1574431772) + ros::Duration(0.358392953);


      
      ros::Time curr_time;
      double sector_time;
      double totalsector_time_1 = 0.0;
      double totalsector_time_2 = 0.0;
      double totalsector_time_3 = 0.0;

      //Declare speed variable speed
      double meanspeed = 0.0;
      double meanspeed_1 = 0.0;
      double meanspeed_2 = 0.0;
      double meanspeed_3 = 0.0;

      //Truncate factor
      double factor = std::pow(10.0, 3);
      double lat;


 
      


      

  public:
      // Constructor: sets up subscribers, publisher, and timer
      SectorTimePubSub() {

        gps_pos_sub_ = nh_.subscribe("/swiftnav/front/gps_pose", 1, &SectorTimePubSub::gps_poseCallback, this);
        speedsteer_sub_ = nh_.subscribe("/speedsteer", 1, &SectorTimePubSub::speedsteerCallback,this);
      
        sector_time_pub_ = nh_.advertise<first_project::Custom>("sector_times", 10);
      

      };
        // Call back function for speedsteer
      void speedsteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
            speedsteerMSG_ = *msg;
            //Extract speed, steer input,and time from msg
            speed = speedsteerMSG_.point.y; // Speed in km/h

            //ROS_INFO("Last Known Speed: %f", speed);
        }

      void  gps_poseCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
        gps_poseMSG_ = *msg;
      
        // Extract the GPS data
        latitude = gps_poseMSG_.latitude;
        longitude = gps_poseMSG_.longitude;
        altitude = gps_poseMSG_.altitude;

        //Extract currrent time
        curr_time = gps_poseMSG_.header.stamp;
        //ROS_INFO("Current TIme: %f", curr_time);

        //Save current LLA into an array
        current_LLA[0] = latitude;
        current_LLA[1] = longitude;
        current_LLA[2] = altitude; 
        //ROS_INFO("Current LLA: %f %f %f", current_LLA[0], current_LLA[1], current_LLA[2]);

        // lat = std::trunc(latitude * factor) / factor;
        // ROS_WARN("Truncated Latitude: %f", lat);

        // ROS_WARN("Truncated Latitude: %f", std::trunc(sector_2_LLA[0]*factor)/factor);

        
        if(fabs(latitude - sector_2_LLA[0])<=1e-14){
            sector_flag = 2;
            sectorstart_time = curr_time;
            ROS_WARN("Sector 2");
        }
        else if (fabs(latitude - sector_3_LLA[0])<=1e-14){
            sector_flag = 3;
            sectorstart_time = curr_time;
            ROS_WARN("Sector 3");
      }
        else if (fabs(latitude - sector_1_LLA[0])<=1e-14){
            sector_flag = 1;
            sectorstart_time = curr_time;
            ROS_WARN("Sector 1");
        }
        // else if {
        //     sector_flag = 1;
        //     sectorstart_time = 1350862497278.7195;
        //     ROS_WARN("Still Default");
        // }
        ROS_INFO("Current Time: %f", curr_time.toSec());
        ROS_INFO("Start Sector Time: %f", sectorstart_time.toSec());

        
        

        // Calculate the sector time based on the sector flag
        if(sector_flag == 1) {
            counter_1++;
            totalspeed_1 += speed;
            meanspeed_1 = totalspeed_1 / counter_1;
            totalsector_time_1 += fabs(curr_time.toSec() - sectorstart_time.toSec());
            sector_time = totalsector_time_1;
            meanspeed = meanspeed_1;
          }
         else if (sector_flag == 2) {
            counter_2++;
            totalspeed_2 += speed;
            meanspeed_2 = totalspeed_2 / counter_2;
            totalsector_time_2 += fabs(curr_time.toSec() - sectorstart_time.toSec());
            sector_time = totalsector_time_2;
            meanspeed = meanspeed_2;
          }
        else if (sector_flag == 3) {
            counter_3++; 
            totalspeed_3 += speed;
            meanspeed_3 = totalspeed_3 / counter_3;
            totalsector_time_3 += fabs(curr_time.toSec() - sectorstart_time.toSec());
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
