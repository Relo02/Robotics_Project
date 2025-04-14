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
      int sector_flag = 0; // 1: sector1, 2: sector2, 3: sector3

      //Declare Message Variables
      double speed; 
      double latitude;
      double longitude;
      double altitude;

      //Delcare LLA holding arrays
      double current_LLA[3];
      double sector_2_LLA[3] = {45.62988851421235, 9.288771322131693, 237.17390273898968}; // Replace with actual LLA values
      double sector_3_LLA[3] = {45.62326750069244, 9.288356463257482, 230.00294296684058}; // Replace with actual LLA values
      double sector_1_LLA[3] = {45.61636342409999, 9.280814244709745, 227.5143122522162}; // Replace with actual LLA values
      double starting_LLA[3] = {45.61893238659240, 9.281178887031235, 229.04906147731415}; // Replace with actual LLA values

      // Declaring sector times list
      double sector_times[3] = {0.0, 0.0, 0.0}; 
      double partial_sector_time_1 = 0.0; // Variable to store the time spent in sector 1 before entering sector 2, to be added to sector 1 time when returning to it in the last part of the lap

      // Declaring starting times for each sector -> Values retrived from the GPS in project.bag file
      double sector_1_start_time = 382.25; 
      double sector_2_start_time = 119.0; 
      double sector_3_start_time = 262.54;
      
      // Declare variables for the sector time
      int counter_1 = 0;
      int counter_2 = 0;
      int counter_3 = 0;


      double totalspeed_1 = 0.0;
      double totalspeed_2 = 0.0;  
      double totalspeed_3 = 0.0;

      //Declare time variables
      //double sectorstart_time = 0.0;
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

      double fs_gps = 10.0; // GPS frequency in Hz
      double prev_time = 0.0;

  public:
      // Constructor: sets up subscribers, publisher, and timer
      SectorTimePubSub() : gps_pos_sub_(nh_, "/swiftnav/front/gps_pose", 1), speedsteer_sub_(nh_, "/speedsteer", 1), sync_(SyncPolicy(10), gps_pos_sub_, speedsteer_sub_) {
      
        sector_time_pub_ = nh_.advertise<first_project::Custom>("sector_times", 10);
      
        // Register callback for synchronized messages
        sync_.registerCallback(boost::bind(&SectorTimePubSub::Callback, this, _1, _2));
      };

      double truncateDecimals(double value, int decimals) {
        double factor = pow(10.0, decimals);
        return trunc(value * factor) / factor;
      }

      bool compareLLA(double a[3], double b[3], double epsilon = 0.005) {
        return (fabs(a[0] - b[0]) < epsilon &&
                fabs(a[1] - b[1]) < epsilon &&
                fabs(a[2] - b[2]) < epsilon);
      }

      void get_sector_times_flags() {
        if (compareLLA(current_LLA, sector_2_LLA) && sector_flag != 2) {
          sector_flag = 2;
          partial_sector_time_1 = sector_time;
          sector_time = 0.0;
        }
        else if (compareLLA(current_LLA, sector_3_LLA) && sector_flag != 3) {
          sector_flag = 3;
          sector_time = 0.0;
        }
        else if (compareLLA(current_LLA, sector_1_LLA) && sector_flag != 1) {
          sector_flag = 1;
          sector_time = partial_sector_time_1;
        }
        else if (compareLLA(current_LLA, starting_LLA) && sector_flag != 1 && sector_flag != 3) {
          sector_flag = 1;
          sector_time = 0.0;
        }
      }

      void Callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg, const geometry_msgs::PointStamped::ConstPtr& speed_msg ){
        // Extract the speed and steer input from the message
        speed = speed_msg->point.y; // Speed in km/h
        
        // Extract the GPS data
        latitude = gps_msg->latitude;
        longitude = gps_msg->longitude;
        altitude = gps_msg->altitude;

        // Call get_sector_times_flags() first to update sector_flag and set start_time once
        get_sector_times_flags();

        // Computing sector time
        //curr_time = gps_poseMSG_.header.stamp.toSec();
        curr_time = ros::Time::now().toSec();
        double dt = curr_time - prev_time;
        sector_time += dt;

        //Save current LLA into an array
        /*current_LLA[0] = truncateDecimals(latitude, 4);
        current_LLA[1] = truncateDecimals(longitude, 4);
        current_LLA[2] = truncateDecimals(altitude, 4);*/
        current_LLA[0] = latitude;
        current_LLA[1] = longitude;
        current_LLA[2] = altitude;
        ROS_INFO("Current LLA: [%f, %f, %f]", current_LLA[0], current_LLA[1], current_LLA[2]);

        if (sector_flag == 1 && dt >= 1 / fs_gps) {
            counter_1++;
            totalspeed_1 += speed;
            meanspeed_1 = totalspeed_1 / counter_1;
            meanspeed = meanspeed_1;
        } else if (sector_flag == 2 && dt >= 1 / fs_gps) {
            counter_2++;
            totalspeed_2 += speed;
            meanspeed_2 = totalspeed_2 / counter_2;
            meanspeed = meanspeed_2;
        } else if (sector_flag == 3 && dt >= 1 / fs_gps) {
            counter_3++;
            totalspeed_3 += speed;
            meanspeed_3 = totalspeed_3 / counter_3;
            meanspeed = meanspeed_3;
        }

        prev_time = curr_time;

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
