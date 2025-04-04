#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Dense>

class GPS_pub_sub {
    private:
        // ROS node handle
        ros::NodeHandle nh_;

        // Subscribers for the "/swiftnav/front/gps_pose"
        ros::Subscriber gps_pos_sub;

        // Publisher for the "nav_msgs/Odometry" topic
        ros::Publisher odom_pub_;

        //  Subscribing pose
        sensor_msgs::NavSatFix gps_poseMSG_;

        // Global string parameter
        double reference_latitude, reference_longitude, reference_altitude;

    public:
        // Constructor: sets up subscribers, publisher, and timer
        GPS_pub_sub() {

            // Calling the global reference latitude
            if (nh_.getParam("/lat_r", reference_latitude)) {
                ROS_INFO("Global parameter: %s", reference_latitude);
            } else {
                ROS_WARN("Global parameter not found, using default value");
            }

            // Calling the global reference longitude
            if (nh_.getParam("/lon_r", reference_longitude)) {
                ROS_INFO("Global parameter: %s", reference_longitude);
            } else {
                ROS_WARN("Global parameter not found, using default value");
            }

            // Calling the global reference altitude
            if (nh_.getParam("/alt_r", reference_altitude)) {
                ROS_INFO("Global parameter: %s", reference_altitude);
            } else {
                ROS_WARN("Global parameter not found, using default value");
            }

            // Subscribe to topics with a queue size of 1
            gps_pos_sub = nh_.subscribe("/swiftnav/front/gps_pose", 1, &GPS_pub_sub::gps_poseCallback, this);

            // Advertise the publisher on "nav_msgs/Odometry" topic
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gps_odom", 1, &GPS_pub_sub::gps_odomCallback, this);

            // Create a timer that triggers every 1 second to publish messages
            //timer_ = nh_.createTimer(ros::Duration(1.0), &PubSubNode::timerCallback, this);
        }

        // Callback function for the "/swiftnav/front/gps_pose" topic
        void gps_poseCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
            gps_poseMSG_ = *msg;
        }

        // Callback function for the "nav_msgs/Odometry" topic
        void gps_odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
            // Process the odometry message here

            // Converting gps data from LLA to ECEF
            Eigen::Vector3d ecef_position;
            double lat = gps_poseMSG_.latitude;
            double lon = gps_poseMSG_.longitude;
            double alt = gps_poseMSG_.altitude;
            double a = 6378137.0; // WGS-84 semi-major axis
            double b = b=6356752;
            double e2 = 1 - b*b / (a*a); // Square of eccentricity
            double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));
            ecef_position(0) = (N + alt) * cos(lat) * cos(lon);
            ecef_position(1) = (N + alt) * cos(lat) * sin(lon);
            ecef_position(2) = (N * (1 - e2) + alt) * sin(lat);

            // ##Converting gps data from ECEF to ENU
            Eigen::Vector3d enu_position;
            double lat_ref = reference_latitude;
            double lon_ref = reference_longitude;
            double alt_ref = reference_altitude;
            Eigen::Vector3d ecef_ref_position = Eigen::Vector3d(lat_ref, lon_ref, alt_ref);
            double dlat = lat - lat_ref;
            double dlon = lon - lon_ref;
            double dalt = alt - alt_ref;

            // Calculate the ENU coordinates
            Eigen::Matrix3d R;
            R << -sin(lon_ref), cos(lon_ref), 0,
                 -sin(lat_ref) * cos(lon_ref), -sin(lat_ref) * sin(lon_ref), cos(lat_ref),
                 cos(lat_ref) * cos(lon_ref), cos(lat_ref) * sin(lon_ref), sin(lat_ref);
            
            enu_position = R * (ecef_position - ecef_ref_position);
        }

        // Timer callback function: publishes messages periodically
        /*void timerCallback(const ros::TimerEvent&) {
            // Publish the latest messages from both subscribers
            rechatter_pub_.publish(chatterMsg_);
            rechatter_pub_.publish(chatter2Msg_);
            ROS_INFO("Timer callback executed: messages published.");
        }*/
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_odometer"); // Initialize the ROS node with the name "gps_odometer" 
    GPS_pub_sub gps_odometry; 
    ros::spin(); // Keep the node running and processing callbacks
    return 0;
}

