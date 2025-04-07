#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <cmath>

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

        // Transform broadcaster
        tf::TransformBroadcaster br;

        // Transform and quaternion as class members to reuse them in each callback
        tf::Quaternion q;

        // Transform to store the transform messages
        tf::Transform transform;

        // Variables for ENU coordinates
        Eigen::Vector3d enu_position;
        double heading_angle = 0.0;
        Eigen::Vector3d prev_enu_position = Eigen::Vector3d::Zero();

        // Variables for denu coordinates
        Eigen::Vector3d denu_position;

        // Previous filtered value for smoothing
        Eigen::Vector3d prev_filtered_value = Eigen::Vector3d::Zero();
        double prev_heading_angle = 0.0;
        double alpha = 0.1; // Smoothing factor (0 < alpha < 1)

        // Timer for periodic tasks
        double prev_time = 0.0;

    public:
        // Constructor: sets up subscribers, publisher, and timer
        GPS_pub_sub() {

            // Calling the global reference latitude
            if (nh_.getParam("/lat_r", reference_latitude)) {
                ROS_INFO("Global parameter: %f", reference_latitude);
            } else {
                ROS_WARN("Global parameter not found, using default value");
            }

            // Calling the global reference longitude
            if (nh_.getParam("/lon_r", reference_longitude)) {
                ROS_INFO("Global parameter: %f", reference_longitude);
            } else {
                ROS_WARN("Global parameter not found, using default value");
            }

            // Calling the global reference altitude
            if (nh_.getParam("/alt_r", reference_altitude)) {
                ROS_INFO("Global parameter: %f", reference_altitude);
            } else {
                ROS_WARN("Global parameter not found, using default value");
            }

            // Subscribe to topics with a queue size of 1
            gps_pos_sub = nh_.subscribe("/swiftnav/front/gps_pose", 1, &GPS_pub_sub::gps_poseCallback, this);

            // Advertise the publisher on "nav_msgs/Odometry" topic
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gps_odom", 1);

            // Create a timer that triggers every 1 second to publish messages
            //timer_ = nh_.createTimer(ros::Duration(1.0), &PubSubNode::timerCallback, this);
        }

        // Callback function for the "/swiftnav/front/gps_pose" topic
        void gps_poseCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
            gps_poseMSG_ = *msg;

            // Process the odometry message here

            double curr_time = ros::Time::now().toSec();

            // Converting gps data from LLA to ECEF
            Eigen::Vector3d ecef_position;
            double lat = gps_poseMSG_.latitude * M_PI / 180.0;
            double lon = gps_poseMSG_.longitude * M_PI / 180.0;
            double alt = gps_poseMSG_.altitude * 0.001; // Convert from mm to m
            double a = 6378137; // WGS-84 semi-major axis
            double b = 6356752;
            double e2 = 1 - b*b / (a*a); // Square of eccentricity
            double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));
            ecef_position(0) = (N + alt) * cos(lat) * cos(lon);
            ecef_position(1) = (N + alt) * cos(lat) * sin(lon);
            ecef_position(2) = (N * (1 - e2) + alt) * sin(lat);

            // ##Converting gps data from ECEF to ENU
            double lat_ref = reference_latitude * M_PI / 180.0;
            double lon_ref = reference_longitude * M_PI / 180.0;
            double alt_ref = reference_altitude * 0.001; // Convert from mm to m
            Eigen::Vector3d ecef_ref_position = Eigen::Vector3d(lat_ref, lon_ref, alt_ref);
            /*double dlat = lat - lat_ref;
            double dlon = lon - lon_ref;
            double dalt = alt - alt_ref;*/

            // Calculate the ENU coordinates
            Eigen::Matrix3d R;
            R << -sin(lon_ref), cos(lon_ref), 0,
                 -sin(lat_ref) * cos(lon_ref), -sin(lat_ref) * sin(lon_ref), cos(lat_ref),
                 cos(lat_ref) * cos(lon_ref), cos(lat_ref) * sin(lon_ref), sin(lat_ref);
            
            enu_position = R * (ecef_position - ecef_ref_position);

            // Estimating the heading angle
            denu_position = enu_position - prev_enu_position;
            heading_angle = atan2(denu_position(1), denu_position(0));

            // Smoothing the heading angle
            //smoothing_algorithm();

            // Publish the odometry message
            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = "gps";
            odom.child_frame_id = "gps_odom";
            odom.pose.pose.position.x = enu_position(0);
            odom.pose.pose.position.y = enu_position(1);
            odom.pose.pose.position.z = enu_position(2);
            odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading_angle);

            // computing the velocity
            if (curr_time - prev_time > 0) {
                odom.twist.twist.linear.x = (enu_position(0) - prev_enu_position(0)) / (curr_time - prev_time);
                odom.twist.twist.linear.y = (enu_position(1) - prev_enu_position(1)) / (curr_time - prev_time);
                odom.twist.twist.linear.z = (enu_position(2) - prev_enu_position(2)) / (curr_time - prev_time);
                odom.twist.twist.angular.x = 0;
                odom.twist.twist.angular.y = 0;
                odom.twist.twist.angular.z = (heading_angle - prev_heading_angle) / (curr_time - prev_time);
                odom_pub_.publish(odom);
            } else {
                odom.twist.twist.linear.x = 0;
                odom.twist.twist.linear.y = 0;
                odom.twist.twist.linear.z = 0;
                odom.twist.twist.angular.x = 0;
                odom.twist.twist.angular.y = 0;
                odom.twist.twist.angular.z = 0;
                odom_pub_.publish(odom);
            }

            // Publish the tf transform from gps odometry to gps frame
            transform.setOrigin(tf::Vector3(enu_position(0), enu_position(1), enu_position(2)));
            q.setRPY(0, 0, 0); // TODO: add the yaw rotation in q.setRPY()
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "gps", "gps_odom"));

            prev_enu_position = enu_position;
            prev_time = curr_time;
            prev_filtered_value = enu_position;
            prev_heading_angle = heading_angle;
        }

        void smoothing_algorithm() {
            // Implementing a moving avarage
            // to smooth the GPS data before publishing
            enu_position = alpha * prev_filtered_value + (1 - alpha) * enu_position;
            heading_angle = alpha * prev_heading_angle + (1 - alpha) * heading_angle;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_odometer"); // Initialize the ROS node with the name "gps_odometer" 
    GPS_pub_sub gps_odometry; 
    ros::spin(); // Keep the node running and processing callbacks
    return 0;
}

