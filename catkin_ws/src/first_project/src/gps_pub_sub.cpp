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
        double alpha; // Smoothing factor (0 < alpha < 1)

        // Timer for periodic tasks
        double prev_time = 0.0;

        // Global reference positions
        /*double X_r;
        double Y_r;
        double Z_r;*/

        double lat_ref;
        double lon_ref;
        double alt_ref;

        double a = 6378137; // WGS-84 semi-major axis
        double b = 6356752;
        double e2 = 1 - b*b / (a*a); // Square of eccentricity

        Eigen::Vector3d ecef_ref_position;

        int counter = 0;
        double mean_gps_angle = 0;
        bool collecting_heading = true;
        double start_time;
        double sum = 0;
        double smooth_angle;
        double speed;


    public:
        // Constructor: sets up subscribers, publisher, and timer
        GPS_pub_sub() {

            get_reference_position();

            // Subscribe to topics with a queue size of 1
            gps_pos_sub = nh_.subscribe("/swiftnav/front/gps_pose", 1, &GPS_pub_sub::gps_poseCallback, this);

            // Advertise the publisher on "nav_msgs/Odometry" topic
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gps_odom", 1);
        }

        double clamp(double val, double min_val, double max_val) {
            return std::max(min_val, std::min(val, max_val));
        }

        // Callback function for the "/swiftnav/front/gps_pose" topic
        void gps_poseCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
            gps_poseMSG_ = *msg;

            //double curr_time = ros::Time::now().toSec();
            double curr_time = gps_poseMSG_.header.stamp.toSec();

            if (counter == 0) 
                start_time = gps_poseMSG_.header.stamp.toSec(); // getting the start time 
            counter++;

            // Converting gps data from LLA to ECEF
            Eigen::Vector3d ecef_position;
            double lat = gps_poseMSG_.latitude * M_PI / 180.0;
            double lon = gps_poseMSG_.longitude * M_PI / 180.0;
            double alt = gps_poseMSG_.altitude * 0.001; // Convert from mm to m
            //ROS_INFO("GPS Position: %f %f %f", lat, lon, alt);
            double a = 6378137; // WGS-84 semi-major axis
            double b = 6356752;
            double e2 = 1 - b*b / (a*a); // Square of eccentricity
            double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));
            ecef_position(0) = (N + alt) * cos(lat) * cos(lon);
            ecef_position(1) = (N + alt) * cos(lat) * sin(lon);
            ecef_position(2) = (N * (1 - e2) + alt) * sin(lat);

            //ROS_INFO("ECEF Position: %f %f %f", ecef_position(0), ecef_position(1), ecef_position(2));

            // ##Converting gps data from ECEF to ENU
            //Eigen::Vector3d ecef_ref_position = Eigen::Vector3d(X_r, Y_r, Z_r);
            /*double dlat = lat - lat_ref;
            double dlon = lon - lon_ref;
            double dalt = alt - alt_ref;*/

            // Calculate the ENU coordinates
            Eigen::Matrix3d R;
            R << -sin(lon_ref), cos(lon_ref), 0,
                 -sin(lat_ref) * cos(lon_ref), -sin(lat_ref) * sin(lon_ref), cos(lat_ref),
                 cos(lat_ref) * cos(lon_ref), cos(lat_ref) * sin(lon_ref), sin(lat_ref);
                 
            //OS_INFO("Rotation Matrix: %f %f %f", R(0,0), R(0,1), R(0,2));
            //ROS_INFO("references: %f %f %f", lat_ref, lon_ref, alt_ref);
            enu_position = R * (ecef_position - ecef_ref_position);

            // Estimating the heading angle
            denu_position = enu_position - prev_enu_position;

            // computing the absolute speed along (x, y)
            // checking if dt is not zero
            if (curr_time - prev_time > 0) 
                speed = sqrt(pow(denu_position(0), 2) + pow(denu_position(1), 2)) / (curr_time - prev_time);

            if (speed > 0.3) {
                heading_angle = atan2(denu_position(0), denu_position(1));
                alpha = clamp(1.0 - speed, 0.5, 0.9); // adjust alpha based on speed -> it computes the speed and sets alpha to a value between 0.5 and 0.9
                smoothing_algorithm();
                ROS_INFO("Smooth angle when the car is moving: %f", smooth_angle * 180 / M_PI);
                prev_heading_angle = smooth_angle;
            } else {
                // If the speed is low, we can use the previous heading angle
                heading_angle = prev_heading_angle;
                smooth_angle = heading_angle;
                ROS_INFO("Smooth angle when the car is not moving: %f", smooth_angle * 180 / M_PI);
            }

            /*if (curr_time - start_time < ros::Duration(14.0).toSec()) {
                alpha = 0.8;  // Smoothing the measurements when the vehicle is not moving ->
                // in this case we are settung a higher alpha such that the smoothing relies more on the previous filtered values
                smoothing_algorithm();
                ROS_INFO("Smooth angle when the car is not moving: %f", smooth_angle * 180 / M_PI);
            } else {
                // Smoothing the heading angle
                alpha = 0.5;  // Smoothing the measurements when the vehicle is moving
                smoothing_algorithm();
                ROS_INFO("Smooth angle when the car is moving: %f", smooth_angle * 180 / M_PI);
            }*/

            // Publish the odometry message
            nav_msgs::Odometry odom;
            odom.header.stamp = gps_poseMSG_.header.stamp;
            odom.header.frame_id = "gps_odom";
            odom.child_frame_id = "gps";
            odom.pose.pose.position.x = enu_position(0);
            odom.pose.pose.position.y = enu_position(1);
            odom.pose.pose.position.z = enu_position(2);
            //ROS_INFO("ENU Position: %f %f %f", enu_position(0), enu_position(1), enu_position(2));
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
            q.setRPY(0, 0, heading_angle);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "gps_odom", "gps"));

            prev_enu_position = enu_position;
            prev_time = curr_time;
            prev_filtered_value = enu_position;
        }

        void smoothing_algorithm() {
            // Implementing a lpf in order to smooth the GPS data before publishing
            smooth_angle = alpha * prev_heading_angle + (1 - alpha) * heading_angle;
        }

        void get_reference_position() {
            // Get the reference position from the parameter server
            if (nh_.getParam("/lat_r", reference_latitude)) {
                ROS_INFO("Global parameter: %f", reference_latitude);
            } else {
                ROS_WARN("Global parameter not found, using default value");
            }

            if (nh_.getParam("/lon_r", reference_longitude)) {
                ROS_INFO("Global parameter: %f", reference_longitude);
            } else {
                ROS_WARN("Global parameter not found, using default value");
            }

            if (nh_.getParam("/alt_r", reference_altitude)) {
                ROS_INFO("Global parameter: %f", reference_altitude);
            } else {
                ROS_WARN("Global parameter not found, using default value");
            }

            // Convert reference latitude, longitude, and altitude to radians and meters
            lat_ref = reference_latitude * M_PI / 180.0;
            lon_ref = reference_longitude * M_PI / 180.0;
            alt_ref = reference_altitude * 0.001; // Convert from mm to m
            double N_ref = a / sqrt(1 - e2 * sin(lat_ref) * sin(lat_ref));
            ecef_ref_position(0) = (N_ref + alt_ref) * cos(lat_ref) * cos(lon_ref);
            ecef_ref_position(1) = (N_ref + alt_ref) * cos(lat_ref) * sin(lon_ref);
            ecef_ref_position(2) = (N_ref * (1 - e2) + alt_ref) * sin(lat_ref);
            //ROS_INFO("ECEF Reference Position: %f %f %f", ecef_ref_position(0), ecef_ref_position(1), ecef_ref_position(2));
            //ROS_INFO("Reference Latitude,Longitude,Altitude: %f %f %f", reference_latitude, reference_longitude, reference_altitude);
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_odometer"); // Initialize the ROS node with the name "gps_odometer" 
    //GPS_pub_sub gps_data; // Call the function to get the reference latitude, longitude, and altitude
    //gps_data.get_reference_position(); // Create an instance of the GPS_pub_sub class
    GPS_pub_sub gps_odometry; 
    ros::spin(); // Keep the node running and processing callbacks
    return 0;
}

