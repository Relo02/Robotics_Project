#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <cmath>
#include "Kalman_filter.h"
#include "first_project/Custom.h"

class EKF_pub_sub {
    private:
      //ROS node handle
      ros::NodeHandle nh_;

      //Create transform broadcaster 
      tf::TransformBroadcaster br_;
      // Declare transform and quaternion as class members to reuse them in each callback
      tf::Transform transform_; //Store transform messages
      tf::Quaternion q_; //Message in the form of a quaternion
      
      // Publisher for "/ekf" topic
      ros::Publisher ekf_pub_;

      // boolean variable for kf usage
      bool use_kf = true;
      //message filtered subscribers
      message_filters::Subscriber<geometry_msgs::PointStamped> speedsteer_sub_;
      message_filters::Subscriber<nav_msgs::Odometry> gps_sub_;

      // Subscribing speedsteer
      geometry_msgs::PointStamped speedsteerMSG_;

      //Subscribing place holder variables
      nav_msgs::Odometry gps_poseMSG_;

      // Publishing odometry
      nav_msgs::Odometry ekfMSG_;

      // Synchronizer for message filters
      typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PointStamped> SyncPolicy;
      message_filters::Synchronizer<SyncPolicy> sync_;

      //Declare parameter values
      double steering_factor_; // Steering factor
      double d_; // Distance between wheels
      double b_; // Real wheel baseline

      KalmanFilter *kf_; // Kalman filter object


      // Intialize speedsteer message variables
      double speed; //Speed(km/h)
      double steer; //Steer at steering angle(deg)
      double alpha; //Steer angle with steering factor(deg)
      ros::Time prev_time; //Last time to get deltaT
      ros::Time curr_time; // Current time to get deltaT
      double dT; // Delta time
      double ome; //Omega value[rad/s]
      double V_f; //Linear Velocity[m/s]
      double R; //Radius[]
      Eigen::VectorXd KF_odom; // Updated KF data
      double a_bias = -7.2 * M_PI / 180; // Bias for steering angle


    public:
      // Constructor: sets up subscriber and publishers
      EKF_pub_sub() : gps_sub_(nh_, "/gps_odom", 1), speedsteer_sub_(nh_, "/speedsteer", 1), sync_(SyncPolicy(10), gps_sub_, speedsteer_sub_){

        if (nh_.getParam("steering_factor", steering_factor_))
        {
          ROS_INFO("Steering Factor: %f", steering_factor_);
        }
        else
        {
          ROS_WARN("Missing required Steering Factor parameter, using default value");
        }

        if (nh_.getParam("d", d_))
        {
          ROS_INFO("Distance Between Wheels Parameter: %f", d_);
        }
        else
        {
          ROS_WARN("Missing required Distance Between Wheels parameter, using default value");
        }

        if (nh_.getParam("b", b_))
        {
          ROS_INFO("Wheel Baseline Parameter: %f", b_);
        }
        else
        {
          ROS_WARN("Missing required Wheel Baseline Parameter, using default value");
        }

        // Advertise the publisher on "/ekf" topic
        ekf_pub_ = nh_.advertise<nav_msgs::Odometry>("/ekf", 10);

        // Initialize Kalman filter with initial state, covariance, process noise, and GPS noise
        Eigen::VectorXd initial_state(5);
        initial_state << 0, 0, 0, 0, 0; // Initial state [x, y, theta, Vx, Vy]
        Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(5, 5) * 1e-3; // Initial covariance
        Eigen::MatrixXd process_noise = Eigen::MatrixXd::Identity(5, 5) * 1e-3; // Process noise
        Eigen::MatrixXd gps_noise = Eigen::MatrixXd::Identity(2, 2) * 1e-3; // GPS noise

        kf_ = new KalmanFilter(initial_state, initial_covariance, process_noise, gps_noise);
        // Register callback for synchronized messages
        sync_.registerCallback(boost::bind(&EKF_pub_sub::ekfCallback, this, _1, _2));
      }

      void ekfCallback(const nav_msgs::Odometry::ConstPtr& gps_msg, const geometry_msgs::PointStamped::ConstPtr& speedsteer_msg) {
        // Extract the speed and steer input from the message

        speed = speedsteer_msg->point.y; // Speed in km/h
        steer = speedsteer_msg->point.x; // Steering angle in degrees

        // Extract the GPS data
        double gps_x = gps_msg->pose.pose.position.x;
        double gps_y = gps_msg->pose.pose.position.y;

        curr_time = ros::Time::now();
        dT = (curr_time - prev_time).toSec();

        //Compute steering angle
        alpha = ((steer* M_PI / 180.0)-a_bias)/steering_factor_; 

        if (fabs(alpha)<= 1e-6){
          alpha = 1e-6;
        }
        // ROS_INFO("Steering Angle(deg): %f", alpha);

        if (dT <= 0) {
          dT = 0.01; // Set a small positive value to avoid division by zero
        }

        //Compute omega, Vf, R
        ome = (speed/3.6)*(tan(alpha)/d_);
        //V_f = V/3.6;
        V_f = (ome*d_)/sin(alpha);

        kf_->predict(dT, V_f, ome);

        kf_->update(Eigen::Vector2d(gps_x, gps_y));

        KF_odom = Eigen::VectorXd(kf_->getState());
        
        // Publish the ekf message
        ekfMSG_.header.stamp = curr_time;
        ekfMSG_.header.frame_id = "world";
        ekfMSG_.child_frame_id = "local";
        ekfMSG_.pose.pose.position.x = KF_odom(0); 
        ekfMSG_.pose.pose.position.y = KF_odom(1); 
        ekfMSG_.pose.pose.position.z = 0.0;
        ekfMSG_.twist.twist.angular.z = KF_odom(4);   

        ekf_pub_.publish(ekfMSG_);  

        //Update the transform's origin with the new pose
        transform_.setOrigin(tf::Vector3(KF_odom(0),KF_odom(1) , 0.0));
        //Transform to Vehicle frame via tf
        q_.setRPY(0,0, KF_odom(4));
        transform_.setRotation(q_);
        // Publish the updated transform
        br_.sendTransform(tf::StampedTransform(transform_, curr_time, "world", "local"));

        prev_time = curr_time;
      }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "EKF"); // Initialize the ROS node with the name odometer
  EKF_pub_sub EKF_pub_sub; 
  ros::spin(); // Keep the node running and processing callbacks
  return 0;
}