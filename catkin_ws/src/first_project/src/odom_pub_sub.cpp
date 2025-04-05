#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

class OdomPubSub {
    private:
        //ROS node handle
        ros::NodeHandle nh_;

        //Create transform broadcaster 
        tf::TransformBroadcaster br_;
        // Declare transform and quaternion as class members to reuse them in each callback
        tf::Transform transform_; //Store transform messages
        tf::Quaternion q_; //Message in the form of a quaternion
        // Subscriber for "/speedsteer" topic
        ros::Subscriber speedsteer_sub_;
        // Publisher for "/odom" topic
        ros::Publisher odom_pub_;
        // Publisher for "/tf-odom-vehicle" topic
        ros::Publisher tf_odom_pub_;
        // Subscribing speedsteer
        geometry_msgs::PointStamped speedsteerMSG_;
        // Publishing odometry
        nav_msgs::Odometry odomMSG_;

        // Intialize Odometry Variables
        

    public:
        // Constructor: sets up subscriber and publishers
        OdomPubSub(){

            // Subscribe to "/speedsteer" topic 
            speedsteer_sub_ = nh_.subscribe("/speedsteer", 1, &OdomPubSub::speedsteerCallback,this);

            // Advertise the publisher on "/odom" topic
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
            
            // Advertiste the publisher on the "/tf_odom" topic
            tf_odom_pub_ = nh_.advertise<geometry_msgs::("tf odom-vehicle",10);
        }

        //Call back function 
        void speedsteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
            //1. Extract 




            // Intialize 

            q.setRPY(0, 0, msg->theta);
            transform.setRotation(q);
            // Publish the updated transform
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "turtle"));
        }


};

