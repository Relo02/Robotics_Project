#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

class GPS_sub {
    private:
        // ROS node handle
        ros::NodeHandle nh_;

        // Subscribers for the "/swiftnav/front/gps_pose"
        ros::Subscriber gps_pos_sub;

        // Publisher for the "/rechatter" topic
        ros::Publisher rechatter_pub_;

        //  Pubilishing pose
        sensor_msgs::NavSatFix gps_poseMSG_;

    public:
        // Constructor: sets up subscribers, publisher, and timer
        GPS_sub() {
            // Subscribe to topics with a queue size of 1
            gps_pos_sub = nh_.subscribe("/swiftnav/front/gps_pose", 1, &GPS_sub::gps_poseCallback, this);

            /*// Advertise the publisher on "/rechatter" topic
            rechatter_pub_ = nh_.advertise<std_msgs::String>("/rechatter", 1);*/

            // Create a timer that triggers every 1 second to publish messages
            //timer_ = nh_.createTimer(ros::Duration(1.0), &PubSubNode::timerCallback, this);
        }

        // Callback function for the "/chatter" topic
        void gps_poseCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
            gps_poseMSG_ = *msg;
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
    ros::init(argc, argv, "gps_odometer");
    GPS_sub node;
    ros::spin(); // Keep the node running and processing callbacks
    return 0;
}

