#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <cmath>

class OdomPubSub {
    private:
        //ROS node handle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh("~");    

        //Create transform broadcaster 
        tf::TransformBroadcaster br_;
        // Declare transform and quaternion as class members to reuse them in each callback
        tf::Transform transform_; //Store transform messages
        tf::Quaternion q_; //Message in the form of a quaternion
        
        // Subscriber for "/speedsteer" topic
        ros::Subscriber speedsteer_sub_;
        
        // Publisher for "/odom" topic
        ros::Publisher odom_pub_;

        // Subscribing speedsteer
        geometry_msgs::PointStamped speedsteerMSG_;
        // Publishing odometry
        nav_msgs::Odometry odomMSG_;


        // Intialize speedsteer message variables
        double V; //Speed(km/h)
        double a; //Steer at steering angle(deg)
        double alpha; //Steer angle with steering factor(deg)
        ros::Time last_time; //Last time to get deltaT
        ros::Time current_time; // Current time to get deltaT
        double dT; // Delta time
        double ome; //Omega value[rad/s]
        double V_f; //Linear Velocity[m/s]
        double R; //Radius[]
        double xk; // Last x position
        double xk_1; // Current x position 
        double yk; // Last y position 
        double yk_1; // Current y position 
        double thetak; //Last theta value
        double thetak_1; // Current theta value 
    public:
        // Constructor: sets up subscriber and publishers
        OdomPubSub() : nh_(), private_nh_("~"){


            if (!private_nh_.getParam("steering_factor", steering_factor_)) {
                ROS_Warning("Missing required Steering Factor parameter, using default value");
            }
            if (!private_nh_.getParam("d", d_)) {
                ROS_WARNING("Missing required Distance Between Wheels parameter, using default value");
            }
            
            if (!private_nh_.getParam("b", b_)) {
                ROS_ERROR("Missing required Real Wheel Baseline parameter, using default value");
            }
            // Subscribe to "/speedsteer" topic 
            speedsteer_sub_ = nh_.subscribe("/speedsteer", 1, &OdomPubSub::speedsteerCallback,this);

            // Advertise the publisher on "/odom" topic
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
            
            
        }

        //Call back function 
        void speedsteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
            odomMSG_ = *msg;
            //Extract speed, steer input,and time from msg
            a = oomMSG_.point.x;
            V = odomMSG_.point.y;
            ros::Time current_time = msg.header.stamp;

            //Compute steering angle
            alpha = a*steering_factor_* M_PI / 180.0;

            //Compute delta Time
            dT = (current_time-last_time).toSec();

            //Compute omega, Vf, R
            ome = (V*1000/3600)*(tan(alpha)/d_);
            V_f = (ome*d_)/sin(alpha);
            R = d_/tan(alpha);

            //Compute new x,y,theta
            if (ome<-0.1 && ome<0.1){
                // call runge kutta approximation if w is near zero
                thetak_1 = theta_k+ome*dT;
                xk_1 = xk+V_f*dT*cos(thetak+(ome*dT)/2);
                yk_1 = yk+V_f*dT*sin(thetak+(ome*dT)/2);
            }
            else{
                thetak_1 = theta_k+ome*dT;
                xk_1 = xk+V_f/ome*(sin(thetak_1)-sin(thetak));
                yk_1 = yk-V_f/ome*(cos(thetak_1)-cos(thetak));
            }

            //Publish x,y,theta on "/odom" topic 
            odomMSG_.header.stamp = current_time;
            odomMSG_.header.frame_id = "odom";
            odomMSG_.child_frame_id = "vehicle";
            odomMSG_.pose.pose.position.x = xk_1;
            odomMSG_.pose.pose.position.y = yk_1;
            odomMSG_.pose.pose.position.z = 0.0;
            //Adding velocity and omega optionally 
            // odomMSG_.twist.twist.linear.x = V_f;
            // odomMSG_.twist.twist.angular.z = ome;
            odom_pub_.publish(odom); 


            //Transform to Vehicle frame via tf
            q.setRPY(xk_1, yk_1, thetak_1);
            transform.setRotation(q);
            // Publish the updated transform
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "vehicle"));

            //last = current loop
            last_time = current_time;
            thetak = thetak_1;
            xk = xk_1;
            yk = yk_1;




        }


};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometer"); // Initialize the ROS node with the name odometer
    OdomPubSub odometry; 
    ros::spin(); // Keep the node running and processing callbacks
    return 0;
}