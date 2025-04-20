#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <cmath>

class OdomPubSub {
    private:
        //ROS node handle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh;    

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

        //Declare parameter values
        double steering_factor_; // Steering factor
        double d_; // Distance between wheels
        double b_; // Real wheel baseline


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
        double xk_1_filt; // Current x position after filtering
        double yk_1_filt; // Current y position after filtering
        double xk_1_filt_prev = 0.0; // Current x position after filtering
        double yk_1_filt_prev = 0.0; // Current y position after 
        double a_bias = 0.0; // Bias for steering angle
        // double a_bias = 0.0; // Bias for steering angle
        // double total_a = 0.0; // Total steering angle
        // int count = 0; // Counter for steering bias computation 
    public:
        // Constructor: sets up subscriber and publishers
        OdomPubSub() : nh_(), private_nh("~"){

            if (private_nh.getParam("steering_factor", steering_factor_))
            {
              ROS_INFO("Steering Factor: %f", steering_factor_);
            }
            else
            {
              ROS_WARN("Missing required Steering Factor parameter, using default value");
            }

            if (private_nh.getParam("d", d_))
            {
              ROS_INFO("Distance Between Wheels Parameter: %f", d_);
            }
            else
            {
              ROS_WARN("Missing required Distance Between Wheels parameter, using default value");
            }

            if (private_nh.getParam("b", b_))
            {
              ROS_INFO("Wheel Baseline Parameter: %f", b_);
            }
            else
            {
              ROS_WARN("Missing required Wheel Baseline Parameter, using default value");
            }

            // Subscribe to "/speedsteer" topic 
            speedsteer_sub_ = nh_.subscribe("/speedsteer", 1, &OdomPubSub::speedsteerCallback,this);
            //ROS_INFO("Subscriber count on /speedsteer: %d", speedsteer_sub_.getNumPublishers());
            

            // Advertise the publisher on "/odom" topic
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
            
            
        }

        //Call back function 
        void speedsteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
            speedsteerMSG_ = *msg;
            //Extract speed, steer input,and time from msg
            a = speedsteerMSG_.point.x;
            //a  a_bias + a_bias;
            V = speedsteerMSG_.point.y;
            current_time = speedsteerMSG_.header.stamp;
            // count++;
            // total_a += a;
            // a_bias = total_a/count;     
            //ROS_INFO("Steering Angle : %f", a);

          


            //Compute steering angle
            //alpha = steering_factor_/a* M_PI / 180.0;
            alpha = ((a* M_PI / 180.0)-a_bias)/32.0; 

            if (fabs(alpha)<= 1e-6){
              alpha = 1e-6;
            }
            // ROS_INFO("Steering Angle(deg): %f", alpha);
          
            //Compute delta Time
            dT = (current_time.toSec()-last_time.toSec());

            if (dT <= 0) {
              dT = 0.01; // Set a small positive value to avoid division by zero
          }


            //ROS_INFO("Delta Time: %f", dT);

            //Compute omega, Vf, R
            ome = (V/3.6)*(tan(alpha)/d_);
            //V_f = V/3.6;
            V_f = (ome*d_)/sin(alpha);
            //R = d_/tan(alpha);
            //V_f = V_f*3.6; //Convert to km/h
            // ROS_INFO("Vf: %f", V_f);
            // ROS_INFO("Omega: %f", ome);
            //Compute x,y,theta
            //Compute new x,y,theta
            if (fabs(ome) < 1e-6){
                // call runge kutta approximation if w is near zero
                thetak_1 = thetak+ome*dT;
                xk_1 = xk+V_f*dT*cos(thetak+(ome*dT)/2);
                yk_1 = yk+V_f*dT*sin(thetak+(ome*dT)/2);
                //ROS_INFO("Runge x: %f", xk_1);
                //ROS_INFO("Runge y: %f", yk_1);
            }
            else{
                thetak_1 = thetak+ome*dT;
                xk_1 = xk+(V_f/ome)*(sin(thetak_1)-sin(thetak));
                yk_1 = yk-(V_f/ome)*(cos(thetak_1)-cos(thetak));
                //ROS_INFO("Non Runge x: %f", xk_1);
                //ROS_INFO("Non Runge y: %f", yk_1);
            }

            // Smoothing algorithm
            //smoothing_algorithm();

            //Publish x,y,theta on "/odom" topic 
            odomMSG_.header.stamp = ros::Time::now();
            odomMSG_.header.frame_id = "odom";
            odomMSG_.child_frame_id = "vehicle";
            odomMSG_.pose.pose.position.x = -yk_1; 
            odomMSG_.pose.pose.position.y = xk_1; 
            odomMSG_.pose.pose.position.z = 0.0;
            odomMSG_.twist.twist.angular.z = ome;

            //ROS_INFO("x: %f", xk_1);
            //ROS_INFO("y: %f", yk_1);
            // ROS_INFO("theta: %f", thetak_1);
            //Adding velocity and omega optionally 
            // odomMSG_.twist.twist.linear.x = V_f;
            
            odom_pub_.publish(odomMSG_); 

            //Update the transform's origin with the new pose
            transform_.setOrigin(tf::Vector3(-yk_1, xk_1, 0.0));
            //Transform to Vehicle frame via tf
            q_.setRPY(0,0, thetak_1);
            transform_.setRotation(q_);
            // Publish the updated transform
            br_.sendTransform(tf::StampedTransform(transform_, current_time, "odom", "vehicle"));

            //last = current loop
            last_time = current_time;
            thetak = thetak_1;
            xk = xk_1;
            yk = yk_1;

            // xk_1_filt_prev = xk_1_filt;
            // yk_1_filt_prev = yk_1_filt;
            
        }

        // void smoothing_algorithm() {
        //     // to smooth the GPS data before publishing
        //     xk_1_filt = .6* xk_1_filt_prev + (1 - .6) * xk_1;
        //     yk_1_filt= .6 * yk_1_filt_prev + (1 - .6) * yk_1;
        // }


};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometer"); // Initialize the ROS node with the name odometer
    OdomPubSub odometry; 
    ros::spin(); // Keep the node running and processing callbacks
    return 0;
}