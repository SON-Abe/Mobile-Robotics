#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <turtlesim/Pose.h>
#include <cmath>

ros::Publisher pose_pub;
ros::Publisher vel_pub;
geometry_msgs::Pose2D pose_tt;

enum class MotionState { MOVE_FORWARD, TURN };
MotionState current_state = MotionState::MOVE_FORWARD;

double side_length = 2.0;
double linear_speed = 1.0;
double angular_speed = M_PI / 2;
ros::Time start_time;

void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
    pose_tt.x = msg->x;
    pose_tt.y = msg->y;
    pose_tt.theta = msg->theta;
    pose_pub.publish(pose_tt);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double elapsed = (ros::Time::now() - start_time).toSec();

    linear_speed = msg->linear.x;
    angular_speed = msg->angular.z;

    geometry_msgs::Twist vel_msg;

    if (current_state == MotionState::MOVE_FORWARD) {
        vel_msg.linear.x = linear_speed;
        vel_msg.angular.z = 0.0;

        ROS_INFO("Moving Forward: linear.x = %f", linear_speed);

        if (elapsed >= side_length / linear_speed) {
            current_state = MotionState::TURN;
            start_time = ros::Time::now();  // Reset timer
        }
    } else if (current_state == MotionState::TURN) {
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = angular_speed;

        ROS_INFO("Turning: angular.z = %f", angular_speed);

        if (elapsed >= M_PI / (2 * angular_speed)) {
            current_state = MotionState::MOVE_FORWARD;
            start_time = ros::Time::now();  // Reset timer
        }
    }
    else{
        vel_msg.linear.x = 0.0; // Stop moving
        vel_msg.angular.z = 0.0; // Stop turning
        ROS_WARN("Received zero linear speed, not moving.");
    }
    // Publish the velocity command to move the turtle
    vel_pub.publish(vel_msg);
    ROS_INFO("Publishing cmd_vel: linear.x = %f, angular.z = %f", vel_msg.linear.x, vel_msg.angular.z); 

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_motion");
    ros::NodeHandle nh;

    pose_pub = nh.advertise<geometry_msgs::Pose2D>("/pose_tt", 10);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);  // Publish to /turtle1/cmd_vel
    ros::Subscriber pose_sub = nh.subscribe("/turtle1/pose", 10, poseCallback);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);

    start_time = ros::Time::now();
    ros::spin();
    return 0;
}
