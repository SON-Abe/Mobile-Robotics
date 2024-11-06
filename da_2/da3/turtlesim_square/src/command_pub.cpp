#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


// Publishes topic /cmd_vel of type Twist

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "command_pub");
    ros::NodeHandle nh;

    // Create a publisher to publish on topic /cmd_vel
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Set up a loop rate (10 Hz)
    ros::Rate loop_rate(10);

    // Create Twist message
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 1.0;  // Set linear speed (can be adjusted)
    move_cmd.angular.z = 0.425; // Set angular speed to 0 initially (rotation managed by turtle_motion)

    while (ros::ok()) {
        // Publish the Twist message to /cmd_vel
        cmd_vel_pub.publish(move_cmd);

        // Log the published values
        ROS_INFO("Publishing cmd_vel: linear.x = %f, angular.z = %f", move_cmd.linear.x, move_cmd.angular.z);

        // Sleep for the remaining loop time
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}