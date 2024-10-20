// Task 2 goal_pose testbench
#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include <cstdlib> //srand() & rand()
#include <ctime>   //time()

int main(int argc, char** argv)
{
    /*initialize ROS node*/
    ros::init(argc, argv, "nodeRan");
    ros::NodeHandle n;
    srand(time(0));

    ros::Publisher goal_pub = n.advertise<geometry_msgs::Pose2D>("/goal_pose", 1);

    ros::Rate loopr(0.01);

    while (ros::ok())
    {
        geometry_msgs::Pose2D ran_goal;

        
        ran_goal.x = -5.0 + 10.0 * double(rand()) / double(RAND_MAX);
        ran_goal.y = -5.0 + 10.0 * double(rand()) / double(RAND_MAX);
        ran_goal.theta = -M_PI + 2 * M_PI * double(rand()) / double(RAND_MAX);

        ROS_INFO("Publishing random goal pose: x = %f, y = %f, theta = %f", ran_goal.x, ran_goal.y, ran_goal.theta);
        goal_pub.publish(ran_goal);

        loopr.sleep();
    }
    return 0;
}
