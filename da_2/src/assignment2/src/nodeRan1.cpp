#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main (int argc, char **argv) 
{
    ros::init(argc, argv, "nodeRan1");
	ros::NodeHandle n;

    srand(time(0)); //seed ran func

    ros::Publisher pubVel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //topic /cmd_vel

    ros::Rate loopr(10); //0.005hz

    while (ros::ok())
    {
        geometry_msgs::Twist vel;

        vel.linear.x = -10 + 20*double(rand()) / double(RAND_MAX);
        vel.angular.z = -10 + 20*double(rand()) / double(RAND_MAX);

        pubVel.publish(vel);

		ros::spinOnce(); //callbacks enabled
        loopr.sleep(); //resets loopr cycle
    }
	ros::shutdown();
	return 0;
}