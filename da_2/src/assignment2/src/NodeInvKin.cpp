//Task 2
#include <ros/ros.h>
#include "geometry_msgs/Twist.h" 
#include "geometry_msgs/Pose2D.h"
#include <cmath>    //for math needed for inv kin

double v=0.0, w=0.0; //TODO: label
double x=0.0, y=0.0, th=0.0; //initial pose(0,0,0)
bool goal=false; //flag to check if goal callback
geometry_msgs::Pose2D goal_pose; //store goal pose

/*callback func to get goal_pose from /goal_pose topic*/
void callback(const geometry_msgs::Pose2D& msg)
{
    ROS_INFO("goal_pose: (%f,%f,%f)", msg.x, msg.y, msg.theta);
    goal_pose = msg;
    goal=true; //true when goal_pose is received
}

/*func to find the dist between current and goal pose*/
double goal_dist()
{
    return sqrt(pow(goal_pose.x-x, 2) + pow(goal_pose.y-y, 2)); //Euclid dist
}

/*func to find the angle from current orient to goal*/
double goal_angle()
{
    return atan2(goal_pose.y-y, goal_pose.x-x)-th;
}

int main(int argc, char** argv)
{
    //init ROS node
    ros::init(argc, argv, "nodeInvKin");
    ros::NodeHandle n;

    //sub to /goal_pose, callback goal_pose
    ros::Subscriber goal_sub=n.subscribe("/goal_pose", 1, callback);
    //pub velocity commands to /cmd_vel
    ros::Publisher vel_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //pub current pose to /curent_pose
    ros::Publisher pose_pub=n.advertise<geometry_msgs::Pose2D>("/current_pose", 1);

    geometry_msgs::Twist cmd_vel; //to pub velocity commands
    geometry_msgs::Pose2D current_pose; //to pub current pose

    ros::Time current_time, last_time; //set both times
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate loopr(40); //80Hz loop to update vel & pose
    
    while(ros::ok())
    {
        ros::spinOnce(); //process incoming msgs

        if(goal)
        {
            //find dist & angle to goal
            double dist = goal_dist();
            double angle = goal_angle();

            if(fabs(angle)>0.1) //rotate to goal
            {
                ROS_INFO("Rotating towards goal. Angle to goal: %f", angle);
                v=0.0;
                w=angle > 0 ? 0.5 : -0.5;
            }
            else if(dist > 0.1) //move to goal if dist >0.1
            {
                ROS_INFO("moving towards goal. distance to goal: %f", dist);
                v=0.5;
                w=0.0;
            }
            else if(fabs(goal_pose.theta-th)>0.1) //match final orientation
            {
                ROS_INFO("Adjusting final orientation.");
                v=0.0;
                w=(goal_pose.theta-th) > 0 ? 0.5 : -0.5;
            }
            else //stop robot, done
            {
                ROS_INFO("Goal reached.");
                v=0.0;
                w=0.0;
                goal=false;
            }

            //assign v and w to cmd_vel
            cmd_vel.linear.x = v;
            cmd_vel.angular.z = w;

            //update current pose
            current_time = ros::Time::now();
            double dt = (current_time - last_time).toSec();
            x += v * cos(th) * dt;
            y += v * sin(th) * dt;
            th += w * dt;

            //convert th to deg to pub
            current_pose.x=x;
            current_pose.y=y;
            current_pose.theta = th*(180.0/3.141592653589793238463); //convert rad to deg

            //pub vel & pose_pub
            pose_pub.publish(current_pose);
            vel_pub.publish(cmd_vel);

            ROS_INFO("Current pose: x = %f, y = %f, theta = %f", current_pose.x, current_pose.y, current_pose.theta);
        }
        last_time = current_time;
        loopr.sleep();
    }
    return 0;
}
