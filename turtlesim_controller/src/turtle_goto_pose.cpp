
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Publisher pose_publisher;
ros::Subscriber pose_subscriber;  // to determine the position for turning the robot in an absolute orientation --> in the setDesiredOrientation fn
ros::Subscriber goto_pose_subscriber; //????
turtlesim::Pose turtlesim_pose;

const double PI = 3.14159265359;

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool cloclwise);  //this will rotate the turtle at specified angle from its current angle
double degrees2radians(double angle_in_degrees);
double setDesiredOrientation(double desired_angle_radians); //this will rotate the turtle at an absolute angle, whatever its current angle is
void poseCallback(const turtlesim::Pose::ConstPtr& pose_message);  //Callback fn everytime the turtle pose msg is published over the /turtle1/pose topic.
void goto_poseCallback(const geometry_msgs::Pose2D& goto_pose);
//void goto_poseCallback(const geometry_msgs::Pose2D::ConstPtr& goto_pose);
void timerCallback(const ros::TimerEvent&, ros::Publisher location_pub);
void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);  //this will move robot to goal
double getDistance(double x1, double y1, double x2, double y2);


int main(int argc, char **argv)
{
  // Initiate new ROS node named "talker"
  ros::init(argc, argv, "turtle_goto_pose");
  ros::NodeHandle nh;

  double speed, angular_speed;
  double distance, angle;
  bool isForward, clockwise;


  velocity_publisher = nh.advertise<geometry_msgs::Twist>("/tintin/cmd_vel", 10);
  pose_publisher = nh.advertise<geometry_msgs::Pose2D>("/pose_tt", 10);
  pose_subscriber = nh.subscribe("/tintin/pose", 10, poseCallback); //call poseCallback everytime the turtle pose msg is published over the /tintin/pose topic.
  goto_pose_subscriber = nh.subscribe("/tintin/goto_pose", 10, goto_poseCallback);

  ros::Timer timer = nh.createTimer(ros::Duration(0.016), boost::bind(timerCallback, _1, pose_publisher));

  ros::Rate loop_rate(0.5);

  //  /turtle1/cmd_vel is the Topic name
  //  /geometry_msgs::Twist is the msg type
  ROS_INFO("\n\n\n ********START TESTING*********\n");


  //turtlesim::Pose goal_pose;
  //goal_pose.x = 10;
  //goal_pose.y = 7;
  //goal_pose.theta = 0;
  //moveGoal(goal_pose, 0.01);
  
////
  
  //ros::Publisher waypointPub = node.advertise<geometry_msgs::Pose2D>("tintin/goto_pose", 0);
  //ros::Subscriber pose_sub = nh.subscribe("/tintin/pose", 10, poseCallback);
  //ros::Subscriber goal_sub = nh.subscribe("/goto_pose", 10, goalCallback);
  //?

////


  loop_rate.sleep();


  ros::spin();

  return 0;
}




//
void move(double speed, double distance, bool isForward){
  geometry_msgs::Twist vel_msg;
   //set a random linear velocity in the x-axis
   if (isForward)
     vel_msg.linear.x =abs(speed);
   else
     vel_msg.linear.x =-abs(speed);
   vel_msg.linear.y =0;
   vel_msg.linear.z =0;
   //set a random angular velocity in the y-axis
   vel_msg.angular.x = 0;
   vel_msg.angular.y = 0;
   vel_msg.angular.z =0;

   double t0 = ros::Time::now().toSec();
   double current_distance = 0.0;
   ros::Rate loop_rate(100);
   do{
     velocity_publisher.publish(vel_msg);
     double t1 = ros::Time::now().toSec();
     current_distance = speed * (t1-t0);
     ros::spinOnce();
     loop_rate.sleep();
     //cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
   }while(current_distance<distance);
   vel_msg.linear.x =0;
   velocity_publisher.publish(vel_msg);

}


//
void rotate (double angular_speed, double relative_angle, bool clockwise){

  geometry_msgs::Twist vel_msg;
     //set a random linear velocity in the x-axis
     vel_msg.linear.x =0;
     vel_msg.linear.y =0;
     vel_msg.linear.z =0;
     //set a random angular velocity in the y-axis
     vel_msg.angular.x = 0;
     vel_msg.angular.y = 0;
     if (clockwise)
      vel_msg.angular.z =-abs(angular_speed);
     else
      vel_msg.angular.z =abs(angular_speed);

     double t0 = ros::Time::now().toSec();
     double current_angle = 0.0;
     ros::Rate loop_rate(1000);
     do{
       velocity_publisher.publish(vel_msg);
       double t1 = ros::Time::now().toSec();
       current_angle = angular_speed * (t1-t0);
       ros::spinOnce();
       loop_rate.sleep();
       //cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
     }while(current_angle<relative_angle);
     vel_msg.angular.z =0;
     velocity_publisher.publish(vel_msg);
}

/**
 *  converts angles from degree to radians
 */

double degrees2radians(double angle_in_degrees){
  return angle_in_degrees *PI /180.0;
}

/**
 *  turns the robot to a desried absolute angle
 */
double setDesiredOrientation(double desired_angle_radians)
{
  double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
  //if we want to turn at a perticular orientation, we subtract the current orientation from it
  bool clockwise = ((relative_angle_radians<0)?true:false);
  //cout<<desired_angle_radians <<","<<turtlesim_pose.theta<<","<<relative_angle_radians<<","<<clockwise<<endl;
  rotate (abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
}

/**
 *  callback function to update the pose of the robot
 */

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
  turtlesim_pose.x=pose_message->x;
  turtlesim_pose.y=pose_message->y;
  turtlesim_pose.theta=pose_message->theta;
}


//// to grab pose 2D type
void goto_poseCallback(const geometry_msgs::Pose2D& goto_pose){

  // turtlesim::Pose goal_pose;
  // goal_pose.x = goto_pose->x;
  // goal_pose.y = goto_pose->y;
  // goal_pose.theta = 0;
  // moveGoal(goal_pose, 0.5);




  turtlesim::Pose goal_pose; //?

  goal_pose.x=goto_pose.x;
  goal_pose.y=goto_pose.y;
  goal_pose.theta=0;
  moveGoal(goal_pose, 0.5);
}


void timerCallback(const ros::TimerEvent&, ros::Publisher location_pub){

  geometry_msgs::Pose2D pose;

  pose.x = turtlesim_pose.x;
  pose.y = turtlesim_pose.y;
  pose.theta = turtlesim_pose.theta;

  location_pub.publish(pose); //?
}



///

void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance){
  //We implement a Proportional Controller. We need to go from (x,y) to (x',y'). Then, linear velocity v' = K ((x'-x)^2 + (y'-y)^2)^0.5 where K is the constant and ((x'-x)^2 + (y'-y)^2)^0.5 is the Euclidian distance. The steering angle theta = tan^-1(y'-y)/(x'-x) is the angle between these 2 points.
  geometry_msgs::Twist vel_msg;

  ros::Rate loop_rate(10);
  do{
    //linear velocity
    vel_msg.linear.x = 1.5*getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    //angular velocity
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 2*(atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x)-turtlesim_pose.theta);

    velocity_publisher.publish(vel_msg);

    ros::spinOnce();
    loop_rate.sleep();

  }while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);
  cout<<"end move goal"<<endl;
  vel_msg.linear.x = 0;
  vel_msg.angular.z = 0;
  velocity_publisher.publish(vel_msg);

}

double getDistance(double x1, double y1, double x2, double y2){
  return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}