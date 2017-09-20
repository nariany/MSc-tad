#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

class TurtleLawnmower
{
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
 public:
 TurtleLawnmower();
 ~TurtleLawnmower();
  void turtleCallback(const turtlesim::Pose::ConstPtr& msg);
};
TurtleLawnmower::TurtleLawnmower()
{
  sub_=nh_.subscribe("turtle1/pose",1,&TurtleLawnmower::turtleCallback,this);
  pub_=nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
}
TurtleLawnmower::~TurtleLawnmower()
{}
void TurtleLawnmower::turtleCallback(const turtlesim::Pose::ConstPtr& msg)
 {
      ROS_INFO("Turtle lawnmower@[%f,%f,%f]",msg->x,msg->y,msg->theta);
      geometry_msgs::Twist turtle_cmd_vel;
      turtle_cmd_vel.linear.x=1;
      turtle_cmd_vel.angular.z=2;
      pub_.publish(turtle_cmd_vel);
 } 
int main(int argc,char **argv){
  ros::init(argc,argv,"turtle_lawnmower_node");
  TurtleLawnmower TtMower;
  ros::spin();
 return 0;
}
