#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_position");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/set_position", 1000);

  ros::Rate loop_rate(10);
  int count =0;
  while (ros::ok())
  {
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 2;
    pub.publish(msg);
    ROS_INFO("I'm sending: %s", msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}