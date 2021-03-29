#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"   //在devel/include目录中，由AddTwoInts.srv文件自动生成的

bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  //一旦检测到有add_two_ints服务，就调用add函数
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
