#include <beginner_tutorials/AddTwoIntsAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<beginner_tutorials::AddTwoIntsAction> Server;

void execute(const beginner_tutorials::AddTwoIntsGoalConstPtr& goal, Server* as)
{
  // Do lots of awesome groundbreaking robot stuff here
  beginner_tutorials::AddTwoIntsResult result;
  ROS_INFO("received %ld %ld", goal->a, goal->b);
  result.sum = goal->a + goal ->b;
  ROS_INFO("returns %ld", result.sum);
  as->setSucceeded(result);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;
  Server server(n, "add_two_ints", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
