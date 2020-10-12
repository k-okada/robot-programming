#include <beginner_tutorials/AddTwoIntsAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<beginner_tutorials::AddTwoIntsAction> Client;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_two_ints_client");
    Client client("add_two_ints", true); // true -> don't need ros::spin()
    client.waitForServer();
    beginner_tutorials::AddTwoIntsGoal goal;
    goal.a = 10;
    goal.b = 20;
    // Fill in goal here
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(5.0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("add two int returns %ld", client.getResult()->sum);
    printf("Current State: %s\n", client.getState().toString().c_str());
    return 0;
}

