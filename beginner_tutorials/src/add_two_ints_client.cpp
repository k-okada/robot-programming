#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_client");
    if (argc != 3) {
            ROS_INFO("usage: add_two_ints_client X Y");
            return 1;
    }
    ros::NodeHandle n;
    ros::ServiceClient client
       = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
    client.waitForExistence();

    beginner_tutorials::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    ROS_INFO("Call: %ld + %ld", srv.request.a, srv.request.b);
    if (client.call(srv)) {
            ROS_INFO("Sum: %ld", srv.response.sum);
    } else {
            ROS_ERROR("Failed to call service add_two_ints");
            return 1;
    }
    return 0;
}
