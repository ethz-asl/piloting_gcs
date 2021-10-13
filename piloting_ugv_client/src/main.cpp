#include <piloting_ugv_client/client_node.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "piloting_ugv_client");
    if (!ros::master::check()) {
        ROS_ERROR("There is not ros master");
        return -1;
    }
    ros::start();

    piloting_ugv_client::ClientNode client;
    if (!client.init())
        return 0;

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}