#include <ros/ros.h>
#include <edit_TL/node.hpp>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "edit_TL");
    traffic_light::MapBasedEditor node;
    ros::spin();

    return 0;
}
