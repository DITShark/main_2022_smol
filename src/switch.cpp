#include <ros/ros.h>
#include "wiringPi.h"
#include "std_msgs/Bool.h"

#define gpio_pin 26

int main(int argc, char **argv)
{
    ros::init(argc, argv, "switch_test");
    ros::NodeHandle nh;
    wiringPiSetupGpio();
    pullUpDnControl(gpio_pin, PUD_UP);
    ros::Publisher start_pub = nh.advertise<std_msgs::Bool>("startRunning", 1);
    int state = 0;
    int change = 1;
    while (ros::ok() || state == 0)
    {
        state = digitalRead(gpio_pin);
        std_msgs::Bool state_;
        if (state == 0)
        {
            if (change)
            {
                ROS_INFO("*\n");
                change = 0;
            }
            else
            {
                ROS_INFO("**\n");
                change = 1;
            }

            state_.data = true;
        }
        start_pub.publish(state_);
        ros::spinOnce();
    }
}