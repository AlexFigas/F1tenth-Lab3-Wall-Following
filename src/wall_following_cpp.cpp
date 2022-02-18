using namespace std;

#include "math.h";
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

float kp = 0.0f; // TODO
float kd = 0.0f; // TODO
float ki = 0.0f; // TODO

ros::Subscriber sub;
ros::Publisher pub;

void callback_scan(sensor_msgs::LaserScan scan);
void publish_nav(ackermann_msgs::AckermannDriveStamped msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_follow");
    ros::NodeHandle nh;

    // init pub and subs
    sub = nh.subscribe("/scan", 1, callback_scan);
    pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);

    ros::spin();
}

void callback_scan(sensor_msgs::LaserScan scan)
{
    ROS_INFO("Received scan!");
    ROS_INFO("%f increments with %llu total scans!", scan.angle_increment, scan.ranges.size());

    // TODO - calculate PID control and call publish_nav function
    ackermann_msgs::AckermannDriveStamped msg;
    float a, b;
    float alpha, tetha;
    vector<float> ranges = scan.ranges;

    b = ranges[ranges.size() / 2]; // Angle 0 (directly in front of the car)

    for (int i = 0; i < ranges.size(); i++)
    {
        a = ranges[i];
        if (i < ranges.size()/2) {
            tetha = 
        }
        scan.angle_increment * i;
        alpha = atan((a * cos(tetha) - b) / (a * sin(tetha)));

    }

    publish_nav(msg);
}
void publish_nav(ackermann_msgs::AckermannDriveStamped msg)
{
    pub.publish(msg);
}
