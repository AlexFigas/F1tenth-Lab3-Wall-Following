using namespace std;

#include "math.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// Constants
float KP = 1.0f;
float KD = 0.001f;
float KI = 0.005f;
double L = 1.0;
double MIN_DIST_RIGHT = 1.0;
double MIN_DIST_LEFT = 1.2;

// Publisher & Subscriber
ros::Subscriber sub;
ros::Publisher pub;

// Variables
int idx_a, idx_b;
double dist_a, dist_b;
double ang_a, ang_b;
double alpha;
double dt, dt1;
double error, prev_error, integral;
double delta_time, last_time;

// Functions
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

    last_time = ros::Time::now().toSec();
}

void callback_scan(sensor_msgs::LaserScan scan)
{
    ROS_INFO("Received scan!");
    ROS_INFO("%f increments with %lu total scans!", scan.angle_increment, scan.ranges.size());

    ackermann_msgs::AckermannDriveStamped msg;

    ang_a = 45.0 * (M_PI / 180.0);
    ang_b = 90.0 * (M_PI / 180.0);

    // Indexes
    idx_a = (int)floor((ang_a - scan.angle_min) / scan.angle_increment); // A knowing that it is 45 degrees
    idx_b = (int)floor((ang_b - scan.angle_min) / scan.angle_increment); // B knowing that it is at 90 degrees

    // Distances
    dist_a = scan.ranges[idx_a];
    dist_b = scan.ranges[idx_b];

    alpha = atan((dist_a * cos(ang_b - ang_a) - dist_b) / (dist_a * sin(ang_b - ang_a)));

    dt = dist_b * cos(alpha);
    dt1 = dt + L * sin(alpha);

    error = MIN_DIST_LEFT - dt1;

    delta_time = ros::Time::now().toSec();

    integral += prev_error * delta_time;

    // PID Algorithm
    msg.drive.steering_angle = -(KP * error + KD * (error - prev_error) / delta_time /* + KI * integral */);

    if (abs(msg.drive.steering_angle) > 20.0 * (M_PI / 180.0))
    {
        msg.drive.speed = 0.5;
    }
    else if (abs(msg.drive.steering_angle) > 10.0 * (M_PI / 180.0))
    {
        msg.drive.speed = 1.0;
    }
    else
    {
        msg.drive.speed = 1.5;
    }

    publish_nav(msg);

    prev_error = error;
    last_time = ros::Time::now().toSec();
}

void publish_nav(ackermann_msgs::AckermannDriveStamped msg)
{
    pub.publish(msg);
}
