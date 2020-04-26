#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h" 
#include <sensor_msgs/Image.h>


ros::ServiceClient client;


void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Following the ball");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_bot");
}

void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    /*rosservice call /ball_chaser/command_robot "linear_x: 0.5
        angular_z: 0.0"  # This request should drive your robot forward

        $ rosservice call /ball_chaser/command_robot "linear_x: 0.0
        angular_z: 0.5"  # This request should drive your robot left

$ rosservice call /ball_chaser/command_robot "linear_x: 0.0
angular_z: -0.5"  # This request should drive your robot right
*/
     // Loop through each pixel in the image and check if its equal to the first one
      
    for (int i = 0; i < img.height * img.step; i = i+3) {
        if (img.data[i]  == white_pixel && img.data[i+1]  == white_pixel && img.data[i+2]  == white_pixel) {
            if (i % img.step < img.step/3 ) {
                ROS_INFO_STREAM("Turn the robot right");
                drive_robot(0.0, 0.1);
            } else if ( i % img.step > (img.step - img.step/3)) {
                ROS_INFO_STREAM("Turn the robot left");
                drive_robot(0.0, -0.1);
            } else {
                ROS_INFO_STREAM("Go Forward");
                drive_robot(0.1, 0.0);
            }
            return;
        }
    }

    drive_robot(0.0, 0.0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    ros::spin();

    return 0;
}