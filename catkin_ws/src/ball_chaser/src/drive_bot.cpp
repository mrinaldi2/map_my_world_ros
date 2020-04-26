#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include "geometry_msgs/Twist.h"

ros::Publisher motor_command_publisher;

void publish_message(float linear_x, float angular_z) {
    geometry_msgs::Twist motor_command;

    motor_command.linear.x = linear_x;
    motor_command.angular.z = angular_z;

    motor_command_publisher.publish(motor_command);
}

// This callback function executes whenever a safe_move service is requested
bool handle_drive_to_target_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{

    ROS_INFO("DriveToTargetRequest received - lx:%1.2f, az:%1.2f", (float)req.linear_x, (float)req.angular_z);

    publish_message((float)req.linear_x, (float)req.angular_z);

    // Wait 3 seconds for arm to settle
    ros::Duration(3).sleep();

    // Return a response message
    res.msg_feedback = "Linear velocity set - x: " + std::to_string(req.linear_x) + " , z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "drive_bot");

    ros::NodeHandle n;

    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_to_target_request);

    ros::spin();
    return 0;
}