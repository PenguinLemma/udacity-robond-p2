#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

#include <string>


class BotDriver
{
    ros::Publisher motor_command_publisher_;

public:
    BotDriver() {}
    // Run the bot driver
    void Run();

    // Handles drive request whenever the service "/ball_chaser/command_robot" is called
    bool PerformDriveRequest(ball_chaser::DriveToTarget::Request& req,
                             ball_chaser::DriveToTarget::Response& res);
};

void BotDriver::Run()
{
    ros::NodeHandle nodeh;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist
    // on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher_ = nodeh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Handle ROS communication events
    ros::spin();
}

bool BotDriver::PerformDriveRequest(ball_chaser::DriveToTarget::Request& req,
                                    ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("DriveToTargetRequest received with velocities:\nlinear_x: %1.2f\nangular_z: %1.2f",
             req.linear_x, req.angular_z);

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    // Set wheel velocities, forward [0.5, 0.0]
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    // Publish angles to drive the robot
    motor_command_publisher_.publish(motor_command);

    // Return a response message
    res.msg_feedback = "Velocities set to:";
    res.msg_feedback += "\n";
    res.msg_feedback += "linear.x: " + std::to_string(req.linear_x);
    res.msg_feedback += "\n";
    res.msg_feedback += "linear.y: 0.0";
    res.msg_feedback += "\n";
    res.msg_feedback += "linear.z: 0.0";
    res.msg_feedback += "\n";
    res.msg_feedback += "angular.x: 0.0";
    res.msg_feedback += "\n";
    res.msg_feedback += "angular.y: 0.0";
    res.msg_feedback += "\n";
    res.msg_feedback += "angular.z: " + std::to_string(req.angular_z);

    ROS_INFO_STREAM(res.msg_feedback);
    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Initialize bot driver
    BotDriver driver;

    // Define drive /ball_chaser/command_robot service
    ros::NodeHandle nodeh;
    ros::ServiceServer service = nodeh.advertiseService("/ball_chaser/command_robot",
                                                        &BotDriver::PerformDriveRequest,
                                                        &driver);

    // Run driver
    driver.Run();

    return 0;
}