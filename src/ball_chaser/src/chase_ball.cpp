#include "ros/ros.h"
#include "ros/time.h"
#include "ros/duration.h"
#include "ball_chaser/DriveToTarget.h"
#include "ball_chaser/HorizontalLocation.h"
#include "ball_chaser/robot_status.hpp"


class BallChaser
{
    ros::ServiceClient drive_srv_client_;
    RobotStatus robot_status_;
    bool is_ball_found_;
    bool are_surroudings_scanned_;

    // Callback for the subscription to /ball_chaser/ball_hor_loc.
    // If ball was found in the image, it commands the drive_bot
    // (through service /ball_chaser/command_robot) to drive towards
    // the ball
    void LocateBallCallback(const ball_chaser::HorizontalLocation& hloc);

    void LookForWhiteBall(double& linear_x, double& angular_z);

public:
    BallChaser() :
        is_ball_found_{false},
        are_surroudings_scanned_{false}
    {}
    // Run the ball chaser
    void Run();
};

void BallChaser::Run()
{
    ros::NodeHandle nodeh;

    // Specify service and type of message of the client
    drive_srv_client_ = nodeh.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /ball_chaser/ball_hor_loc topic to read the horizontal
    // relative location of the ball as seen from the camera
    ros::Subscriber sub_hloc = nodeh.subscribe("/ball_chaser/ball_hor_loc", 10,
                                                &BallChaser::LocateBallCallback,
                                                this);

    // Handle ROS communication events
    ros::spin();
}

void BallChaser::LocateBallCallback(const ball_chaser::HorizontalLocation& hloc)
{
    double linear_x_vel{1.0};
    double angular_z_vel{0.0};
    double max_angular_vel{0.5};
    double turning_threshold = 0.2;
    double turning_threshold_sq = turning_threshold * turning_threshold;
    if(hloc.contains_object)
    {
        if (not is_ball_found_)
        {
            ROS_INFO_STREAM("Camera sees white ball");
            is_ball_found_ = true;
        }
        are_surroudings_scanned_ = false;
        robot_status_.SetToChasing();

        double hor_pos = hloc.horizontal_relative_position;
        double sign = 1.0;
        if (hor_pos < 0.0)
            sign = -1.0;

        if(fabs(hor_pos) > turning_threshold)
        {
            linear_x_vel = 0.5;
            angular_z_vel = max_angular_vel * sign;
        }
        else
        {
            // We want angular_z to be a function of hor_pos that goes
            // smoothly from 0 to max_angular_vel in the interval
            // s[0, turning_threshold].
            // We also would like angular_z to have derivatives
            // in the extremes close to 0, so that:
            //   a) It glues smoothly to the constant function described
            //      in the case hor_pos > turning_threshold
            //   b) For values of hor_pos close enough to 0, there is not
            //      much variation of angular_z, so that there is no rapid
            //      oscillarions and we are as close as driving in straight
            //      line as possible
            //   c) We can mirror for [-turning_threshold, 0]
            // Function f(x) = (1 + 2*x / (x^2 + 1))/2 behaves this exact way
            // in [-1, 1] (with max at 1), so all we need to do is:
            //   1) Multiply f by max_angular_vel
            //   2) substitute x by the right
            //      transformation g(hor_pos) that sends [0, turning_threshold]
            //      to [-1, 1]
            double x = 2.0 * fabs(hor_pos) / turning_threshold - 1.0;
            angular_z_vel = sign * max_angular_vel * (1.0 + 2 * x / (x * x + 1.0)) / 2.0;

            // In the case of the linear velocity on x, we just take a
            // parabole with respect to hor_pos so that less we need to turn,
            // the faster we can move.
            linear_x_vel = 1.0 - 0.5 * hor_pos * hor_pos / turning_threshold_sq;
        }

    }
    else
    {
        if (robot_status_.IsChasing())
            robot_status_.SetToStop();
        LookForWhiteBall(linear_x_vel, angular_z_vel);
    }

    // Request velocities applied to differential drive
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = linear_x_vel;
    srv.request.angular_z = angular_z_vel;

    // Call /ball_chaser/command_robot service
    if (not drive_srv_client_.call(srv))
        ROS_ERROR("Failed to call service /ball_chaser/command_robot");

}

void BallChaser::LookForWhiteBall(double& linear_x, double& angular_z)
{
    const double rotation_std_angular_z_velocity = 0.5;

    // If the robot is static and hasn't been static for more than
    // 2 seconds, it stays static. If it was stopped for longer,
    // we start scanning
    if(robot_status_.IsStopped())
    {
        if (robot_status_.TimeStoppedInSeconds() < 2.0)
        {
            linear_x = 0.0;
            angular_z = 0.0;
            robot_status_.SetToStop();
            return;
        }
        else
        {
            robot_status_.SetToScanning();
        }
    }

    if(not are_surroudings_scanned_)
    {
        linear_x = 0.0;
        angular_z = rotation_std_angular_z_velocity;

        double angle_rotated = robot_status_.TimeScanningInSeconds() * rotation_std_angular_z_velocity;

        if(angle_rotated > 2 * M_PI)
        {
            are_surroudings_scanned_ = true;

        }
    }
    else
    {
        linear_x = 0.0;
        angular_z = 0.0;
        robot_status_.SetToStop();
    }
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "chase_ball");

    // Initialize ball chaser
    BallChaser chaser;

    // Run chaser
    chaser.Run();

    return 0;
}