#pragma once

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/duration.h"

class RobotStatus
{
    ros::Time time_scanning_started_;
    ros::Time stopping_time_;
    bool is_chasing_;
    bool is_scanning_;
    bool is_stopped_;
    bool is_stopping_time_set_;

public:
    RobotStatus() :
        is_scanning_{false},
        is_stopped_{false},
        is_stopping_time_set_{false},
        is_chasing_{false}
    {
        SetToStop();
    }

    void SetToStop()
    {
        if(not is_stopping_time_set_)
        {
            if(ros::Time::isValid())
            {
                stopping_time_ = ros::Time::now();
                is_stopping_time_set_ = true;
            }
        }
        is_stopped_ = true;
        is_scanning_ = false;
        is_chasing_ = false;
    }

    void SetToScanning()
    {
        if (not is_scanning_)
        {
            is_scanning_ = true;
            time_scanning_started_ = ros::Time::now();
        }
        is_chasing_ = false;
        is_stopped_ = false;
        is_stopping_time_set_ = false;
    }

    void SetToChasing()
    {
        is_chasing_ = true;
        is_stopping_time_set_ = false;
        is_stopped_ = false;
        is_scanning_ = false;
    }

    bool IsStopped() { return is_stopped_; }
    bool IsScanning() { return is_scanning_; }
    bool IsChasing() { return is_chasing_; }
    double TimeStoppedInSeconds()
    {
        if(is_stopping_time_set_)
        {
            ros::Duration time_stopped = ros::Time::now() - stopping_time_;
            return time_stopped.toSec();
        }
        return 0.0;
    }

    double TimeScanningInSeconds()
    {
        if(is_scanning_)
        {
            ros::Duration time_scanning = ros::Time::now() - time_scanning_started_;
            return time_scanning.toSec();
        }
        return 0.0;
    }
};