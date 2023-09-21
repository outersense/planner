#include <iostream>
#include "TrajectoryPlannerRos.h"
#include "ros/ros.h"
#include "ros/timer.h"

TrajectoryPlan::TrajectoryPlan(double *x_wp, double *y_wp, double *velocity, int size)
{
    int interpPoints = 10;
    this->size = size;

    waypoints = new double *[interpPoints];
    for (int i = 0; i < size; i++)
    {
        double xstep = (x_wp[i + 1] - x_wp[i]) / interpPoints;
        double ystep = (y_wp[i + 1] - y_wp[i]) / interpPoints;
        double velstep = (velocity[i + 1] - velocity[i]) / interpPoints;

        for (int j = 0; j < interpPoints; ++j)
        {

            waypoints[i] = new double[3]; // 3 values for x, y, and velocity
            waypoints[j][0] = x_wp[i] + j * xstep;
            waypoints[j][1] = y_wp[i] + j * ystep;
            waypoints[j][2] = velocity[i] + j * velstep;
        }
    }
    this->size = size * interpPoints;
    ros::Nodehandle nh;

    // ros::Publisher vehicleTrajectoryPub = n.advertise<planner::waypoints>("/vehicle_trajectory_control", 1000);
    // ros::Subscriber vehiclePoseSub = n.subscribe("rccar_pose", 1000, rcCarPoseCallback);
    ros::Timer planTime = nh.createTimer(ros::Duration(0.1), planTimerCallback);
}

void TrajectoryPlan::planTimerCallback(const ros::TimerEvent &event)
{
    for (int i = 0; i < size; ++i)
    {
        double x, y, velocity;
        waypoints.getWaypoints(i, x, y, velocity);
        std::cout << "Index " << i << ": ";
        std::cout << "X Position = " << x << ", ";
        std::cout << "Y Position = " << y << ", ";
        std::cout << "Velocity = " << velocity << std::endl;
    }
}
void TrajectoryPlan::getWaypoints(int index, double &x, double &y, double &velocity) const
{
    x = waypoints[index][0];
    y = waypoints[index][1];
    velocity = waypoints[index][2];
}

void TrajectoryPlan::getInterpWaypoints(int index, double &x, double &y, double &velocity) const
{
    x = waypoints[index][0];
    y = waypoints[index][1];
    velocity = waypoints[index][2];
}

int TrajectoryPlan::getSize() const
{
    return size;
}