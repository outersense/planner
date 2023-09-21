#include "TrajectoryPlannerRos.h"
#include <iostream>
#include "ros/ros.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "planner");

    double x_waypoints[] = {0.0, 2.2, 4.9, 5.4, 4.5, 3.1};
    double y_waypoints[] = {0.5, 1, 0.0, 0.5, 1.5, 1.4};
    double velocity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    int arraySize = sizeof(x_waypoints) / sizeof(x_waypoints[0]); // Calculate the array size

    TrajectoryPlan waypoints(x_waypoints, y_waypoints, velocity, arraySize);
    ros::spin();
    return 0;
}
