#ifndef TrajectoryPlanner_H
#define TrajectoryPlanner_H

#include "ros/ros.h"
#include "ros/timer.h"

class TrajectoryPlan
{
public:
    TrajectoryPlan(double *x_wp, double *y_wp, double *velocity, int size);
    // ~TrajectoryPlan();

    void getWaypoints(int index, double &x_wp, double &y_wp, double &velocity) const;
    void getInterpWaypoints(int index, double &x_wp, double &y_wp, double &velocity) const;
    int getSize() const;
    void planTimerCallback(const ros::TimerEvent &event);

private:
    int size;
    double **waypoints;
};

#endif
