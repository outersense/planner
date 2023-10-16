/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H
#define HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <deque>
#include <mutex>
#include <thread>
#include <string>

class CostMapSubscriber {
public:
    CostMapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);

    void ParseData(std::deque<nav_msgs::OccupancyGridPtr> &deque_costmap_msg_ptr);
    

private:
    void MessageCallBack(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr);


private:
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    std::deque<nav_msgs::OccupancyGridPtr> deque_costmap_;

    std::mutex buff_mutex_;


};

#endif //HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H



// #include "costmap_subscriber.h"
// class CostMapSubscriber {
// CostMapSubscriber::CostMapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size)
// {
//     // Initialize your subscriber here
//     subscriber_ = nh.subscribe(topic_name, buff_size, &CostMapSubscriber::MessageCallBack, this);
// }

// void CostMapSubscriber::MessageCallBack(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr)
// {
//     buff_mutex_.lock();
//     deque_costmap_.emplace_back(costmap_msg_ptr);
//     buff_mutex_.unlock();
// }

// void CostMapSubscriber::PrintCostMapData()
// {
//     buff_mutex_.lock();
//     if (!deque_costmap_.empty())
//     {
//         // Access the latest costmap data
//         nav_msgs::OccupancyGridPtr latest_costmap = deque_costmap_.back();

//         // Print or process the data as needed
//         // For example, to print the data, you can access individual elements as follows:
//         for (int i = 0; i < latest_costmap->data.size(); ++i)
//         {
//             // Print the occupancy data at index 'i'
//             int occupancy_value = latest_costmap->data[i];
//             ROS_INFO("Occupancy value at index %d: %d", i, occupancy_value);
//         }
//     }
//     buff_mutex_.unlock();
// }
// };