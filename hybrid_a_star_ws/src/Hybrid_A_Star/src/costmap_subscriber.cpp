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

#include "hybrid_a_star/costmap_subscriber.h"
// #include <typeinfo>

CostMapSubscriber::CostMapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(topic_name, buff_size, &CostMapSubscriber::MessageCallBack, this);
    // publisher_ = nh.advertise<nav_msgs::OccupancyGrid>("/map", 10);
    // publisher_ = nh.advertise<nav_msgs::OccupancyGrid>("/modified_map", 1);
}

void CostMapSubscriber::MessageCallBack(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr) {
    buff_mutex_.lock();
    deque_costmap_.emplace_back(costmap_msg_ptr);
    buff_mutex_.unlock();
    
    // nav_msgs::OccupancyGrid modified_map = *costmap_msg_ptr;
// Modify the map
    // for(auto& cell : costmap_msg_ptr->data)
    // for(auto& cell : modified_map.data)
    // {
    //     cell = 0; // Set all cells to unknown (-1). Modify as needed.
    // }
    // std::cout<< modified_map << std::endl;
    // // nav_msgs::OccupancyGrid modified_map = *costmap_msg_ptr;
    // publisher_.publish(modified_map);
    // for(auto& cell : *costmap_msg_ptr )
    //     {
    //         cell = -1; // Set all cells to unknown (-1). Modify as needed.
    //     }

}

void CostMapSubscriber::ParseData(std::deque<nav_msgs::OccupancyGridPtr> &deque_costmap_msg_ptr) {
    buff_mutex_.lock();
    if (!deque_costmap_.empty()) {
        deque_costmap_msg_ptr.insert(deque_costmap_msg_ptr.end(),
                                     deque_costmap_.begin(),
                                     deque_costmap_.end()
        );

        deque_costmap_.clear();
    }
    buff_mutex_.unlock();
}