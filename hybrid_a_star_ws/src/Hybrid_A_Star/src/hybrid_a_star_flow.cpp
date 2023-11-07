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

#include "hybrid_a_star/hybrid_a_star_flow.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <fstream>
#include <string>
#include <cmath>


double Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}
bool scale_100 = false;

HybridAStarFlow::HybridAStarFlow(ros::NodeHandle &nh) {
    double steering_angle = nh.param("planner/steering_angle", 10);
    int steering_angle_discrete_num = nh.param("planner/steering_angle_discrete_num", 1);
    double wheel_base = nh.param("planner/wheel_base", 1.0);
    double segment_length = nh.param("planner/segment_length", 1.6);
    int segment_length_discrete_num = nh.param("planner/segment_length_discrete_num", 8);
    double steering_penalty = nh.param("planner/steering_penalty", 1.05);
    double steering_change_penalty = nh.param("planner/steering_change_penalty", 1.5);
    double reversing_penalty = nh.param("planner/reversing_penalty", 2.0);
    double shot_distance = nh.param("planner/shot_distance", 5.0);

    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance
    );
    costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(nh, "/map", 1);
    // costmap_sub_ptr_ = std::make_shared<CostMapSubscriber>(nh, "/updated_map", 1);
    // std::cout << "this is the map pointer" << std::endl;
    // std::cout << *costmap_sub_ptr_<< std::endl;
    // costmap_sub_ptr_->PrintCostMapData();
    // init_pose_sub_ptr_ = std::make_shared<InitPoseSubscriber2D>(nh, "/initialpose", 1);
    init_pose_sub_ptr_ = std::make_shared<InitPoseSubscriber2D>(nh, "/car2/planner_curr_pos", 1);
    goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(nh, "/car2/planner_goal_pos", 1);

    obstacle_pose_sub_ptr_ = std::make_shared<ObstaclePoseSubscriber2D>(nh, "/rccar_pose", 1);
    
    // obstacle_subscriber = nh.subscribe("/rccar_pose", 10, &HybridAStarFlow::obstacleCallback, this);
    // goal_pose_sub_ptr_ = std::make_shared<GoalPoseSubscriber2D>(nh, "/move_base_simple/goal", 1);

    path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 1);
    path_pub_os = nh.advertise<nav_msgs::Path>("/car2/planned_path", 1);
    searched_tree_pub_ = nh.advertise<visualization_msgs::Marker>("searched_tree", 1);
    vehicle_path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("vehicle_path", 1);
    
    has_map_ = false;


    // std::vector<double> theta_values;



    double x, y, theta;
    std::string filename = "/home/dhanesh/Masters/OuterSense/Planning_new/planner/hybrid_a_star_ws/src/Hybrid_A_Star/src/waypoints_Nov6_1bag_cheat.txt";
    std::ifstream file(filename);
    if (file.is_open()) {
        std::string line;
        while (file >> x >> y >> theta) {

            x_values.push_back(x);
            y_values.push_back(y);
            theta_values.push_back(theta);
            // std::cout << line << std::endl; 
            // std::cout<<"read the file"<<std::endl;
        }

        file.close();  // Close the file when done
    } else {
        std::cerr << "Failed to open the file: " << filename << std::endl;
    }

    // std::cout<<x_values[1]<<std::endl;

}

// void HybridAStarFlow::obstacleCallback()
// int count_ddddd =0;
void HybridAStarFlow::Run() {
    kinodynamic_astar_searcher_ptr_->Reset();
    ReadData();
    int i_made_obstacles = 0;
    if (!has_map_) {
        if (costmap_deque_.empty()) {
            return;
        }

        current_costmap_ptr_ = costmap_deque_.front();
        costmap_deque_.pop_front();

        const double map_resolution = 0.2;
        kinodynamic_astar_searcher_ptr_->Init(
                current_costmap_ptr_->info.origin.position.x,
                1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution,
                current_costmap_ptr_->info.origin.position.y,
                1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution,
                current_costmap_ptr_->info.resolution,
                map_resolution
        );

        unsigned int map_w = std::floor(current_costmap_ptr_->info.width / map_resolution);
        unsigned int map_h = std::floor(current_costmap_ptr_->info.height / map_resolution);
        // std::cout<<"scaled width" << map_w << "          scaled height "<< map_h <<std::endl;
        for (unsigned int w = 0; w < map_w; ++w) {
            for (unsigned int h = 0; h < map_h; ++h) {
                auto x = static_cast<unsigned int> ((w + 0.5) * map_resolution
                                                    / current_costmap_ptr_->info.resolution);
                auto y = static_cast<unsigned int> ((h + 0.5) * map_resolution
                                                    / current_costmap_ptr_->info.resolution);

                if (current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x]) {
                    kinodynamic_astar_searcher_ptr_->SetObstacle(w, h); 
                }
            }
        }
        has_map_ = true;
    }
    if (vals_accessed.size() != 0){
        // std::cout<<"need to do something here"<< std::endl;
        const double map_resolution = 0.2;
        for (size_t i = 0; i < vals_accessed.size(); i += 2) {
            unsigned int x_pls = vals_accessed[i];
            unsigned int y_pls = vals_accessed[i + 1];

            unsigned int bhagwan_k_bharose_x = std::floor(x_pls/map_resolution);
            unsigned int bhagwan_k_bharose_y = std::floor(y_pls/map_resolution);
            // std::cout<< x_pls << "                        "<< y_pls << std::endl;
            // kinodynamic_astar_searcher_ptr_->SetObstacle(x_pls, y_pls);
            kinodynamic_astar_searcher_ptr_->SetObstacle(bhagwan_k_bharose_x, bhagwan_k_bharose_y);
            i_made_obstacles=1;
        }
    }
    






    // kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
    costmap_deque_.clear();
    
    
    while (HasStartPose() && HasGoalPose()) {
        // std::cout<<"#################"<< count_ddddd <<std::endl;
        InitPoseData();

        double start_yaw = tf::getYaw(current_init_pose_ptr_->pose.pose.orientation);
        double goal_yaw = tf::getYaw(current_goal_pose_ptr_->pose.orientation);

        Vec3d start_state = Vec3d(
                current_init_pose_ptr_->pose.pose.position.x,
                current_init_pose_ptr_->pose.pose.position.y,
                start_yaw
        );
        Vec3d goal_state = Vec3d(
                current_goal_pose_ptr_->pose.position.x,
                current_goal_pose_ptr_->pose.position.y,
                goal_yaw
        );
        int variation_id = kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state);
        if (variation_id !=0) {
            if (variation_id ==1){
                auto path = kinodynamic_astar_searcher_ptr_->GetPath();
                
                PublishPath(path);
                PublishPathOutersense(path);
                if (scale_100 == true){
                    PublishVehiclePath(path, 30.0, 20.0, 20u);
                }
                else{
                    PublishVehiclePath(path, 3.0, 2.0, 5u);
                }
                
                
                PublishSearchedTree(kinodynamic_astar_searcher_ptr_->GetSearchedTree());
            }
            else if(variation_id == 9){
                std::cout<< "i am in interpolate"<< std::endl;
                //  change the current position vec3d to nearest position vec3d
                // std::cout<<"finding neigbor"<<std::endl;
                // std::cout<< "previous: "<<start_state[0]<<std::endl;

                // start_state = FindNearestNeighbor(start_state,x_values,y_values);
                std::cout<<"before finding nearest neighbour "<< std::endl;
                std::cout<<"xval is "<< start_state[0]<<std::endl;
                std::cout<<"yval is "<< start_state[1]<<std::endl;
                std::cout<<"thetaval is "<< start_state[2]<<std::endl;
                Vec3d new_start_state = FindNearestNeighbor(start_state,x_values,y_values, theta_values);
                std::cout<<std::endl;
                std::cout<<"AFTER finding nearest neighbour "<< std::endl;
                std::cout<<"xval is "<< new_start_state[0]<<std::endl;
                std::cout<<"yval is "<< new_start_state[1]<<std::endl;
                std::cout<<"thetaval is "<< new_start_state[2]<<std::endl;
                std::cout<<std::endl;
                std::cout<<std::endl;

                std::cout<<std::endl;
                std::cout<<std::endl;

                // std::cout<<new_init[0];

                // std::cout<<"after: "<<start_state[0]<<std::endl;


                
                int variation_id_2 = kinodynamic_astar_searcher_ptr_->Search(new_start_state, goal_state);
                if (variation_id_2 !=0) {
                    if (variation_id_2 ==1){
                        auto path = kinodynamic_astar_searcher_ptr_->GetPath();
                        PublishPath(path);
                        PublishPathOutersense(path);
                        if (scale_100 == true){
                            PublishVehiclePath(path, 30.0, 20.0, 20u);
                        }
                        else{
                            PublishVehiclePath(path, 3.0, 2.0, 5u);
                        }
                // path = InterpolatePath(start_state, goal_state, 1000.0);
                // std::cout<< "i am here now"<< std::endl;
                // PublishPath(path);
                // PublishPathOutersense(path);
                    }
                else if (variation_id_2 ==9 ){std::cout<<"back in interpolate"<<std::endl;}
                }
                else{std::cout<<"could not find anything yaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaarrrrrr"<<std::endl;}
            }

            // nav_msgs::Path path_ros;
            // geometry_msgs::PoseStamped pose_stamped;

            // for (const auto &pose: path) {
            //     pose_stamped.header.frame_id = "world";
            //     pose_stamped.pose.position.x = pose.x();
            //     pose_stamped.pose.position.y = pose.y();
            //     pose_stamped.pose.position.z = 0.0;

            //     pose_stamped.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.z());

            //     path_ros.poses.emplace_back(pose_stamped);
            // }

            // path_ros.header.frame_id = "world";
            // path_ros.header.stamp = ros::Time::now();
            // static tf::TransformBroadcaster transform_broadcaster;
            // for (const auto &pose: path_ros.poses) {
            //     tf::Transform transform;
            //     transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0));

            //     tf::Quaternion q;
            //     q.setX(pose.pose.orientation.x);
            //     q.setY(pose.pose.orientation.y);
            //     q.setZ(pose.pose.orientation.z);
            //     q.setW(pose.pose.orientation.w);
            //     transform.setRotation(q);

            //     transform_broadcaster.sendTransform(tf::StampedTransform(transform,
            //                                                              ros::Time::now(), "world",
            //                                                              "ground_link")
            //     );

            //     ros::Duration(0.05).sleep();
            // }
        }
        else{
            // interpolate
            std::cout<< "##############################did not find path#################################"<< std::endl;
            // auto path = InterpolatePath(start_state, goal_state, 1000.0);
            // std::cout<< "i am here now"<< std::endl;
            // PublishPath(path);
            // PublishPathOutersense(path);

        }


        // debug
//        std::cout << "visited nodes: " << kinodynamic_astar_searcher_ptr_->GetVisitedNodesNumber() << std::endl;
        kinodynamic_astar_searcher_ptr_->Reset();
        // std::cout<< "############## end "<<count_ddddd <<std::endl;
        // count_ddddd = count_ddddd+1;

    }
    if (i_made_obstacles != 0){
        // std::cout<<"need to remove obstacles"<< std::endl;
        const double map_resolution = 0.2;
        for (size_t i = 0; i < vals_accessed.size(); i += 2) {
            unsigned int x_pls = vals_accessed[i];
            unsigned int y_pls = vals_accessed[i + 1];

            unsigned int bhagwan_k_bharose_x = std::floor(x_pls/map_resolution);
            unsigned int bhagwan_k_bharose_y = std::floor(y_pls/map_resolution);
            // std::cout<< x_pls << "                        "<< y_pls << std::endl;
            // kinodynamic_astar_searcher_ptr_->SetObstacle(x_pls, y_pls);
            kinodynamic_astar_searcher_ptr_->RemoveObstacle(bhagwan_k_bharose_x, bhagwan_k_bharose_y);
            i_made_obstacles=0;
        }
    }
}

// VectorVec3d HybridAStarFlow::InterpolatePath(const Vec3d& start_state, const Vec3d& goal_state, double step_size){
//     VectorVec3d path;
//     std::cout<< "i am here"<< std::endl;
//     Vec3d diff = goal_state - start_state;
//     double distance = std::sqrt(diff.x()*diff.x() + diff.y()*diff.y() +diff.z()*diff.z());
//     Vec3d direction;
//     if (distance >0){
//         direction = diff/distance;
//         std::cout<< "kya karu yaar"<< direction << std::endl<< "  "<< diff<< std::endl<< step_size/distance<<std::endl;
//     }
//     else{
//         direction = Vec3d(0,0,0);
        
//     }
//     for (double t=0.0; t<=1.0; t+= distance/step_size){
//         Vec3d interpolated_state = start_state + t*direction;
//         path.push_back(interpolated_state);
//         std::cout<< "                                                    "<<path.size() << std::endl;
//     }
    
//     return path;

// }

// Vec3d HybridAStarFlow::FindNearestNeighbor(Vec3d start_state, const std::vector<double>& x_values, const std::vector<double>& y_values) {

Vec3d HybridAStarFlow::FindNearestNeighbor(Vec3d start_state, const std::vector<double>& x_values, const std::vector<double>& y_values, const std::vector<double>& theta_values) {
    if (x_values.empty() || y_values.empty() || x_values.size() != y_values.size()) {
        // Handle invalid input or mismatched vector sizes
        return start_state;
    }

    double min_distance = std::numeric_limits<double>::max();
    size_t nearest_index = 0;

    for (size_t i = 0; i < x_values.size(); ++i) {
        double dx = start_state[0] - x_values[i];
        double dy = start_state[1] - y_values[i];
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            nearest_index = i;
        }
    }

    Vec3d nearest_neighbor;
    nearest_neighbor[0] = x_values[nearest_index];
    nearest_neighbor[1] = y_values[nearest_index];
    nearest_neighbor[2] = theta_values[nearest_index]; // Assuming z-coordinate remains the same

    return nearest_neighbor;
}










VectorVec3d HybridAStarFlow::InterpolatePath(const Vec3d& start_state, const Vec3d& goal_state, double step_size){
    VectorVec3d path;
    std::cout<< "i am here"<< std::endl;
    Vec3d diff = goal_state - start_state;
    Vec3d temp_path;
    double distance = diff.norm();
    std::cout<< "            " << distance << std::endl;
    // if (distance>0)
    if (distance<1e-6){
        path.push_back(start_state);
    }

    Vec3d direction = diff.normalized();
    // if (distance >0){
    //     direction = diff/distance;
    //     std::cout<< "kya karu yaar"<< direction << std::endl<< "  "<< diff<< std::endl<< step_size/distance<<std::endl;
    // }
    // else{
    //     direction = Vec3d(0,0,0);
        
    // for (j = 0; j < numofDOFs; j++){
    //     if(distance < fabs(q_near[j] - qrand[j]))
    //         distance = fabs(q_near[j] - qrand[j]);
    // }



    int numofsamples = (int)(distance/0.2);
    for (int i = 0; i < numofsamples/1; i++){
        for(int j = 0; j < 3; j++){
            temp_path[j] = start_state[j] + ((double)(i)/(numofsamples-1))*(goal_state[j] - start_state[j]);
            
        }
        path.push_back(temp_path);
        // if(IsValidArmConfiguration(joint_angles, numofDOFs, map, x_size, y_size)) {
		// 	std::vector<double> valid_angles(joint_angles, joint_angles + numofDOFs);
		// 	double temp_dist = get_distance(q_near, valid_angles);
		// 	if ( temp_dist< Eu_dist){
		// 		if (int_dist< temp_dist){
		// 			int_dist = temp_dist;
		// 			q_new = valid_angles;
		// 		}

		// 	}
        // }
		// else{
		// 	if(q_new.empty()){
		// 		// std::cout<< "first step was only invalid"<< std::endl;
		// 		q_new = q_near;
		// 		break;
		// 		}
		// 	else{
		// 		// std::cout<< "some step was invalid exiting the for loop"<< std::endl;
		// 		break;
		// 		}
		// 	}
    }    




    // for (double t=0.0; t<=1.0; t+= distance/step_size){
    //     Vec3d interpolated_state = start_state + t*direction;
    //     path.push_back(interpolated_state);
    //     // std::cout<< "                                                    "<<path.size() << std::endl;


    // }

    // path.push_back(goal_state);
    // std::cout<<"jash debugging statement                  "<<std::endl;
    
    return path;

}

VectorVec3d HybridAStarFlow::InterpolateSpline(const Vec3d& start_state, const Vec3d& goal_state, double step_size, double car_radius) {
    VectorVec3d path;
    std::cout << "i am in spline" << std::endl;
    Vec3d diff = goal_state - start_state;
    double distance = diff.norm();
    // std::cout << "            " << distance << std::endl;

    if (distance < 1e-6) {
        path.push_back(start_state);
        // return path;  // Early exit for cases where start and end are the same
    }

    // Calculate the maximum number of segments (path points) based on the turning radius
    double max_segments = distance / (2.0 * car_radius);
    int numofsamples = std::min(static_cast<int>(max_segments), 100);  // Limit the number of segments

    Eigen::MatrixXd control_points(3, 2);
    control_points.col(0) = start_state;
    control_points.col(1) = goal_state;

    // Generate the spline coefficients
    Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(4, 3);
    coefficients.row(0) = control_points.col(0);
    coefficients.row(3) = control_points.col(1);

    // Calculate intermediate control points
    for (int i = 1; i < 3; i++) {
        coefficients.row(i) = control_points.col(0) + i * (control_points.col(1) - control_points.col(0)) / 3.0;
    }

    for (int i = 0; i < numofsamples; i++) {
        double t = static_cast<double>(i) / (numofsamples - 1);
        Eigen::VectorXd t_powers(4);
        t_powers << 1.0, t, t * t, t * t * t;
        Eigen::VectorXd interpolated_state = t_powers.transpose() * coefficients;
        path.push_back(interpolated_state);
    }

    return path;
}

void HybridAStarFlow::ReadData() {
    costmap_sub_ptr_->ParseData(costmap_deque_);
    init_pose_sub_ptr_->ParseData(init_pose_deque_);
    goal_pose_sub_ptr_->ParseData(goal_pose_deque_);
    // ParseData for obstacle
    // goal_pose_sub_ptr_->x, goal_pose_sub_ptr_-y, 
    // std::vector<double> vals = obstacle_pose_sub_ptr_->getX_Y_Vals();
    vals_accessed = obstacle_pose_sub_ptr_->getX_Y_Vals();
    // std::cout<< "size of the vector that has been made is "<< vals_accessed.size() <<std::endl;
}

void HybridAStarFlow::InitPoseData() {
    current_init_pose_ptr_ = init_pose_deque_.front();
    init_pose_deque_.pop_front();

    current_goal_pose_ptr_ = goal_pose_deque_.front();
    goal_pose_deque_.pop_front();
}

bool HybridAStarFlow::HasGoalPose() {
    // std::cout<< "goal pose " << init_pose_deque_.empty() << std::endl;
    return !goal_pose_deque_.empty();
}

bool HybridAStarFlow::HasStartPose() {
    // std::cout<< "init pose " << init_pose_deque_.empty() << std::endl;
    return !init_pose_deque_.empty();
}

void HybridAStarFlow::PublishPath(const VectorVec3d &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);
}
void HybridAStarFlow::PublishPathOutersense(const VectorVec3d &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        int scale_factor = 1;
        double translate_x = 0.0;
        double translate_y = 0.0;
        if (scale_100 == true){
            scale_factor = 100;
            translate_x = -30.419910440369464;
            translate_y = 29.681722692357146;
        }
        else{
            scale_factor = 10;
            translate_x = -3.0419910440369464;
            translate_y = 2.9681722692357146;
        }
        
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = (pose.x()- translate_x)/scale_factor;
        pose_stamped.pose.position.y = (pose.y()- translate_y)/scale_factor;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    path_pub_os.publish(nav_path);
}

void HybridAStarFlow::PublishVehiclePath(const VectorVec3d &path, double width,
                                         double length, unsigned int vehicle_interval = 25u) {
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::Marker vehicle;

        if (i == 0) {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "world";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = width;
        vehicle.scale.y = length;
        vehicle.scale.z = 0.01;
        vehicle.color.a = 0.1;

        vehicle.color.r = 1.0;
        vehicle.color.b = 0.0;
        vehicle.color.g = 0.0;

        vehicle.pose.position.x = path[i].x();
        vehicle.pose.position.y = path[i].y();
        vehicle.pose.position.z = 0.0;

        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        vehicle_array.markers.emplace_back(vehicle);
    }

    vehicle_path_pub_.publish(vehicle_array);
}

void HybridAStarFlow::PublishSearchedTree(const VectorVec4d &searched_tree) {
    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "world";
    tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::Point point;
    for (const auto &i: searched_tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }

    searched_tree_pub_.publish(tree_list);
}