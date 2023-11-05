#include "hybrid_a_star/obstacle_pose_subscriber.h"

ObstaclePoseSubscriber2D::ObstaclePoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(
            topic_name, buff_size, &ObstaclePoseSubscriber2D::MessageCallBack, this
    );
}

void ObstaclePoseSubscriber2D::MessageCallBack(const std_msgs::Float32MultiArray::ConstPtr& obstacle_data) {
    buff_mutex_.lock();
    // goal_poses_.emplace_back(goal_pose_ptr);
    // extract x and y 
    // std::cout<<"i am here"<< std::endl;
    // std::cout<< x_y_vals.size() <<std::endl;
    x_y_vals.clear();
    updateMapWithObstacles(obstacle_data);


    buff_mutex_.unlock();
}

void ObstaclePoseSubscriber2D::updateMapWithObstacles(const std_msgs::Float32MultiArray::ConstPtr& obstacle_data)
    {
        // std::cout<<"i am here now"<< std::endl;
        // buff_mutex_.lock();
        int scale_factor = 100;
        int translate_x = -23;
        int translate_y = 37;
        // std::cout<<"Now i am here"<< std::endl;
        // int scale_factor = 10;
        // int translate_x = -2;
        // int translate_y = 4;

        for (std::vector<float>::size_type i = 0; i < obstacle_data->data.size(); i += 5)
        {
            int id = (obstacle_data->data[i]);
            // std::cout<< "id is "<< id <<"for "<< i << std::endl;
            int x = static_cast<int>(obstacle_data->data[i + 1]*scale_factor)+translate_x;
            int y = static_cast<int>(obstacle_data->data[i + 2]*scale_factor)+translate_y;
            double vel = std::abs(obstacle_data->data[i + 3]);
            if (id < 1000 && vel <= 0.05) // && x >= 0 && x < map_width && y >= 0 && y < map_height)
            {
                x_y_vals.push_back(x);
                x_y_vals.push_back(y);


                for (int pix = 1; pix < 13; pix++){
                    // Update surrounding cells
                    for (int dx = -pix; dx <= pix; dx++) {
                        for (int dy = -pix; dy <= pix; dy++) {
                            int new_x = x + dx;
                            int new_y = y + dy;
                            x_y_vals.push_back(new_x);
                            x_y_vals.push_back(new_y);
                        }
                    }
                }
            }           
        }
        // buff_mutex_.unlock();
    }

std::vector<int> ObstaclePoseSubscriber2D::getX_Y_Vals() const {
    return x_y_vals;
}
// void ObstaclePoseSubscriber2D::ParseData(std::deque<geometry_msgs::PoseStampedPtr> &pose_data_buff) {
//     buff_mutex_.lock();
//     // if (!goal_poses_.empty()) {
//     //     pose_data_buff.insert(pose_data_buff.end(), goal_poses_.begin(), goal_poses_.end());
//     //     goal_poses_.clear();
//     // }
//     // parse and expand
//     buff_mutex_.unlock();
// }



